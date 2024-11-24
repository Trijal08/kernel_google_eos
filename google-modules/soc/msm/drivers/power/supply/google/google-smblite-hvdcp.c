/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2023 Google LLC */

/*
Controller state transitions illustrated below in graphviz format:

	digraph {
		note [shape=none label="Note: Every state transitions to OFF upon watch unplug,
		but showing these transitions would cause clutter."]
		OFF -> READY [label="HVDCP request received"]
		READY -> USBIN_ACTIVE_WAIT [label="HVDCP functionality enabled"]
		READY -> OFF [label="unplug"]

		USBIN_ACTIVE_WAIT -> USBIN_SUSPEND_WAIT [label="usbin is active and drawing current"]
		USBIN_SUSPEND_WAIT -> ENTER_IDLE_WAIT [label="usbin suspended"]

		ENTER_IDLE_WAIT -> NEGOTIATION [label="Charger in QC3.0 idle mode"]

		NEGOTIATION -> NEGOTIATION [label="Voltage adjusted"]
		NEGOTIATION -> SETTLE_MONITOR [label="Voltage within target"]
		NEGOTIATION -> ABANDON_RECOVERY_USBIN_PREP [label="PMIC boost=1 || Forced abandon timeout reached", fontcolor=red, color=red]

		SETTLE_MONITOR -> NEGOTIATION [label="Voltage changed"]
		SETTLE_MONITOR -> USBIN_RESUME_WAIT [label="Voltage remained consistent"]

		USBIN_RESUME_WAIT -> USBIN_ACTIVE_WAIT [label="PMIC boost recovery"]
		USBIN_RESUME_WAIT -> DONE [label="usbin resumed && !abandoning"]
		USBIN_RESUME_WAIT -> ABANDONED [label="usbin resumed && abandoning", fontcolor=maroon, color=maroon]
		DONE -> OFF [label="unplug"]

		USBIN_SUSPEND_WAIT -> ABANDON_RECOVERY_USBIN_PREP [label="PMIC boost=1 || Forced abandon timeout reached", fontcolor=red, color=red]
		ENTER_IDLE_WAIT -> ABANDON_RECOVERY_USBIN_PREP [label="PMIC boost=1 || Forced abandon timeout reached", fontcolor=red, color=red]
		SETTLE_MONITOR -> ABANDON_RECOVERY_USBIN_PREP [label="Forced abandon timeout reached", fontcolor=teal, color=teal]

		USBIN_SUSPEND_WAIT -> ABANDON_VIA_5V [label="Abandon timeout reached", fontcolor=maroon, color=maroon]
		ENTER_IDLE_WAIT -> ABANDON_VIA_5V [label="Abandon timeout reached", fontcolor=maroon, color=maroon]
		NEGOTIATION -> ABANDON_VIA_5V [label="Abandon timeout reached", fontcolor=maroon, color=maroon]
		SETTLE_MONITOR -> ABANDON_VIA_5V [label="Abandon timeout reached", fontcolor=maroon, color=maroon]

		ABANDON_VIA_5V -> ABANDON_RECOVERY_USBIN_PREP [label="PMIC boost=1 || Forced abandon timeout reached", fontcolor=red, color=red]
		ABANDONED -> OFF [label="unplug"]

		ABANDON_RECOVERY_USBIN_PREP -> USBIN_RESUME_WAIT [label="PMIC boost=0 && (APSD forced if: (timeout reached || voltage > ceiling))"]
	}
*/

#include <linux/alarmtimer.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include "smblite-shim.h"
#include "smblite-reg.h"
#include "misc/gvotable.h"

#define GOOGLE_HVDCP_VOTER "GOOGLE_HVDCP_VOTER"
#define USBIN_SUSPENDED_MAX_UA 10000
#define HVDCP_PULSE_DELTA_UV 200000
#define NUM_VOLTAGE_READS 15
#define PMIC_USBIN_SUSPENDED_ICL_UA 0
#define PMIC_USBIN_LOWEST_ICL_UA 100000
#define VBUS_MEAS_ERROR_UV 30000
#define USBIN_PREP_WAIT_MS 200
#define SUSPEND_WAIT_MS 50
#define ACTIVE_WAIT_MS 200
#define VOLTAGE_CHECK_RETRY_MS 50
#define WAIT_DISABLED U32_MAX

#define STATES \
	X(OFF) \
	X(READY) \
	X(USBIN_ACTIVE_WAIT) \
	X(USBIN_SUSPEND_WAIT) \
	X(ENTER_IDLE_WAIT) \
	X(NEGOTIATION) \
	X(SETTLE_MONITOR) \
	X(ABANDON_RECOVERY_USBIN_PREP) \
	X(USBIN_RESUME_WAIT) \
	X(DONE) \
	X(ABANDON_VIA_5V) \
	X(ABANDONED)

#define CMDS \
	X(IDLE) \
	X(FORCE_5V) \
	X(INCREMENT) \
	X(DECREMENT)

#define X(cmd) CMD_##cmd,
enum cmd {
	CMDS
	NUM_CMDS
};
#undef X

#define X(state) HVDCP_##state,
enum hvdcp_state {
	STATES
	NUM_HVDCP_STATES
};
#undef X

#define X(state) #state,
static const char *state_str[] = {
	STATES
};
#undef X

#define X(cmd) #cmd,
static const char *cmd_str[] = {
	CMDS
};
#undef X

struct state_data {
	unsigned int elapsed_ms;
	ktime_t last_elapsed_check_kt;

	unsigned int wait_ms;
	ktime_t last_state_work_kt;
};

struct session_data {
	bool pmic_had_boost;
	bool boost_recovery;

	/*
	 * Slow waits are enabled if a charger voltage change occurs while
	 * in SETTLE_MONITOR state. Spacing out voltage increment / decrement
	 * commands seems to help in avoiding extra voltage changes after the
	 * command is sent.
	 */
	bool slow_waits;

	bool abandon;
	unsigned int total_elapsed_ms[NUM_HVDCP_STATES];

	int start_voltage;
};

struct cmd_wait_info {
	u16 typ;

	/* Used when session_data slow_waits == true */
	u16 slow;
};

struct hvdcp {
	struct mutex lock;
	struct notifier_block hvdcp_req_nb;
	struct notifier_block plugin_nb;
	struct notifier_block boost_nb;
	struct workqueue_struct *controller_q;
	struct work_struct controller_work;
	struct alarm controller_alarm;
	enum hvdcp_state state;
	struct state_data state_data;
	struct session_data sesh_data;
	struct cmd_wait_info cmd_waits[NUM_CMDS];
	u16 renegotiation_wait_ms;
	u16 cmd_retry_wait_ms;
	u16 settle_monitor_ms;
	u32 voltage_ceiling;
	u32 abandon_via_5v_timeout_ms;
	u32 forced_abandon_timeout_ms;
	bool enabled;
	struct device *dev;
	struct power_supply *shim_psy;
	struct smblite_shim *smblite_shim;
	struct gvotable_election *usb_icl_votable;
	struct gvotable_election *fake_psy_online_votable;
};

enum usbin_active_type {
	USBIN_ACTIVE,
	USBIN_SUSPENDED,
};

enum abandon_type {
	ABANDON_VIA_5V,	/* Use the QC3.0 5V command before bailing */
	ABANDON_FORCED,	/* Simply exit state machine */
};

enum usb_icl {
	ICL_SUSPEND,
	ICL_LOWEST_NONZERO,
	ICL_UNRESTRICTED,
};

static inline uint16_t get_cmd_wait(struct hvdcp *hvdcp, enum cmd cmd)
{
	return hvdcp->sesh_data.slow_waits
		? hvdcp->cmd_waits[cmd].slow : hvdcp->cmd_waits[cmd].typ;
}

static int run_cmd(struct hvdcp *hvdcp, enum cmd cmd, unsigned int *wait_ms_out)
{
	int ret;

	*wait_ms_out = get_cmd_wait(hvdcp, cmd);

	dev_info(hvdcp->dev, "Running cmd %s\n", cmd_str[cmd]);

	if (smblite_lib_is_boost_en(hvdcp->smblite_shim->chg)) {
		dev_err(hvdcp->dev, "Boost mode en (pre-cmd)\n");
		return -EAGAIN;
	}

	switch(cmd) {
	case CMD_IDLE:
		ret = smblite_lib_force_vbus_voltage(hvdcp->smblite_shim->chg,
						IDLE_BIT);
		break;
	case CMD_FORCE_5V:
		ret = smblite_lib_force_vbus_voltage(hvdcp->smblite_shim->chg,
						FORCE_5V_BIT);
		break;
	case CMD_INCREMENT:
		ret = smblite_lib_dp_pulse(hvdcp->smblite_shim->chg);
		break;
	case CMD_DECREMENT:
		ret = smblite_lib_dm_pulse(hvdcp->smblite_shim->chg);
		break;
	default:
		break;
	};

	/*
	 * Try and catch cases where boost occurs at the same time as command.
	 * If boost is active, the PMIC does not toggle data lines for HVDCP
	 * commands
	 */
	if (smblite_lib_is_boost_en(hvdcp->smblite_shim->chg)) {
		dev_err(hvdcp->dev, "Boost mode en (post-cmd)\n");
		return -EAGAIN;
	}

	return ret;
}

static void set_voltage_max(int no_load_voltage)
{
	struct gvotable_election *election =
		gvotable_election_get_handle("SHIM_VMAX");
	int max = DIV_ROUND_CLOSEST(no_load_voltage, 100000) * 100000;

	if (!election)
		return;

	gvotable_cast_int_vote(election, GOOGLE_HVDCP_VOTER, max, (max != 0));
}

static int set_psy_fake_online(struct hvdcp *hvdcp, bool fake_online)
{
	return gvotable_cast_bool_vote(hvdcp->fake_psy_online_votable,
				GOOGLE_HVDCP_VOTER, fake_online);
}

static int set_usb_icl(struct hvdcp *hvdcp, enum usb_icl icl)
{
	int icl_ua;

	if (icl == ICL_SUSPEND)
		icl_ua = PMIC_USBIN_SUSPENDED_ICL_UA;
	else
		icl_ua = PMIC_USBIN_LOWEST_ICL_UA;

	return gvotable_cast_int_vote(hvdcp->usb_icl_votable,
				GOOGLE_HVDCP_VOTER, icl_ua,
				(icl != ICL_UNRESTRICTED));
}

static bool is_usbin(struct hvdcp *hvdcp, enum usbin_active_type query)
{
	struct smblite_shim *shim = hvdcp->smblite_shim;
	int pmic_icl;
	union power_supply_propval curr_now;
	int ret;

	ret = smblite_lib_get_charge_param(shim->chg,
					&shim->chg->param.icl_stat,
					&pmic_icl);
	if (ret != 0) {
		dev_dbg(hvdcp->dev, "Could not read PMIC ICL (%d)\n", ret);
		return false;
	}

	ret = smblite_lib_get_prop_usbin_current(shim->chg, &curr_now);
	if (ret != 0) {
		dev_dbg(hvdcp->dev, "Could not get USB current (%d)\n", ret);
		return false;
	}

	dev_dbg(hvdcp->dev, "icl:%d current:%d query:%s\n", pmic_icl,
		curr_now.intval,
		(query == USBIN_ACTIVE) ? "active" : "suspend");

	if (query == USBIN_ACTIVE)
		return (pmic_icl != PMIC_USBIN_SUSPENDED_ICL_UA)
			&& (curr_now.intval > USBIN_SUSPENDED_MAX_UA);

	return (pmic_icl == PMIC_USBIN_SUSPENDED_ICL_UA)
		&& (curr_now.intval <= USBIN_SUSPENDED_MAX_UA);
}

static int read_voltage(struct hvdcp *hvdcp, int *voltage_out)
{
	struct smb_charger *chg = hvdcp->smblite_shim->chg;
	union power_supply_propval val;
	int ret;
	int x;
	int avg;
	int minimum = INT_MAX;
	int maximum = 0;

	if (!voltage_out)
		return -EINVAL;

	avg = 0;
	for (x = 0; x < NUM_VOLTAGE_READS; x++) {
		ret = smblite_lib_get_prop_usb_voltage_now(chg, &val);
		if (ret != 0)
			break;
		avg += val.intval;
		minimum = min(minimum, val.intval);
		maximum = max(maximum, val.intval);

		/* Sleep for about a millisecond between samples */
		usleep_range(1000, 1111);
	}

	avg = (avg / NUM_VOLTAGE_READS);

	if (ret == 0) {
		*voltage_out = (avg + VBUS_MEAS_ERROR_UV);
		dev_info(hvdcp->dev, "Voltage: avg:%d min:%d max:%d " \
			"returned (w/ %d meas err):%d (uV)\n",
			avg, minimum, maximum, VBUS_MEAS_ERROR_UV,
			*voltage_out);
	} else {
		*voltage_out = 0;
		dev_err(hvdcp->dev, "Voltage read error: %d\n", ret);
	}

	return ret;
}

static void clear_state_data_locked(struct state_data *data)
{
	data->elapsed_ms = 0;
	data->last_elapsed_check_kt = ktime_get_boottime();

	data->wait_ms = 0;
	data->last_state_work_kt = 0;
}

static void update_elapsed_time_locked(struct hvdcp *hvdcp)
{
	struct session_data *sesh_data = &hvdcp->sesh_data;
	struct state_data *state_data = &hvdcp->state_data;
	ktime_t now = ktime_get_boottime();
	unsigned int delta = ktime_ms_delta(now,
					state_data->last_elapsed_check_kt);

	state_data->elapsed_ms += delta;
	state_data->last_elapsed_check_kt = now;

	sesh_data->total_elapsed_ms[hvdcp->state] += delta;
}

static void update_state_locked(struct hvdcp *hvdcp,
				enum hvdcp_state new_state)
{
	update_elapsed_time_locked(hvdcp);
	hvdcp->state_data.last_state_work_kt = ktime_get_boottime();

	if (new_state != hvdcp->state) {
		dev_info(hvdcp->dev, "State change: %s -> %s\n",
			state_str[hvdcp->state],
			state_str[new_state]);
		hvdcp->state = new_state;
		clear_state_data_locked(&hvdcp->state_data);
	}
}

/* Do not clear elapsed totals so they can be still be inspected after device
 * is taken off-charger
 */
static void reset_state_locked(struct hvdcp *hvdcp)
{
	update_state_locked(hvdcp, HVDCP_OFF);
	alarm_try_to_cancel(&hvdcp->controller_alarm);
	set_psy_fake_online(hvdcp, false);
	set_usb_icl(hvdcp, ICL_UNRESTRICTED);

	hvdcp->sesh_data.pmic_had_boost = false;
	hvdcp->sesh_data.boost_recovery = false;
	hvdcp->sesh_data.slow_waits = false;
	hvdcp->sesh_data.abandon = false;
	hvdcp->sesh_data.start_voltage = 0;

	clear_state_data_locked(&hvdcp->state_data);
	set_voltage_max(0);
}

static bool is_state_relevant_for_abandon_via_5v(enum hvdcp_state state)
{
	switch (state) {
	case HVDCP_USBIN_SUSPEND_WAIT:
	case HVDCP_ENTER_IDLE_WAIT:
	case HVDCP_NEGOTIATION:
	case HVDCP_SETTLE_MONITOR:
		return true;
	default:
		return false;
	}
}

static bool is_state_relevant_for_forced_abandon(enum hvdcp_state state)
{
	return is_state_relevant_for_abandon_via_5v(state)
		|| (state == HVDCP_ABANDON_VIA_5V);
}

static unsigned int accumulated_time_towards_abandon(struct session_data *data,
						enum abandon_type type)
{
	unsigned int total = 0;
	enum hvdcp_state x;

	for (x = HVDCP_OFF; x < NUM_HVDCP_STATES; x++) {
		bool relevant = (type == ABANDON_VIA_5V)
				&& is_state_relevant_for_abandon_via_5v(x);

		relevant |= ((type == ABANDON_FORCED)
				&& is_state_relevant_for_forced_abandon(x));

		if (relevant)
			total += data->total_elapsed_ms[x];
	}

	return total;
}

static bool state_needs_recovery_from_boost(enum hvdcp_state state)
{
	switch (state) {
	case HVDCP_USBIN_SUSPEND_WAIT:
	case HVDCP_ENTER_IDLE_WAIT:
	case HVDCP_NEGOTIATION:
	case HVDCP_ABANDON_VIA_5V:
		return true;
	default:
		return false;
	}
}

static void controller_work(struct work_struct *work)
{
	struct hvdcp *hvdcp = container_of(work, struct hvdcp, controller_work);
	int voltage;
	int delta;
	int ret;
	enum hvdcp_state new_state;
	enum cmd cmd;
	union power_supply_propval plugged;
	struct session_data *sesh_data = &hvdcp->sesh_data;
	struct state_data *state_data = &hvdcp->state_data;
	unsigned int wait_ms;
	unsigned int waited_dur_ms;
	ktime_t now;
	bool is_boost_en;
	unsigned int active_time;
	bool past_abandon_timeout;

rerun_controller:
	dev_dbg(hvdcp->dev, "entry\n");

	is_boost_en = smblite_lib_is_boost_en(hvdcp->smblite_shim->chg);
	dev_dbg(hvdcp->dev, "Boost en: %u\n", is_boost_en);
	wait_ms = WAIT_DISABLED;

	mutex_lock(&hvdcp->lock);
	update_elapsed_time_locked(hvdcp);

	ret = smblite_lib_get_prop_usb_present(hvdcp->smblite_shim->chg,
					&plugged);
	if ((ret != 0) || !plugged.intval) {
		reset_state_locked(hvdcp);
		mutex_unlock(&hvdcp->lock);
		return;
	}

	new_state = hvdcp->state;

	active_time = accumulated_time_towards_abandon(&hvdcp->sesh_data,
						ABANDON_FORCED);
	dev_dbg(hvdcp->dev, "Accumulated time (forced abandon): %u\n",
		active_time);
	past_abandon_timeout = active_time > hvdcp->forced_abandon_timeout_ms;
	if (past_abandon_timeout
		&& is_state_relevant_for_forced_abandon(hvdcp->state)) {
		dev_err(hvdcp->dev, "Force abandon!\n");
		sesh_data->abandon = true;
		new_state = HVDCP_ABANDON_RECOVERY_USBIN_PREP;
		wait_ms = 0;
		goto out;
	}

	active_time = accumulated_time_towards_abandon(&hvdcp->sesh_data,
						ABANDON_VIA_5V);
	dev_dbg(hvdcp->dev, "Accumulated time (abandon via 5V): %u\n",
		active_time);
	past_abandon_timeout = active_time > hvdcp->abandon_via_5v_timeout_ms;
	if (past_abandon_timeout
		&& is_state_relevant_for_abandon_via_5v(hvdcp->state)) {
		dev_err(hvdcp->dev, "Abandon via 5V\n");
		sesh_data->abandon = true;
		new_state = HVDCP_ABANDON_VIA_5V;
		wait_ms = 0;
		goto out;
	}

	sesh_data->pmic_had_boost = sesh_data->pmic_had_boost || is_boost_en;

	now = ktime_get_boottime();
	waited_dur_ms = ktime_ms_delta(now, state_data->last_state_work_kt);
	if (waited_dur_ms < state_data->wait_ms) {
		int corrected_wait_ms = (state_data->wait_ms - waited_dur_ms);
		dev_dbg(hvdcp->dev, "Need to wait %u ms before handling %s",
			corrected_wait_ms, state_str[hvdcp->state]);
		wait_ms = corrected_wait_ms;
		goto out;
	}

	switch (hvdcp->state) {
	case HVDCP_OFF:
		new_state = HVDCP_READY;
		wait_ms = 0;
		break;
	case HVDCP_READY:
		if (!hvdcp->enabled) {
			/*
			 * Nothing to do if functionality is disabled, park
			 * here in case functionality gets enabled.
			 */
			wait_ms = WAIT_DISABLED;
			break;
		}
		set_psy_fake_online(hvdcp, true);
		new_state = HVDCP_USBIN_ACTIVE_WAIT;
		wait_ms = 0;
		break;
	case HVDCP_USBIN_ACTIVE_WAIT:
		if (is_boost_en || is_usbin(hvdcp, USBIN_SUSPENDED)) {
			wait_ms = ACTIVE_WAIT_MS;
			break;
		}

		sesh_data->pmic_had_boost = false;
		sesh_data->boost_recovery = false;
		/*
		 * Suspend the charging path so that:
		 *   - We can read an accurate charger voltage without drops due
		 *     to cable resistance.
		 *   - If the charger boosts to over the voltage ceiling, the
		 *     charging puck won't shut down the eletrical path.
		 *     If the puck's overvoltage threshold is reached, it will
		 *     clamp the voltage and excess power will be dissipated
		 *     in the form of heat. If a significant amount of power is
		 *     dissipated, the generated heat will trip the thermal
		 *     protection and the puck will temporarily shut down.
		 */
		ret = set_usb_icl(hvdcp, ICL_SUSPEND);
		if (ret == 0)
			new_state = HVDCP_USBIN_SUSPEND_WAIT;
		wait_ms = SUSPEND_WAIT_MS;
		break;
	case HVDCP_USBIN_SUSPEND_WAIT:
		if (is_usbin(hvdcp, USBIN_ACTIVE)) {
			wait_ms = SUSPEND_WAIT_MS;
			break;
		}

		if ((sesh_data->start_voltage == 0)
			&& read_voltage(hvdcp,
					&sesh_data->start_voltage) != 0) {
			wait_ms = VOLTAGE_CHECK_RETRY_MS;
			break;
		}

		new_state = HVDCP_ENTER_IDLE_WAIT;
		wait_ms = 0;
		break;
	case HVDCP_ENTER_IDLE_WAIT:
		ret = run_cmd(hvdcp, CMD_IDLE, &wait_ms);
		if (ret == -EAGAIN) {
			sesh_data->pmic_had_boost = true;
			wait_ms = 0;
			break;
		}
		if (ret != 0) {
			wait_ms = hvdcp->cmd_retry_wait_ms;
			break;
		}
		new_state = HVDCP_NEGOTIATION;
		break;
	case HVDCP_NEGOTIATION:
		ret = read_voltage(hvdcp, &voltage);
		if (ret != 0) {
			wait_ms = VOLTAGE_CHECK_RETRY_MS;
			break;
		}
		delta = (hvdcp->voltage_ceiling - voltage);
		if ((delta >= 0) && (delta < HVDCP_PULSE_DELTA_UV)) {
			new_state = HVDCP_SETTLE_MONITOR;
			wait_ms = VOLTAGE_CHECK_RETRY_MS;
			break;
		}
		cmd = (delta > 0) ? CMD_INCREMENT : CMD_DECREMENT;
		ret = run_cmd(hvdcp, cmd, &wait_ms);
		if (ret == -EAGAIN) {
			sesh_data->pmic_had_boost = true;
			wait_ms = 0;
			break;
		}
		if (ret != 0) {
			wait_ms = hvdcp->cmd_retry_wait_ms;
			break;
		}
		break;
	case HVDCP_SETTLE_MONITOR:
		ret = read_voltage(hvdcp, &voltage);
		if (ret != 0) {
			wait_ms = VOLTAGE_CHECK_RETRY_MS;
			break;
		}
		if (voltage > hvdcp->voltage_ceiling) {
			dev_dbg(hvdcp->dev, "%u uV over %u uV ceiling!\n",
				voltage, hvdcp->voltage_ceiling);
			new_state = HVDCP_NEGOTIATION;
			sesh_data->slow_waits = true;
			wait_ms = hvdcp->renegotiation_wait_ms;
			break;
		}

		delta = (hvdcp->voltage_ceiling - voltage);
		if (delta > (HVDCP_PULSE_DELTA_UV + 20000)) {
			dev_dbg(hvdcp->dev, "Delta %d uV larger than pulse!\n",
				delta);
			new_state = HVDCP_NEGOTIATION;
			sesh_data->slow_waits = true;
			wait_ms = hvdcp->renegotiation_wait_ms;
			break;
		}

		if (state_data->elapsed_ms < hvdcp->settle_monitor_ms) {
			wait_ms = VOLTAGE_CHECK_RETRY_MS;
			break;
		}

		set_voltage_max(voltage);
		/* Can go straight to waiting for USBIN to resume */
		new_state = HVDCP_USBIN_RESUME_WAIT;
		wait_ms = 0;
		break;
	case HVDCP_ABANDON_RECOVERY_USBIN_PREP:
		/* Wait for boost to stop. Note, we won't reset
		 * hvdcp->had_boost. Instead we'll do that in in
		 * HVDCP_USBIN_ACTIVE_WAIT, since it checks that there is no
		 * boost *and* that USBIN was successfully resumed.
		 */
		if (is_boost_en) {
			wait_ms = USBIN_PREP_WAIT_MS;
			break;
		}

		ret = read_voltage(hvdcp, &voltage);
		if ((ret != 0) || (voltage > hvdcp->voltage_ceiling)) {
			/*
			 * Force APSD to rerun to hopefully reset charger to
			 * base voltage.
			 */
			dev_warn(hvdcp->dev,
				"Charger voltage reset via APSD\n");
			smblite_lib_rerun_apsd(hvdcp->smblite_shim->chg);
			if (sesh_data->abandon)
				wait_ms = USBIN_PREP_WAIT_MS;
			else {
				/* Wait for trigger via hvdcp_req_notify() */
				wait_ms = WAIT_DISABLED;
			}
		} else {
			wait_ms = 0;
		}
		new_state = HVDCP_USBIN_RESUME_WAIT;
		break;
	case HVDCP_USBIN_RESUME_WAIT:
		if (sesh_data->abandon || !sesh_data->boost_recovery)
			ret = set_usb_icl(hvdcp, ICL_UNRESTRICTED);
		else
			ret = set_usb_icl(hvdcp, ICL_LOWEST_NONZERO);

		if (ret != 0) {
			wait_ms = SUSPEND_WAIT_MS;
			break;
		}
		if (sesh_data->abandon) {
			set_psy_fake_online(hvdcp, false);
			new_state = HVDCP_ABANDONED;
		} else if (sesh_data->boost_recovery) {
			/* Keep faking psy online to avoid charge UX toggle */

			new_state = HVDCP_USBIN_ACTIVE_WAIT;
			/*
			 * The only state transition here that requires more
			 * controller work
			 */
			wait_ms = 0;
		} else {
			set_psy_fake_online(hvdcp, false);
			new_state = HVDCP_DONE;
		}
		break;
	case HVDCP_ABANDON_VIA_5V:
		ret = read_voltage(hvdcp, &voltage);
		if (ret != 0) {
			wait_ms = VOLTAGE_CHECK_RETRY_MS;
			break;
		}

		delta = (voltage - sesh_data->start_voltage);
		if ((voltage <= sesh_data->start_voltage)
			|| delta < (HVDCP_PULSE_DELTA_UV / 2)) {
			new_state = HVDCP_USBIN_RESUME_WAIT;
			wait_ms = 0;
			break;
		}

		ret = run_cmd(hvdcp, CMD_FORCE_5V, &wait_ms);
		if (ret == -EAGAIN) {
			sesh_data->pmic_had_boost = true;
			wait_ms = 0;
			break;
		}
		if (ret != 0) {
			wait_ms = hvdcp->cmd_retry_wait_ms;
		}
		break;
	default:
		break;
	}

out:
	if (sesh_data->pmic_had_boost
		&& state_needs_recovery_from_boost(new_state)) {
		sesh_data->boost_recovery = true;
		new_state = HVDCP_ABANDON_RECOVERY_USBIN_PREP;
		wait_ms = 0;
	}

	update_state_locked(hvdcp, new_state);
	mutex_unlock(&hvdcp->lock);

	if (wait_ms != WAIT_DISABLED) {
		dev_dbg(hvdcp->dev, "Next controller run in %d ms\n",
			wait_ms);
		state_data->wait_ms = wait_ms;
		alarm_try_to_cancel(&hvdcp->controller_alarm);

		if (wait_ms == 0)
			goto rerun_controller;

		alarm_start_relative(&hvdcp->controller_alarm,
				ms_to_ktime(wait_ms));
	}
}

static enum alarmtimer_restart
controller_alarm_expired(struct alarm *alarm, ktime_t time)
{
	struct hvdcp *hvdcp = container_of(alarm, struct hvdcp,
					controller_alarm);

	dev_dbg(hvdcp->dev, "entry\n");

	queue_work(hvdcp->controller_q, &hvdcp->controller_work);
	return ALARMTIMER_NORESTART;
}

static int hvdcp_req_notify(struct notifier_block *nb,
			unsigned long unused_a, void *unused_b)
{
	struct hvdcp *hvdcp = container_of(nb, struct hvdcp, hvdcp_req_nb);

	dev_dbg(hvdcp->dev, "entry\n");

	mutex_lock(&hvdcp->lock);
	if (hvdcp->state == HVDCP_OFF) {
		memset(hvdcp->sesh_data.total_elapsed_ms, 0,
			sizeof(hvdcp->sesh_data.total_elapsed_ms));
		reset_state_locked(hvdcp);
	}
	if ((hvdcp->state == HVDCP_DONE)
		|| ((hvdcp->state) == HVDCP_ABANDONED)) {
		goto out;
	}

	queue_work(hvdcp->controller_q, &hvdcp->controller_work);
out:
	mutex_unlock(&hvdcp->lock);

	return NOTIFY_OK;
}

static int plugin_notify(struct notifier_block *nb, unsigned long plugged,
			void *unused)
{
	struct hvdcp *hvdcp = container_of(nb, struct hvdcp, plugin_nb);

	dev_dbg(hvdcp->dev, "%s: %u\n", __func__, plugged);

	/*
	 * Only care about when device is unplugged. For plugin, we rely on
	 * the hvdcp req notifier to start hvdcp negotiation.
	 */
	if (((enum smblite_shim_plug_sts)plugged) == SMBLITE_SHIM_UNPLUGGED) {
		mutex_lock(&hvdcp->lock);
		reset_state_locked(hvdcp);
		mutex_unlock(&hvdcp->lock);
	}

	return NOTIFY_OK;
}

static int boost_notify(struct notifier_block *nb, unsigned long boost_en,
			void *unused)
{
	struct hvdcp *hvdcp = container_of(nb, struct hvdcp, boost_nb);
	dev_dbg(hvdcp->dev, "boost: %u\n", boost_en);

	mutex_lock(&hvdcp->lock);

	if ((hvdcp->state == HVDCP_OFF)
		|| (hvdcp->state == HVDCP_DONE)
		|| (hvdcp->state == HVDCP_ABANDONED))
		goto out;

	if (((enum smblite_shim_boost_sts)boost_en) == SMBLITE_SHIM_BOOST_EN) {
		hvdcp->sesh_data.pmic_had_boost = true;
	}

	queue_work(hvdcp->controller_q, &hvdcp->controller_work);
out:
	mutex_unlock(&hvdcp->lock);

	return NOTIFY_OK;
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	enum hvdcp_state state;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct hvdcp *hvdcp = platform_get_drvdata(pdev);

	mutex_lock(&hvdcp->lock);
	state = hvdcp->state;
	mutex_unlock(&hvdcp->lock);

	return scnprintf(buf, PAGE_SIZE, "%s\n", state_str[state]);
}
static const DEVICE_ATTR_RO(state);

static ssize_t enabled_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	bool enabled;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct hvdcp *hvdcp = platform_get_drvdata(pdev);

	mutex_lock(&hvdcp->lock);
	enabled = hvdcp->enabled;
	mutex_unlock(&hvdcp->lock);

	return scnprintf(buf, PAGE_SIZE, "%u\n", (int)enabled);
}

static ssize_t enabled_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	bool enable;
	int ret;
	struct platform_device *pdev =
	container_of(dev, struct platform_device, dev);
	struct hvdcp *hvdcp = platform_get_drvdata(pdev);

	ret = kstrtobool(buf, &enable);
	if (ret != 0)
		return ret;

	mutex_lock(&hvdcp->lock);
	hvdcp->enabled = enable;
	if (hvdcp->state == HVDCP_READY)
		queue_work(hvdcp->controller_q, &hvdcp->controller_work);
	mutex_unlock(&hvdcp->lock);

	return count;
}
static const DEVICE_ATTR_RW(enabled);

static int setup_sysfs(struct hvdcp *hvdcp)
{
	struct device *dev = hvdcp->dev;
	int ret;

	ret = device_create_file(hvdcp->dev, &dev_attr_enabled);
	if (ret < 0) {
		dev_err(dev, "Failed to create %s: %d\n",
			dev_attr_enabled.attr.name, ret);
		return ret;
	}

	ret = device_create_file(hvdcp->dev, &dev_attr_state);
	if (ret < 0) {
		dev_err(dev, "Failed to create %s: %d\n",
			dev_attr_state.attr.name, ret);

		/* Nonfatal */
		ret = 0;
	}

	return ret;
}

static void teardown_sysfs(struct hvdcp *hvdcp)
{
	device_remove_file(hvdcp->dev, &dev_attr_state);
}

static struct power_supply *dt_find_psy(struct device *dev, const char *prop)
{
	int ret;
	struct device_node *node = dev->of_node;
	const char *name;

	ret = of_property_read_string(node, prop, &name);
	if (ret != 0)
		return NULL;

	return power_supply_get_by_name(name);
}

static int hvdcp_probe(struct platform_device *pdev)
{
	struct hvdcp *hvdcp;
	struct gvotable_election *icl_votable;
	struct gvotable_election *fake_psy_online_votable;
	struct power_supply *psy;
	int ret;

	icl_votable = gvotable_election_get_handle("USB_ICL");
	if (!icl_votable)
		return -EPROBE_DEFER;

	fake_psy_online_votable =
		gvotable_election_get_handle("SHIM_FAKE_OLN");

	if (!fake_psy_online_votable) {
		return -EPROBE_DEFER;
	}

	/*
	 * Check that the power supply registered by qpnp-smblite.c exists.
	 * If it does, then it's likely that we have an smblite_shim.
	 */
	psy = dt_find_psy(&pdev->dev, "google,qcom-psy");
	if (!psy) {
		dev_err(&pdev->dev, "Could not find qcom's psy\n");
		return -EPROBE_DEFER;
	}
	power_supply_put(psy);

	psy = dt_find_psy(&pdev->dev, "google,shim-psy");
	if (!psy) {
		dev_err(&pdev->dev, "Could not find shim psy\n");
		return -EPROBE_DEFER;
	}

	hvdcp = devm_kzalloc(&pdev->dev, sizeof(*hvdcp), GFP_KERNEL);
	if (!hvdcp)
		return -ENOMEM;

	hvdcp->controller_q = alloc_ordered_workqueue("goog-hvdcp-ctrl", 0);
	if (!hvdcp->controller_q) {
		dev_err(&pdev->dev, "Could not alloc wq!\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, hvdcp);
	hvdcp->dev = &pdev->dev;

	hvdcp->usb_icl_votable = icl_votable;
	hvdcp->fake_psy_online_votable = fake_psy_online_votable;
	hvdcp->shim_psy = psy;
	hvdcp->smblite_shim = power_supply_get_drvdata(hvdcp->shim_psy);

	mutex_init(&hvdcp->lock);

	INIT_WORK(&hvdcp->controller_work, controller_work);
	alarm_init(&hvdcp->controller_alarm, ALARM_BOOTTIME,
		controller_alarm_expired);

	hvdcp->cmd_waits[CMD_IDLE].typ = 100;
	hvdcp->cmd_waits[CMD_IDLE].slow = 2500;

	hvdcp->cmd_waits[CMD_FORCE_5V].typ = 150;
	hvdcp->cmd_waits[CMD_FORCE_5V].slow = 2500;

	hvdcp->cmd_waits[CMD_INCREMENT].typ = 150;
	hvdcp->cmd_waits[CMD_INCREMENT].slow = 2500;

	hvdcp->cmd_waits[CMD_DECREMENT].typ = 150;
	hvdcp->cmd_waits[CMD_DECREMENT].slow = 2500;

	hvdcp->renegotiation_wait_ms = 2500;
	hvdcp->cmd_retry_wait_ms = 2500;
	hvdcp->settle_monitor_ms = 6000;

	hvdcp->voltage_ceiling = 5500000;
	hvdcp->abandon_via_5v_timeout_ms = 60000;
	hvdcp->forced_abandon_timeout_ms = 66000;

	reset_state_locked(hvdcp);

	hvdcp->plugin_nb.notifier_call = plugin_notify;
	ret = smblite_shim_plugin_register_notifier(hvdcp->smblite_shim,
						&hvdcp->plugin_nb);
	if (ret != 0) {
		dev_err(hvdcp->dev, "Couldn't register plugin notifier (%d)\n",
			ret);
		goto fail;
	}

	hvdcp->hvdcp_req_nb.notifier_call = hvdcp_req_notify;
	ret = smblite_shim_hvdcp_req_register_notifier(hvdcp->smblite_shim,
						&hvdcp->hvdcp_req_nb);
	if (ret != 0) {
		dev_err(hvdcp->dev, "Couldn't register hvdcp notifier (%d)\n",
			ret);
		goto fail_hvdcp_nb;
	}

	hvdcp->boost_nb.notifier_call = boost_notify;
	ret = smblite_shim_boost_register_notifier(hvdcp->smblite_shim,
						&hvdcp->boost_nb);
	if (ret != 0) {
		dev_err(hvdcp->dev, "Couldn't register boost notifier (%d)\n",
			ret);
		goto fail_boost_nb;
	}

	setup_sysfs(hvdcp);
	return 0;

fail_boost_nb:
	smblite_shim_hvdcp_req_unregister_notifier(hvdcp->smblite_shim,
						&hvdcp->hvdcp_req_nb);
fail_hvdcp_nb:
	smblite_shim_plugin_unregister_notifier(hvdcp->smblite_shim,
						&hvdcp->plugin_nb);
fail:
	destroy_workqueue(hvdcp->controller_q);
	return ret;
}

static int hvdcp_remove(struct platform_device *pdev)
{
	struct hvdcp *hvdcp = platform_get_drvdata(pdev);
	alarm_try_to_cancel(&hvdcp->controller_alarm);
	teardown_sysfs(hvdcp);
	smblite_shim_hvdcp_req_unregister_notifier(hvdcp->smblite_shim,
						&hvdcp->hvdcp_req_nb);
	smblite_shim_plugin_unregister_notifier(hvdcp->smblite_shim,
						&hvdcp->plugin_nb);
	smblite_shim_boost_unregister_notifier(hvdcp->smblite_shim,
					&hvdcp->boost_nb);
	alarm_try_to_cancel(&hvdcp->controller_alarm);
	power_supply_put(hvdcp->shim_psy);
	destroy_workqueue(hvdcp->controller_q);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static int hvdcp_pm_suspend(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct hvdcp *hvdcp = platform_get_drvdata(pdev);

	dev_dbg(dev, "entry\n");
	flush_workqueue(hvdcp->controller_q);
	return 0;
}

static int hvdcp_pm_resume(struct device *dev)
{
	dev_dbg(dev, "entry\n");
	return 0;
}

static const struct dev_pm_ops hvdcp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hvdcp_pm_suspend, hvdcp_pm_resume)
};

static const struct of_device_id match_table[] = {
	{ .compatible = "google,smblite-hvdcp" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, match_table);

static struct platform_driver hvdcp_driver = {
	.driver	= {
		.name = "google-smblite-hvdcp",
		.of_match_table	= of_match_ptr(match_table),
		.pm = &hvdcp_pm_ops,
	},
	.probe		= hvdcp_probe,
	.remove		= hvdcp_remove,
};
module_platform_driver(hvdcp_driver);

MODULE_DESCRIPTION("QC 3.0 (HVDCP) controller");
MODULE_LICENSE("GPL v2");
