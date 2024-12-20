/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * SCMI Vendor Protocols header
 *
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _SCMI_VENDOR_H
#define _SCMI_VENDOR_H

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/types.h>


#define SCMI_PROTOCOL_MEMLAT    0x80
#define MAX_EV_CNTRS		4 /* Maximum number of grp or common events */
#define MAX_NAME_LEN		20

struct scmi_protocol_handle;
/**
 * struct scmi_memlat_vendor_ops - represents the various operations provided
 *      by SCMI HW Memlat Protocol
 *
 * @mem_grp: set the memgrp
 * @set_mon: set the supported monitors
 * @set_ev_map: set the list of events supported
 * @ipm_ceil: sets the ipm_ceil needed for hw memlat governor
 * @stall_floor: sets the stall_floor needed for hw memlat governor
 * @l2wb_pct: sets the stall_floor needed for hw memlat governor
 * @l2wb_filter: sets the l2wb_filter needed for hw memlat governor
 * @freq_scale_pct: sets the scaling factor for ipm differential voting
 * @freq_scale_ceil_mhz: upper limit upto which ipm scaling is allowed
 * @freq_scale_floor_mhz: lower limit upto which ipm scaling is allowed
 * @sample_ms: sets the sample_ms at this interval governor will poll
 * @freq_map: sets the freq_map of the monitor
 * @min_freq: sets the min_freq of monitor
 * @max_freq: sets the max_freq of monitor
 * @start_mon: starts monitor in cpucp
 * @stop_mon: stops monitor in cpucp
 * @log_level: configure the supported log_level in cpucp
 * @get_data: added for debug purpose gets the data structure information
 */
struct scmi_memlat_vendor_ops {
	int (*set_mem_grp)(const struct scmi_protocol_handle *ph,
			   u32 cpus_mpidr, u32 hw_type);
	int (*set_mon)(const struct scmi_protocol_handle *ph, u32 cpus_mpidr,
		       u32 hw_type, u32 mon_type, u32 index, const char *mon_name);
	int (*set_grp_ev_map)(const struct scmi_protocol_handle *ph, u32 hw_type,
			  void *buf, u32 num_evs);
	int (*adaptive_low_freq)(const struct scmi_protocol_handle *ph,
					 u32 hw_type, u32 index, u32 val);
	int (*adaptive_high_freq)(const struct scmi_protocol_handle *ph,
					  u32 hw_type, u32 index, u32 val);
	int (*get_adaptive_cur_freq)(const struct scmi_protocol_handle *ph, u32 hw_type,
				     u32 mon_idx, void *buf);
	int (*set_common_ev_map)(const struct scmi_protocol_handle *ph, void *buf,
				 u32 num_evs);
	int (*ipm_ceil)(const struct scmi_protocol_handle *ph,
			u32 hw_type, u32 index, u32 val);
	int (*fe_stall_floor)(const struct scmi_protocol_handle *ph,
			      u32 hw_type, u32 index, u32 val);
	int (*be_stall_floor)(const struct scmi_protocol_handle *ph,
			      u32 hw_type, u32 index, u32 val);
	int (*wb_pct_thres)(const struct scmi_protocol_handle *ph,
			    u32 hw_type, u32 index, u32 val);
	int (*wb_filter_ipm)(const struct scmi_protocol_handle *ph,
			     u32 hw_type, u32 index, u32 val);
	int (*freq_scale_pct)(const struct scmi_protocol_handle *ph,
			     u32 hw_type, u32 index, u32 val);
	int (*freq_scale_ceil_mhz)(const struct scmi_protocol_handle *ph,
			     u32 hw_type, u32 index, u32 val);
	int (*freq_scale_floor_mhz)(const struct scmi_protocol_handle *ph,
			     u32 hw_type, u32 index, u32 val);
	int (*sample_ms)(const struct scmi_protocol_handle *ph, u32 val);
	int (*freq_map)(const struct scmi_protocol_handle *ph,
			u32 hw_type, u32 index, u32 nr_rows, void *buf);
	int (*min_freq)(const struct scmi_protocol_handle *ph,
			u32 hw_type, u32 index, u32 val);
	int (*max_freq)(const struct scmi_protocol_handle *ph,
			u32 hw_type, u32 index, u32 val);
	int (*get_cur_freq)(const struct scmi_protocol_handle *ph, u32 hw_type,
			    u32 mon_idx, void *buf);
	int (*start_timer)(const struct scmi_protocol_handle *ph);
	int (*stop_timer)(const struct scmi_protocol_handle *ph);
	int (*set_log_level)(const struct scmi_protocol_handle *ph, u32 val);
	int (*flush_cpucp_log)(const struct scmi_protocol_handle *ph);
	int (*get_timestamp)(const struct scmi_protocol_handle *ph, void *buf);
};
#endif
