/**
 * The module represents a ASOC codec driver responsible for turning Microphone
 * on the MCU on or off.
 *
 * Copyright 2022 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/completion.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>


// Valid sampling rates for the MCU DMIC
#define MCU_PCM_RATE_8000	(8000)
#define MCU_PCM_RATE_16000	(16000)
#define MCU_PCM_RATE_48000	(48000)
#define MIC_MCU_RATES		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
				SNDRV_PCM_RATE_48000)

// Valid formats for the MCU DMIC
#define MIC_MCU_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE)

// Must match mixer paths default config + MCU side FW
#define MIC_DEFAULT_SAMPLE_RATE	(MCU_PCM_RATE_16000)
// Must match MCU side FW
#define MIC_MIN_GAIN		(0)
#define MIC_MAX_GAIN		(15)
#define MIC_DEFAULT_GAIN	(3)

// Values validated by Audio tuning team
// All gains are set to 0 and each individual use case is
// added proper gain directly in its ACDB audio topology
#define GAIN_FOR_8000	(0)
#define GAIN_FOR_16000	(0)
#define GAIN_FOR_48000	(0)

#define NANOHUB_AUDIO_CHANNEL_ID (16)

// Values here should match MCU FW values defined in audio_services.cc
#define DMIC_MCU_MESSAGE_VERSION		(1)
#define DMIC_MCU_MESSAGE_MIC_ON			(0x01)
#define DMIC_MCU_MESSAGE_SAMPLE_RATE_KHZ	(0x02)
// #define DMIC_MCU_MESSAGE_CHANNEL		(0x03) // <= Legacy. Not used anymore.
#define DMIC_MCU_MESSAGE_GAIN			(0x04)
#define DMIC_MCU_MESSAGE_MIC_READY		(0x05)
#define DMIC_MCU_MESSAGE_HOTWORD_STATUS		(0x06)
#define DMIC_MCU_MESSAGE_RTT_STATUS		(0x07)

/* Only for external static functions */
static struct platform_device *priv_platform_device;

extern ssize_t nanohub_send_message(int channel_id, const char *buffer,
				    size_t length);
extern void nanohub_register_listener(
	int channel_id,
	void (*on_message_received)(const char *buffer, size_t length));
extern void nanohub_unregister_listener(int channel_id);

// Structure used to store mcu dmic state:
// [mic_on] 0: OFF - No condition
//	    1: ON - No condition
//	    10: OFF - Conditional
//	    11: ON - Conditional
//	    254: Sawtooth recording - Used for some SW testing
//	    255: Fake recording - Used for some factory testing
// [sample_rate_hz]: Units of hertz
// [gain]: 0->15 is the gainshift the MCU DMIC uses
// [gain_user_requested]: Gain requested from tinymix control
// [hw_enabled] 1: Hotword enabled, 0: Hotword disabled
// [rtt_enabled] 1: Raise-to-Talk enabled, 0: Raise-to-Talk disabled
// [screen_off_dsp_hw_on]: 1: Screen if off and DSP HW is the only mic user.
//			   0: One if the condition above is not true.
// [pending_rec]: Pending rec. Same values as mic_on
// [iolock]: Utilized to guarantee integrity of the data
struct mcu_mic_codec_data {
	int mic_on;
	unsigned int sample_rate_hz;
	int gain;
	bool gain_user_requested;
	bool hw_enabled;
	bool rtt_enabled;
	bool screen_off_dsp_hw_on;
	int pending_rec;
	struct mutex iolock;
};

enum dmic_mcu_on {
	DMIC_MCU_ON_OFF,
	DMIC_MCU_ON_ON,
	DMIC_MCU_ON_FORCE_OFF,
	DMIC_MCU_ON_FORCE_ON,

	DMIC_MCU_ON_OFF_CONDITIONAL = 10,
	DMIC_MCU_ON_ON_CONDITIONAL = 11,
	DMIC_MCU_ON_AND_TOGGLE_ADSP,

	DMIC_MCU_ON_SAWTOOTH_REC = 254,
	DMIC_MCU_ON_FAKE_REC = 255,
};

#define ERROR_NACK	-1 // Need to match nanohub/comms.h
#define MAX_NACK_RETRY	5
static int dmic_mcu_send_message(struct device *dev,
				 char mcu_message_id, int data)
{
	char buffer[3];
	ssize_t bytes;
	int retry_nack = 0;

	buffer[0] = DMIC_MCU_MESSAGE_VERSION;	// version
	buffer[1] = mcu_message_id;		// message identifier
	buffer[2] = (char)data;

	do {
		if (retry_nack)
			dev_warn(dev, "ERROR_NACK, retry: %d\n", retry_nack);

		bytes = nanohub_send_message(NANOHUB_AUDIO_CHANNEL_ID, buffer,
					sizeof(buffer));
	} while ((bytes == ERROR_NACK) && (retry_nack++ < MAX_NACK_RETRY));

	if (bytes != sizeof(buffer)) {
		dev_err(dev, "Bytes sent expected = %zd, actual = %zd\n",
			sizeof(buffer), bytes);
		return -EIO;
	}

	return 0;
}

// The callback that is called when ASOC needs to fetch the value of the
// property 'DMIC_MCU Input Gain'.
static int dmic_mcu_input_gain_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);

	mutex_lock(&codec_data->iolock);
	ucontrol->value.integer.value[0] = codec_data->gain;
	dev_dbg(component->dev, "%s: mcu_input_gain: %d\n",
		__func__, codec_data->gain);
	mutex_unlock(&codec_data->iolock);

	return 0;
}

// The callback that is called when ASOC needs to set the value of the
// property 'DMIC_MCU Input Gain'.
static int dmic_mcu_input_gain_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];
	int ret;

	dev_dbg(component->dev, "%s\n", __func__);

	if (value < MIC_MIN_GAIN || value > MIC_MAX_GAIN) {
		dev_err(component->dev,
			"Invalid value: %d, it must be between %d and %d\n",
			value, MIC_MIN_GAIN, MIC_MAX_GAIN);
		return -EINVAL;
	}

	ret = dmic_mcu_send_message(component->dev, DMIC_MCU_MESSAGE_GAIN, value);
	if (ret != 0)
		return ret;

	mutex_lock(&codec_data->iolock);
	codec_data->gain_user_requested = true;
	codec_data->gain = value;
	dev_info(component->dev, "%s: new mcu_input_gain: %d\n",
		__func__, codec_data->gain);
	mutex_unlock(&codec_data->iolock);

	return 0;
}

// The callback that is called when ASOC needs to fetch the value of the
// property 'DMIC_MCU Sample Rate'.
static int dmic_mcu_sample_rate_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);

	mutex_lock(&codec_data->iolock);
	ucontrol->value.integer.value[0] = codec_data->sample_rate_hz;
	dev_info(component->dev, "%s: mcu_mic_sample_rate: %u\n",
		__func__, codec_data->sample_rate_hz);
	mutex_unlock(&codec_data->iolock);

	return 0;
}

// The callback that is called when ASOC needs to set the value of the
// property 'DMIC_MCU Sample Rate'.
static int dmic_mcu_sample_rate_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);
	unsigned int value = ucontrol->value.integer.value[0];
	int ret;

	dev_dbg(component->dev, "%s\n", __func__);

	if (value != MCU_PCM_RATE_8000
		&& value != MCU_PCM_RATE_16000
		&& value != MCU_PCM_RATE_48000) {
		if (value == 0) {
			// Gets here when starting a recording that needs to wait
			// for the sampling rate to arrive in mcu_mic_hw_params().
			// In this case, we don't want to do anything and wait for
			// the update to happen in mcu_mic_hw_params().
			mutex_lock(&codec_data->iolock);
			codec_data->sample_rate_hz = value;
			mutex_unlock(&codec_data->iolock);
			dev_info(component->dev, "%s: Special mixer_paths request\n",
				__func__);

			return 0;
		} else {
			dev_err(component->dev,
				"Invalid Sample Rate: %u. (Valid rates: %d, %d, %d Hz)\n",
				value, MCU_PCM_RATE_8000, MCU_PCM_RATE_16000,
				MCU_PCM_RATE_48000);
			return -EINVAL;
		}
	}

	ret = dmic_mcu_send_message(component->dev, DMIC_MCU_MESSAGE_SAMPLE_RATE_KHZ,
		(int)(value / 1000));
	if (ret != 0)
		return ret;

	mutex_lock(&codec_data->iolock);
	codec_data->sample_rate_hz = value;
	dev_info(component->dev, "%s: new mcu_mic_sample_rate: %u\n",
		__func__, codec_data->sample_rate_hz);
	mutex_unlock(&codec_data->iolock);

	return 0;
}

// The callback that is called when ASOC needs to fetch the value of the
// property 'DMIC_MCU On'.
static int dmic_mcu_on_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);

	mutex_lock(&codec_data->iolock);
	ucontrol->value.integer.value[0] = codec_data->mic_on;
	dev_info(component->dev, "%s: mcu_mic_on: %d\n",
		__func__, codec_data->mic_on);
	mutex_unlock(&codec_data->iolock);

	return 0;
}

// The callback that is called when ASOC needs to set the value of the
// property 'DMIC_MCU On'.
static int dmic_mcu_on_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];
	int ret = 0;
	int conditionalized_value;
	bool value_forced = false;

	dev_dbg(component->dev, "%s\n", __func__);

	mutex_lock(&codec_data->iolock);

	// Check if this is a DSP hotword related request
	if (value == DMIC_MCU_ON_FORCE_OFF) {
		value = DMIC_MCU_ON_OFF;
		value_forced = true;
	} else if (value == DMIC_MCU_ON_FORCE_ON) {
		value = DMIC_MCU_ON_ON;
		value_forced = true;
	}

	if (value != DMIC_MCU_ON_OFF &&
		value != DMIC_MCU_ON_ON &&
		value != DMIC_MCU_ON_FAKE_REC &&
		value != DMIC_MCU_ON_SAWTOOTH_REC) {
		dev_err(component->dev,
			"Invalid value: %d, it must be %d, %d, %d, or %d.\n", value,
			DMIC_MCU_ON_OFF, DMIC_MCU_ON_ON, DMIC_MCU_ON_SAWTOOTH_REC,
			DMIC_MCU_ON_FAKE_REC);
		ret = -EINVAL;
		goto end;
	} else if ((value == DMIC_MCU_ON_ON || value == DMIC_MCU_ON_FAKE_REC ||
		value == DMIC_MCU_ON_SAWTOOTH_REC) &&
		codec_data->sample_rate_hz == 0) {
		// Gets here when starting a recording that needs to wait
		// for the sampling rate to arrive in mcu_mic_hw_params().
		// In this case, we don't want to do anything and wait for
		// the update to happen in mcu_mic_hw_params().
		codec_data->pending_rec = value;
		dev_info(component->dev, "%s: Special mixer_paths request: %d\n",
			__func__, value);
		goto end;
	} else if (value == DMIC_MCU_ON_OFF) {
		codec_data->gain_user_requested = false;
		codec_data->pending_rec = DMIC_MCU_ON_OFF;
	}

	// Update to conditional ON/OFF in case of screen off AND DSP HW only
	if (codec_data->screen_off_dsp_hw_on && (value == DMIC_MCU_ON_ON))
		conditionalized_value = DMIC_MCU_ON_ON_CONDITIONAL;
	else if (codec_data->screen_off_dsp_hw_on && (value == DMIC_MCU_ON_OFF))
		conditionalized_value = DMIC_MCU_ON_OFF_CONDITIONAL;
	else if (value_forced && (value == DMIC_MCU_ON_ON))
		conditionalized_value = DMIC_MCU_ON_AND_TOGGLE_ADSP;
	else
		conditionalized_value = value;

	ret = dmic_mcu_send_message(component->dev, DMIC_MCU_MESSAGE_MIC_ON, conditionalized_value);
	if (ret != 0) {
		dev_err(component->dev,
			"Error sending DMIC_MCU_MESSAGE_MIC_ON: %d", ret);
		ret = -EIO;
		// Set codec_data->mic_on anyways if this is a DSP hotword request
		if (value_forced)
			codec_data->mic_on = value;
		goto end;
	}

	codec_data->mic_on = value;

	dev_info(component->dev, "%s: new mcu_mic_on: %d\n",
		__func__, codec_data->mic_on);

end:
	mutex_unlock(&codec_data->iolock);

	return ret;
}

static int dmic_mcu_hw_enabled_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);

	mutex_lock(&codec_data->iolock);
	ucontrol->value.integer.value[0] = codec_data->hw_enabled;
	dev_info(component->dev, "%s: mcu_mic_hw_enabled: %d\n",
		__func__, codec_data->hw_enabled);
	mutex_unlock(&codec_data->iolock);

	return 0;
}

static int dmic_mcu_hw_enabled_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];
	int ret = 0;

	dev_dbg(component->dev, "%s\n", __func__);

	// Save HW Enable Status regardless if actual send to MCU succeed or
	// not.  Reasoning is this saved flag is only used to notify MCU if MCU
	// crashes, and we want to use the framework's intended state in restoring
	// MCU after the crash.
	mutex_lock(&codec_data->iolock);
	codec_data->hw_enabled = value;
	dev_info(component->dev, "%s: mcu_mic_hw_enabled: %d\n",
		__func__, codec_data->hw_enabled);
	mutex_unlock(&codec_data->iolock);

	ret = dmic_mcu_send_message(component->dev, DMIC_MCU_MESSAGE_HOTWORD_STATUS, value);
	if (ret != 0) {
		dev_err(component->dev,
			"Error sending DMIC_MCU_MESSAGE_HOTWORD_STATUS: %d", ret);
		ret = -EIO;
	}
	return ret;
}

static int dmic_mcu_rtt_enabled_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);

	mutex_lock(&codec_data->iolock);
	ucontrol->value.integer.value[0] = codec_data->rtt_enabled;
	dev_info(component->dev, "%s: mcu_mic_rtt_enabled: %d\n",
		__func__, codec_data->rtt_enabled);
	mutex_unlock(&codec_data->iolock);

	return 0;
}

static int dmic_mcu_rtt_enabled_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];
	int ret = 0;

	dev_dbg(component->dev, "%s\n", __func__);

	// Save RTT Enable Status regardless if actual send to MCU succeed or
	// not.  Reasoning is this saved flag is only used to notify MCU if MCU
	// crashes, and we want to use the framework's intended state in restoring
	// MCU after the crash.
	mutex_lock(&codec_data->iolock);
	codec_data->rtt_enabled = value;
	dev_info(component->dev, "%s: mcu_mic_rtt_enabled: %d\n",
		__func__, codec_data->rtt_enabled);
	mutex_unlock(&codec_data->iolock);

	ret = dmic_mcu_send_message(component->dev, DMIC_MCU_MESSAGE_RTT_STATUS, value);
	if (ret != 0) {
		dev_err(component->dev,
			"Error sending DMIC_MCU_MESSAGE_RTT_STATUS: %d", ret);
		ret = -EIO;
	}
	return ret;
}

static int dmic_mcu_screen_off_dsp_hw_on_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);

	mutex_lock(&codec_data->iolock);
	ucontrol->value.integer.value[0] = codec_data->screen_off_dsp_hw_on;
	dev_info(component->dev, "%s: mcu_mic_screen_off_dsp_hw_on: %d\n",
		__func__, codec_data->screen_off_dsp_hw_on);
	mutex_unlock(&codec_data->iolock);

	return 0;
}

static int dmic_mcu_screen_off_dsp_hw_on_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];

	dev_dbg(component->dev, "%s\n", __func__);

	mutex_lock(&codec_data->iolock);
	codec_data->screen_off_dsp_hw_on = value;
	dev_info(component->dev, "%s: mcu_mic_screen_off_dsp_hw_on: %d\n",
		__func__, codec_data->screen_off_dsp_hw_on);
	mutex_unlock(&codec_data->iolock);

	return 0;
}

static const struct snd_kcontrol_new mcu_snd_controls[] = {
	// Defines a property DMIC_MCU On.
	// It can be switched in console for example using tinyalsa:
	// tinymix 'DMIC_MCU On' 1
	SOC_SINGLE_EXT("DMIC_MCU On", SND_SOC_NOPM, 0, DMIC_MCU_ON_FAKE_REC, 0,
		dmic_mcu_on_get, dmic_mcu_on_put),
	SOC_SINGLE_EXT("DMIC_MCU Sample Rate", SND_SOC_NOPM, 0, MCU_PCM_RATE_48000, 0,
		dmic_mcu_sample_rate_get, dmic_mcu_sample_rate_put),
	SOC_SINGLE_EXT("DMIC_MCU Input Gain", SND_SOC_NOPM, 0, MIC_MAX_GAIN, 0,
		dmic_mcu_input_gain_get, dmic_mcu_input_gain_put),
	SOC_SINGLE_EXT("DMIC_MCU HW Enabled", SND_SOC_NOPM, 0, 1, 0,
		dmic_mcu_hw_enabled_get, dmic_mcu_hw_enabled_put),
	SOC_SINGLE_EXT("DMIC_MCU Screen off DSP HW on", SND_SOC_NOPM, 0, 1, 0,
		dmic_mcu_screen_off_dsp_hw_on_get, dmic_mcu_screen_off_dsp_hw_on_put),
	SOC_SINGLE_EXT("DMIC_MCU RTT Enabled", SND_SOC_NOPM, 0, 1, 0,
		dmic_mcu_rtt_enabled_get, dmic_mcu_rtt_enabled_put),
};

static int mcu_codec_probe(struct snd_soc_component *component)
{
	dev_dbg(component->dev, "%s\n", __func__);

	return 0;
}

static void mcu_codec_remove(struct snd_soc_component *component)
{
	dev_dbg(component->dev, "%s\n", __func__);
}

static struct snd_soc_component_driver mcu_mic_soc_component_driver = {
	.probe = mcu_codec_probe,
	.remove = mcu_codec_remove,
	.controls = mcu_snd_controls,
	.num_controls = ARRAY_SIZE(mcu_snd_controls),
};

static int mcu_mic_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);
	unsigned int sample_rate = params_rate(params);
	int ret = 0;
	int conditionalized_value;

	mutex_lock(&codec_data->iolock);

	if (!codec_data->pending_rec && (codec_data->mic_on == DMIC_MCU_ON_ON)) {
		// In case the mic is already running, we allow new audio input
		// requests without having to force the mic to first be turned off.
		dev_info(component->dev,
			"No pending rec request but mic is already on => Proceeding");
		codec_data->pending_rec = DMIC_MCU_ON_ON;
	}

	// Double checking there is indeed a pending recording going on
	if (codec_data->pending_rec) {
		// Double check the requested sampling rate is a valid one.
		if (sample_rate != MCU_PCM_RATE_8000
			&& sample_rate != MCU_PCM_RATE_16000
			&& sample_rate != MCU_PCM_RATE_48000) {
			dev_err(component->dev,
				"Invalid Sample Rate: %u. (Valid rates: %d, %d, %d Hz)\n",
				sample_rate, MCU_PCM_RATE_8000, MCU_PCM_RATE_16000,
				MCU_PCM_RATE_48000);
			ret = -EINVAL;
			goto end;
		}

		dev_info(component->dev, "Sampling at: %u\n", sample_rate);

		// Set MCU sampling rate with the one requested by the AP to the ADSP
		ret = dmic_mcu_send_message(component->dev,
			DMIC_MCU_MESSAGE_SAMPLE_RATE_KHZ,
			(int)(sample_rate / 1000));
		if (ret != 0) {
			dev_err(component->dev,
				"Error sending DMIC_MCU_MESSAGE_SAMPLE_RATE_KHZ: %d", ret);
			ret = -EIO;
			goto end;
		}
		codec_data->sample_rate_hz = sample_rate;

		// Set MCU gain if none was requested through tinymix controls.
		// Note that only when using tinymix/audio factory commands would the gain
		// be set. In all other cases through regular Android operations, the gain
		// will always be set here, not in the mixer_paths_monaco_idp_google.xml
		if (!codec_data->gain_user_requested) {
			int value = 0;
			if (sample_rate == MCU_PCM_RATE_8000)
				value = GAIN_FOR_8000;
			else if (sample_rate == MCU_PCM_RATE_16000)
				value = GAIN_FOR_16000;
			else // sample_rate == MCU_PCM_RATE_48000
				value = GAIN_FOR_48000;

			ret = dmic_mcu_send_message(component->dev, DMIC_MCU_MESSAGE_GAIN, value);
			if (ret != 0) {
				dev_err(component->dev,
					"Error sending DMIC_MCU_MESSAGE_GAIN: %d", ret);
				ret = -EIO;
				goto end;
			}
			codec_data->gain = value;
		}

		// Update to conditional ON/OFF in case of screen off AND DSP HW only
		if (codec_data->screen_off_dsp_hw_on &&
			(codec_data->pending_rec == DMIC_MCU_ON_ON))
			conditionalized_value = DMIC_MCU_ON_ON_CONDITIONAL;
		else if (codec_data->screen_off_dsp_hw_on &&
			(codec_data->pending_rec == DMIC_MCU_ON_OFF))
			conditionalized_value = DMIC_MCU_ON_OFF_CONDITIONAL;
		else
			conditionalized_value = codec_data->pending_rec;

		// Send message to MCU to start recording
		ret = dmic_mcu_send_message(component->dev, DMIC_MCU_MESSAGE_MIC_ON,
			conditionalized_value);
		if (ret != 0) {
			dev_err(component->dev,
				"Error sending DMIC_MCU_MESSAGE_MIC_ON: %d", ret);
			ret = -EIO;
			goto end;
		}
		codec_data->mic_on = codec_data->pending_rec;
	} else {
		dev_err(component->dev, "Called without calling 'DMIC_MCU On' first");
		ret = -EINVAL;
	}

end:
	codec_data->gain_user_requested = false;
	codec_data->pending_rec = DMIC_MCU_ON_OFF;

	mutex_unlock(&codec_data->iolock);

	return ret;
}

static struct snd_soc_dai_ops mcu_mic_dai_ops = {
	.hw_params = mcu_mic_hw_params,
};

static struct snd_soc_dai_driver mcu_mic_soc_dai_driver = {
	.name = "mcu_mic_codec_dai",
	.capture = {
			.stream_name = "MCU Mic DAI Capture",
			.channels_min = 1,
			.channels_max = 1,
			.rates = MIC_MCU_RATES,
			.formats = MIC_MCU_FORMATS,
		},
	.ops = &mcu_mic_dai_ops,
};

static void on_message_received(const char *buffer, size_t length)
{
	struct mcu_mic_codec_data *codec_data;
	if (length != 3 ||
		buffer[0] != DMIC_MCU_MESSAGE_VERSION ||
		buffer[1] != DMIC_MCU_MESSAGE_MIC_READY) {
		return;
	}

	if(!priv_platform_device) {
		pr_err("priv_platform_device is not ready for external access\n");
		return;
	}

	codec_data = platform_get_drvdata(priv_platform_device);
	if (!codec_data) {
		dev_err(&priv_platform_device->dev, "Failed to get codec_data\n");
		return;
	}

	mutex_lock(&codec_data->iolock);
	// hw_enabled and rtt_enabled need to be sent first
	if (codec_data->hw_enabled) {
		dev_info(&priv_platform_device->dev,
			"Resend DSP Hotword enable notification\n");
		dmic_mcu_send_message(&priv_platform_device->dev,
			DMIC_MCU_MESSAGE_HOTWORD_STATUS,
			(int)(codec_data->hw_enabled));
	}
	if (codec_data->rtt_enabled) {
		dev_info(&priv_platform_device->dev,
			"Resend RTT Hotword enable notification\n");
		dmic_mcu_send_message(&priv_platform_device->dev,
			DMIC_MCU_MESSAGE_RTT_STATUS,
			(int)(codec_data->rtt_enabled));
	}
	if (codec_data->mic_on == DMIC_MCU_ON_ON) {
		dev_info(&priv_platform_device->dev,
			"Audio recording is on-going, resend parameters to restart "
			"the recording\n");
		dmic_mcu_send_message(&priv_platform_device->dev,
			DMIC_MCU_MESSAGE_SAMPLE_RATE_KHZ,
			(int)(codec_data->sample_rate_hz / 1000));
		dmic_mcu_send_message(&priv_platform_device->dev,
			DMIC_MCU_MESSAGE_GAIN, codec_data->gain);
		if (codec_data->screen_off_dsp_hw_on) {
			dmic_mcu_send_message(&priv_platform_device->dev,
				DMIC_MCU_MESSAGE_MIC_ON, DMIC_MCU_ON_ON_CONDITIONAL);
		} else {
			dmic_mcu_send_message(&priv_platform_device->dev,
				DMIC_MCU_MESSAGE_MIC_ON, DMIC_MCU_ON_ON);
		}
	}
	mutex_unlock(&codec_data->iolock);
}

static int mcu_mic_probe(struct platform_device *pdev)
{
	int ret;
	struct mcu_mic_codec_data *codec_data;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	codec_data = devm_kzalloc(&pdev->dev, sizeof(struct mcu_mic_codec_data),
				  GFP_KERNEL);
	if (!codec_data) {
		dev_err(&pdev->dev, "No memory for codec_data\n");
		return -ENOMEM;
	}
	codec_data->mic_on = DMIC_MCU_ON_OFF;
	codec_data->sample_rate_hz = MIC_DEFAULT_SAMPLE_RATE;
	codec_data->gain = MIC_DEFAULT_GAIN;
	codec_data->hw_enabled = false;
	codec_data->rtt_enabled = false;
	codec_data->screen_off_dsp_hw_on = false;

	platform_set_drvdata(pdev, codec_data);

	mutex_init(&codec_data->iolock);

	priv_platform_device = pdev;

	ret = devm_snd_soc_register_component(&pdev->dev,
		&mcu_mic_soc_component_driver,
		&mcu_mic_soc_dai_driver,
		1);

	nanohub_register_listener(NANOHUB_AUDIO_CHANNEL_ID, &on_message_received);

	dev_dbg(&pdev->dev, "%s: ret: %d\n", __func__, ret);

	return ret;
}

static int mcu_mic_remove(struct platform_device *pdev)
{
	struct mcu_mic_codec_data *codec_data;
	nanohub_unregister_listener(NANOHUB_AUDIO_CHANNEL_ID);

	priv_platform_device = NULL;

	codec_data = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	mutex_destroy(&codec_data->iolock);

	return 0;
}


const struct of_device_id mcu_mic_of_match[] = {
	{
		.compatible = "google,mcu_mic_codec",
	},
	{},
};
MODULE_DEVICE_TABLE(of, mcu_mic_of_match);

static struct platform_driver mcu_mic_driver = {
	.driver = {
			.name = "mcu_mic_codec",
			.owner = THIS_MODULE,
			.of_match_table = of_match_ptr(mcu_mic_of_match),
		},
	.probe = mcu_mic_probe,
	.remove = mcu_mic_remove,
};

/* Register the platform driver */
module_platform_driver(mcu_mic_driver);

MODULE_DESCRIPTION("ASoC MCU Mic codec driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mcu_mic_codec");
