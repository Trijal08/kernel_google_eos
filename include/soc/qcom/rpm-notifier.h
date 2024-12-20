/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __ARCH_ARM_MACH_MSM_RPM_NOTIF_H
#define __ARCH_ARM_MACH_MSM_RPM_NOTIF_H

struct msm_rpm_notifier_data {
	uint32_t rsc_type;
	uint32_t rsc_id;
	uint32_t key;
	uint32_t size;
	uint8_t *value;
};
/**
 * msm_rpm_register_notifier - Register for sleep set notifications
 *
 * @nb - notifier block to register
 *
 * return 0 on success, errno on failure.
 */
int msm_rpm_register_notifier(struct notifier_block *nb);

/**
 * msm_rpm_unregister_notifier - Unregister previously registered notifications
 *
 * @nb - notifier block to unregister
 *
 * return 0 on success, errno on failure.
 */
int msm_rpm_unregister_notifier(struct notifier_block *nb);

/**
 * msm_rpm_waiting_for_ack - Indicate if there is RPM message
 *				pending acknowledgment.
 * returns true for pending messages and false otherwise
 */
bool msm_rpm_waiting_for_ack(void);

#endif /*__ARCH_ARM_MACH_MSM_RPM_NOTIF_H */
