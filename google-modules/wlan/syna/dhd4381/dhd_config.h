/*
 * DHD CONFIG INI
 *
 * Copyright (C) 2023, Broadcom.
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id$
 */

#ifndef __DHD_CONFIG_H__
#define __DHD_CONFIG_H__

#include <dhd.h>

// Added for 'roam_score_coeff'
#define CONST_WL_ROAM_SCORE_COEFF_QTY    10
typedef struct roam_score_coeff {
	uint32  index;
	uint32  count;
	uint32  coeff[CONST_WL_ROAM_SCORE_COEFF_QTY];
} roam_score_coeff_t;

int dhd_preinit_config(dhd_pub_t *dhd, int ifidx, char * config_file_path);
int dhd_preinit_config_proc(dhd_pub_t *dhd, int ifidx, char *name, char *value);

#endif // __DHD_CONFIG_H__
