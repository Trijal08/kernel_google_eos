/*
 * Platform Dependent file
 *
 * Copyright (C) 2021, Broadcom.
 * Copyright (C) 2023, Synaptics.
 * Copyright (C) 2023, Google Inc.
 *
 *     Unless you, Broadcom, Google Inc, Synaptics execute a separate written
 * software license agreement governing use of this software, this software is
 * licensed to you under the terms of the GNU General Public License version 2
 * (the "GPL"), available at http://www.broadcom.com/licenses/GPLv2.php, with
 * the following added to such license:
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
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/mmc/host.h>
#include <linux/skbuff.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#ifdef CONFIG_WIFI_CONTROL_FUNC
#include <linux/wlan_plat.h>
#else
#include <dhd_plat.h>
#endif /* CONFIG_WIFI_CONTROL_FUNC */
#include <dhd_dbg.h>
#include <dhd.h>
#ifdef DHD_COREDUMP
#include <soc/qcom/qcom_ramdump.h>
#include <linux/list.h>
#endif /* DHD_COREDUMP */


#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
extern int dhd_init_wlan_mem(void);
extern void *dhd_wlan_mem_prealloc(int section, unsigned long size);
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

static int wlan_reg_on = -1;
#define DHD_DT_COMPAT_ENTRY		"android,syna_wlan"
#define WIFI_WL_REG_ON_PROPNAME		"wl_reg_on"

static int wlan_host_wake_up = -1;
#ifdef CONFIG_BCMDHD_OOB_HOST_WAKE
static int wlan_host_wake_irq = 0;
#endif /* CONFIG_BCMDHD_OOB_HOST_WAKE */
#define WIFI_WLAN_HOST_WAKE_PROPNAME    "wl_host_wake"

#ifdef DHD_COREDUMP
#define DEVICE_NAME "wlan"

void *qcom_create_ramdump_device(const char *dev_name, struct device *parent);
void qcom_destroy_ramdump_device(void *dev);
int qcom_dump(struct list_head *head, struct device *dev);

static struct device *ramdump_dev;
#endif /* DHD_COREDUMP */

#ifdef GET_CUSTOM_MAC_ENABLE

#define CDB_PATH "/chosen/config"
#define WIFI_MAC "wlan_mac1"
static u8 wlan_mac[6] = {0};

static int
dhd_wlan_get_mac_addr(unsigned char *buf)
{
    if (memcmp(wlan_mac, "\0\0\0\0\0\0", 6)) {
        memcpy(buf, wlan_mac, sizeof(wlan_mac));
        return 0;
    }
    return -EIO;
}

static int
dhd_wlan_init_mac_addr(void)
{
    u8 mac[6] = {0};
    unsigned int size;
    unsigned char *mac_addr = NULL;
    struct device_node *node;
    unsigned int mac_found = 0;

    node = of_find_node_by_path(CDB_PATH);
    if (!node) {
        DHD_ERROR(("CDB Node not created under %s\n", CDB_PATH));
        return -ENODEV;
    } else {
        mac_addr = (unsigned char *)
                of_get_property(node, WIFI_MAC, &size);
    }

    /* In case Missing Provisioned MAC Address, exit with error */
    if (!mac_addr) {
        DHD_ERROR(("Missing Provisioned MAC address\n"));
        return -EINVAL;
    }

    /* Start decoding MAC Address
     * Note that 2 formats are supported for now
     * AA:BB:CC:DD:EE:FF (with separating colons) and
     * AABBCCDDEEFF (without separating colons)
     */
    if (sscanf(mac_addr,
            "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
            &mac[0], &mac[1], &mac[2], &mac[3], &mac[4],
            &mac[5]) == 6) {
        mac_found = 1;
    } else if (sscanf(mac_addr,
            "%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx",
            &mac[0], &mac[1], &mac[2], &mac[3], &mac[4],
            &mac[5]) == 6) {
        mac_found = 1;
    }

    /* Make sure Address decoding succeeds */
    if (!mac_found) {
        DHD_ERROR(("Invalid format for Provisioned MAC Address\n"));
        return -EINVAL;
    }

    /* Make sure Provisioned MAC Address is globally Administered */
    if (mac[0] & 2) {
        DHD_ERROR(("Invalid Provisioned MAC Address\n"));
        return -EINVAL;
    }

    memcpy(wlan_mac, mac, sizeof(mac));
    return 0;
}
#endif /* GET_CUSTOM_MAC_ENABLE */

#ifdef DHD_COREDUMP
/* trigger coredump */
static int
dhd_set_coredump(const char *buf, int buf_len, const char *info)
{
	struct qcom_dump_segment segment;
	struct list_head head;

	if (ramdump_dev) {
		INIT_LIST_HEAD(&head);
		memset(&segment, 0, sizeof(segment));
		segment.va = (void *) buf;
		segment.size = buf_len;
		list_add(&segment.node, &head);

		qcom_dump(&head, ramdump_dev);
	}
	return 0;
}
#endif /* DHD_COREDUMP */

#if defined(SUPPORT_MULTIPLE_NVRAM)
enum {
	CHIP_REV_SKU = 0,
	CHIP_REV = 1,
	CHIP_SKU = 2,
	CHIP = 3,
	REV_SKU = 4,
	REV_ONLY = 5,
	SKU_ONLY = 6,
	NO_EXT_NAME = 7,
	MAX_FILE_COUNT
};

#define PLT_PATH "/chosen/plat"

#ifndef CDB_PATH
#define CDB_PATH "/chosen/config"
#endif /* CDB_PATH */

#define HW_SKU    "sku"
#define HW_STAGE  "stage"
#define HW_MAJOR  "major"
#define HW_MINOR  "minor"
#define HW_PRODUCT  "product"

char val_revision[MAX_HW_INFO_LEN] = "NA";
char val_sku[MAX_HW_INFO_LEN] = "NA";
char val_product[MAX_HW_INFO_LEN] = "NA";

typedef struct {
    char hw_id[MAX_HW_INFO_LEN];
    char sku[MAX_HW_INFO_LEN];
} sku_info_t;

sku_info_t sku_table[] = {
	0
};

typedef struct {
    int id;
    char product[MAX_HW_INFO_LEN];
} product_info_t;

product_info_t product_table[] = {
	{8,{"FSE22"}},
	{9,{"FSE22"}},
	{10,{"FL22"}},
	{11,{"FL22"}},
};

enum hw_stage_attr {
	DEV = 1,
	PROTO = 2,
	EVT = 3,
	DVT = 4,
	PVT = 5,
	MP = 6,
	HW_STAGE_MAX
};
typedef struct platform_hw_info {
	uint8 avail_bmap;
	char ext_name[MAX_FILE_COUNT][MAX_HW_EXT_LEN];
} platform_hw_info_t;
platform_hw_info_t platform_hw_info;

static void
dhd_set_platform_ext_name(char *hw_rev, char* hw_sku)
{
	bzero(&platform_hw_info, sizeof(platform_hw_info_t));

	if (strncmp(hw_rev, "NA", MAX_HW_INFO_LEN) != 0) {
		if (strncmp(hw_sku, "NA", MAX_HW_INFO_LEN) != 0) {
			snprintf(platform_hw_info.ext_name[REV_SKU], MAX_HW_EXT_LEN, "_%s_%s",
				hw_rev, hw_sku);
			setbit(&platform_hw_info.avail_bmap, REV_SKU);
		}
		snprintf(platform_hw_info.ext_name[REV_ONLY], MAX_HW_EXT_LEN, "_%s", hw_rev);
		setbit(&platform_hw_info.avail_bmap, REV_ONLY);
	}

	if (strncmp(hw_sku, "NA", MAX_HW_INFO_LEN) != 0) {
		snprintf(platform_hw_info.ext_name[SKU_ONLY], MAX_HW_EXT_LEN, "_%s", hw_sku);
		setbit(&platform_hw_info.avail_bmap, SKU_ONLY);
	}

#ifdef USE_CID_CHECK
	setbit(&platform_hw_info.avail_bmap, NO_EXT_NAME);
#endif /* USE_CID_CHECK */

	return;
}

void
dhd_set_platform_ext_name_for_chip_version(char* chip_version)
{
	if (strncmp(val_revision, "NA", MAX_HW_INFO_LEN) != 0) {
		if (strncmp(val_sku, "NA", MAX_HW_INFO_LEN) != 0) {
			snprintf(platform_hw_info.ext_name[CHIP_REV_SKU], MAX_HW_EXT_LEN,
				"%s_%s_%s", chip_version, val_revision, val_sku);
			setbit(&platform_hw_info.avail_bmap, CHIP_REV_SKU);
		}

		snprintf(platform_hw_info.ext_name[CHIP_REV], MAX_HW_EXT_LEN, "%s_%s",
			chip_version, val_revision);
		setbit(&platform_hw_info.avail_bmap, CHIP_REV);
	}
	if (strncmp(val_sku, "NA", MAX_HW_INFO_LEN) != 0) {
		snprintf(platform_hw_info.ext_name[CHIP_SKU], MAX_HW_EXT_LEN, "%s_%s",
			chip_version, val_sku);
		setbit(&platform_hw_info.avail_bmap, CHIP_SKU);
	}

	snprintf(platform_hw_info.ext_name[CHIP], MAX_HW_EXT_LEN, "%s", chip_version);
	setbit(&platform_hw_info.avail_bmap, CHIP);

	return;
}

static int
dhd_check_file_exist(char* fname)
{
	int err = BCME_OK;
#ifdef DHD_LINUX_STD_FW_API
	const struct firmware *fw = NULL;
#else
	struct file *filep = NULL;
	mm_segment_t fs;
#endif /* DHD_LINUX_STD_FW_API */

	if (fname == NULL) {
		DHD_ERROR(("%s: ERROR fname is NULL \n", __FUNCTION__));
		return BCME_ERROR;
	}

#ifdef DHD_LINUX_STD_FW_API
	err = dhd_os_get_img_fwreq(&fw, fname);
	if (err < 0) {
		DHD_LOG_MEM(("dhd_os_get_img(Request Firmware API) error : %d\n",
			err));
		goto fail;
	}
#else
	fs = get_fs();
	set_fs(KERNEL_DS);

	filep = dhd_filp_open(fname, O_RDONLY, 0);
	if (IS_ERR(filep) || (filep == NULL)) {
		DHD_LOG_MEM(("%s: Failed to open %s \n",  __FUNCTION__, fname));
		err = BCME_NOTFOUND;
		goto fail;
	}
#endif /* DHD_LINUX_STD_FW_API */

fail:
#ifdef DHD_LINUX_STD_FW_API
	if (fw) {
		dhd_os_close_img_fwreq(fw);
	}
#else
	if (!IS_ERR(filep))
		dhd_filp_close(filep, NULL);

	set_fs(fs);
#endif /* DHD_LINUX_STD_FW_API */
	return err;
}

int
dhd_get_platform_naming_for_nvram_clmblob_file(download_type_t component, char *file_name)
{
	int i, error = BCME_OK;
	char tmp_fname[MAX_FILE_LEN] = {0};

	if (!platform_hw_info.avail_bmap) {
		DHD_ERROR(("ext_name is not composed.\n"));
		return BCME_ERROR;
	}

	for (i = 0; i < MAX_FILE_COUNT; i++) {
		if (!isset(&platform_hw_info.avail_bmap, i)) {
			continue;
		}
		memset_s(tmp_fname, MAX_FILE_LEN, 0, MAX_FILE_LEN);
		snprintf(tmp_fname, MAX_FILE_LEN,
			"%s%s", file_name, platform_hw_info.ext_name[i]);
		error = dhd_check_file_exist(tmp_fname);
		if (error == BCME_OK) {
			DHD_LOG_MEM(("%02d path[%s]\n", i, tmp_fname));
			strlcpy(file_name, tmp_fname, MAX_FILE_LEN);
			break;
		}
	}
	return error;
}

static int
dhd_wlan_init_hardware_info(void)
{

	struct device_node *node = NULL;
	const char *hw_sku = NULL;
	int hw_stage = -1;
	int hw_major = -1;
	int hw_minor = -1;
	int hw_product = -1;
	int i;

	node = of_find_node_by_path(PLT_PATH);
	if (!node) {
		DHD_ERROR(("Node not created under %s\n", PLT_PATH));
		goto exit;
	} else {

		if (of_property_read_u32(node, HW_STAGE, &hw_stage)) {
			DHD_ERROR(("%s: Failed to get hw stage\n", __FUNCTION__));
			goto exit;
		}

		if (of_property_read_u32(node, HW_MAJOR, &hw_major)) {
			DHD_ERROR(("%s: Failed to get hw major\n", __FUNCTION__));
			goto exit;
		}

		if (of_property_read_u32(node, HW_MINOR, &hw_minor)) {
			DHD_ERROR(("%s: Failed to get hw minor\n", __FUNCTION__));
			goto exit;
		}

		switch (hw_stage) {
			case DEV:
				snprintf(val_revision, MAX_HW_INFO_LEN, "DEV%d.%d",
					hw_major, hw_minor);
				break;
			case PROTO:
				snprintf(val_revision, MAX_HW_INFO_LEN, "PROTO%d.%d",
					hw_major, hw_minor);
				break;
			case EVT:
				snprintf(val_revision, MAX_HW_INFO_LEN, "EVT%d.%d",
					hw_major, hw_minor);
				break;
			case DVT:
				snprintf(val_revision, MAX_HW_INFO_LEN, "DVT%d.%d",
					hw_major, hw_minor);
				break;
			case PVT:
				snprintf(val_revision, MAX_HW_INFO_LEN, "PVT%d.%d",
					hw_major, hw_minor);
				break;
			case MP:
				snprintf(val_revision, MAX_HW_INFO_LEN, "MP%d.%d",
					hw_major, hw_minor);
				break;
			default:
				strcpy(val_revision, "NA");
				break;
		}

		// Read hw product
		if (of_property_read_u32(node, HW_PRODUCT, &hw_product)) {
			DHD_ERROR(("%s: Failed to get hw product\n", __FUNCTION__));
		} else {
			for (i = 0; i < ARRAYSIZE(product_table); i ++) {
				if (hw_product == product_table[i].id) {
					strcpy(val_product, product_table[i].product);
					break;
				}
			}
		}
		DHD_ERROR(("%s: hw_product is %d, val_product is %s\n", __FUNCTION__, hw_product, val_product));
	}

	node = of_find_node_by_path(CDB_PATH);
	if (!node) {
		DHD_ERROR(("Node not created under %s\n", CDB_PATH));
		goto exit;
	} else {

		if (of_property_read_string(node, HW_SKU, &hw_sku)) {
			DHD_ERROR(("%s: Failed to get hw sku\n", __FUNCTION__));
		} else {
			for (i = 0; i < ARRAYSIZE(sku_table); i ++) {
				if (strcmp(hw_sku, sku_table[i].hw_id) == 0) {
					strcpy(val_sku, sku_table[i].sku);
					break;
				}
			}
		}
		DHD_ERROR(("%s: hw_sku is %s, val_sku is %s\n", __FUNCTION__, hw_sku, val_sku));
	}

	// Combine product and sku into val_sku for an extension
	if (strcmp(val_product, "NA") != 0) {
		if (strcmp(val_sku, "NA") != 0)
			snprintf(val_sku, MAX_HW_INFO_LEN, "_%s", val_product);
		else
			strcpy(val_sku, val_product);
	}

exit:
	dhd_set_platform_ext_name(val_revision, val_sku);

	return 0;
}
#endif /* SUPPORT_MULTIPLE_NVRAM */

static int
dhd_wifi_init_gpio(void)
{
	int gpio_reg_on_val;
	/* ========== WLAN_PWR_EN ============ */
	char *wlan_node = DHD_DT_COMPAT_ENTRY;
	struct device_node *root_node = NULL;
	struct platform_device *wlan_pdev = NULL;
	struct pinctrl_state *wlan_gpios_default_state;

	root_node = of_find_compatible_node(NULL, NULL, wlan_node);
	if (!root_node) {
		DHD_ERROR(("failed to get device node of SYNA WLAN\n"));
		return -ENODEV;
	}

/* The current design on CONFIG_SOC_GOOGLE is executing pinctrl in PCIe driver.
(Not this project). Since we are using SDIO as bus, so pinctrl is handled
in the wlan driver(here).
Note: Review it when enabling CONFIG_SOC_GOOGLE.
*/
#ifndef CONFIG_SOC_GOOGLE
	/* Lookup the wlan device */
	wlan_pdev = of_find_device_by_node(root_node);
	if (!wlan_pdev) {
		DHD_ERROR(("failed to get device of SYNA WLAN\n"));
		return -ENODEV;
	}
	/* Lookup the default state pins settings*/
	wlan_gpios_default_state = pinctrl_lookup_state(
		devm_pinctrl_get(&wlan_pdev->dev), "default");
	/* Actually write the default configuration to hardware */
	if (pinctrl_select_state(devm_pinctrl_get(&wlan_pdev->dev),
							 wlan_gpios_default_state)) {
		DHD_ERROR(("Invalid initial wlan pins to default configuration\n"));
		return -ENODEV;
	}
#endif

	wlan_reg_on = of_get_named_gpio(root_node, WIFI_WL_REG_ON_PROPNAME, 0);
	if (!gpio_is_valid(wlan_reg_on)) {
		DHD_ERROR(("Invalid gpio pin : %d\n", wlan_reg_on));
		return -ENODEV;
	}

	/* ========== WLAN_PWR_EN ============ */
	DHD_INFO(("%s: gpio_wlan_power : %d\n", __FUNCTION__, wlan_reg_on));

	/*
	 * For reg_on, gpio_request will fail if the gpio is configured to output-high
	 * in the dts using gpio-hog, so do not return error for failure.
	 */
	if (gpio_request_one(wlan_reg_on, GPIOF_OUT_INIT_HIGH, "WL_REG_ON")) {
		DHD_ERROR(("%s: Failed to request gpio %d for WL_REG_ON, "
			"might have configured in the dts\n",
			__FUNCTION__, wlan_reg_on));
	} else {
		DHD_ERROR(("%s: gpio_request WL_REG_ON done - WLAN_EN: GPIO %d\n",
			__FUNCTION__, wlan_reg_on));
	}

	gpio_reg_on_val = gpio_get_value(wlan_reg_on);
	DHD_INFO(("%s: Initial WL_REG_ON: [%d]\n",
		__FUNCTION__, gpio_get_value(wlan_reg_on)));

	if (gpio_reg_on_val == 0) {
		DHD_INFO(("%s: WL_REG_ON is LOW, drive it HIGH\n", __FUNCTION__));
		if (gpio_direction_output(wlan_reg_on, 1)) {
			DHD_ERROR(("%s: WL_REG_ON is failed to pull up\n", __FUNCTION__));
			return -EIO;
		}
	}

	DHD_ERROR(("%s: WL_REG_ON is pulled up\n", __FUNCTION__));

	/* Wait for WIFI_TURNON_DELAY due to power stability */
	msleep(WIFI_TURNON_DELAY);
#ifdef CONFIG_BCMDHD_OOB_HOST_WAKE
	/* ========== WLAN_HOST_WAKE ============ */
	wlan_host_wake_up = of_get_named_gpio(root_node,
		WIFI_WLAN_HOST_WAKE_PROPNAME, 0);
	DHD_INFO(("%s: gpio_wlan_host_wake : %d\n", __FUNCTION__, wlan_host_wake_up));

	if (gpio_request_one(wlan_host_wake_up, GPIOF_IN, "WLAN_HOST_WAKE")) {
		DHD_ERROR(("%s: Failed to request gpio %d for WLAN_HOST_WAKE\n",
			__FUNCTION__, wlan_host_wake_up));
			return -ENODEV;
	} else {
		DHD_ERROR(("%s: gpio_request WLAN_HOST_WAKE done"
			" - WLAN_HOST_WAKE: GPIO %d\n",
			__FUNCTION__, wlan_host_wake_up));
	}

	if (gpio_direction_input(wlan_host_wake_up)) {
		DHD_ERROR(("%s: Failed to set WL_HOST_WAKE gpio direction\n", __FUNCTION__));
	}

	wlan_host_wake_irq = gpio_to_irq(wlan_host_wake_up);
#endif /* CONFIG_BCMDHD_OOB_HOST_WAKE */
	return 0;
}

int
dhd_wlan_power(int onoff)
{
	DHD_INFO(("------------------------------------------------\n"));
	DHD_INFO(("------------------------------------------------\n"));
	DHD_INFO(("%s Enter: power %s\n", __func__, onoff ? "on" : "off"));

	if (onoff) {
		if (gpio_direction_output(wlan_reg_on, 1)) {
			DHD_ERROR(("%s: WL_REG_ON is failed to pull up\n", __FUNCTION__));
			return -EIO;
		}
		if (gpio_get_value(wlan_reg_on)) {
			DHD_ERROR(("WL_REG_ON on-step-2 : [%d]\n",
				gpio_get_value(wlan_reg_on)));
		} else {
			DHD_ERROR(("[%s] gpio value is 0. We need reinit.\n", __func__));
			if (gpio_direction_output(wlan_reg_on, 1)) {
				DHD_ERROR(("%s: WL_REG_ON is "
					"failed to pull up\n", __func__));
			}
		}
	} else {
		if (gpio_direction_output(wlan_reg_on, 0)) {
			DHD_ERROR(("%s: WL_REG_ON is failed to pull up\n", __FUNCTION__));
			return -EIO;
		}
		if (gpio_get_value(wlan_reg_on)) {
			DHD_ERROR(("WL_REG_ON on-step-2 : [%d]\n",
				gpio_get_value(wlan_reg_on)));
		}
	}
	return 0;
}
EXPORT_SYMBOL(dhd_wlan_power);

static int
dhd_wlan_reset(int onoff)
{
	return 0;
}

/* function is defined in drivers/mmc/core/core.c */
extern void mmc_detect_change(struct mmc_host *host, unsigned long delay);
extern struct mmc_host *msm_wifi_mmc_host_get(void);

static int
dhd_wlan_set_carddetect(int val)
{
	struct mmc_host *mmc = NULL;
	if (NULL == (mmc = msm_wifi_mmc_host_get())) {
		return -ENODEV;
	}
	// The wifi card is marked to unremovable in dtsi, but the wifi card
	// didn't ready when mmc host driver has been probed. Force
	// rescan_entered to 0 to trigger detect wifi card.
	mmc->rescan_entered = 0;
	mmc_detect_change(mmc, 0 /* no delay */);
	return 0;
}

#ifdef BCMSDIO
static int dhd_wlan_get_wake_irq(void)
{
	return gpio_to_irq(wlan_host_wake_up);
}
#endif /* BCMSDIO */

#if defined(CONFIG_BCMDHD_OOB_HOST_WAKE) && defined(CONFIG_BCMDHD_GET_OOB_STATE)
int
dhd_get_wlan_oob_gpio(void)
{
	return gpio_is_valid(wlan_host_wake_up) ?
		gpio_get_value(wlan_host_wake_up) : -1;
}
EXPORT_SYMBOL(dhd_get_wlan_oob_gpio);
int
dhd_get_wlan_oob_gpio_number(void)
{
	return gpio_is_valid(wlan_host_wake_up) ?
		wlan_host_wake_up : -1;
}
EXPORT_SYMBOL(dhd_get_wlan_oob_gpio_number);
#endif /* CONFIG_BCMDHD_OOB_HOST_WAKE && CONFIG_BCMDHD_GET_OOB_STATE */

struct resource dhd_wlan_resources = {
	.name	= "bcmdhd_wlan_irq",
	.start	= 0, /* Dummy */
	.end	= 0, /* Dummy */
	.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE |
#ifdef CONFIG_BCMDHD_PCIE
	IORESOURCE_IRQ_HIGHEDGE,
#else
	IORESOURCE_IRQ_HIGHLEVEL,
#endif /* CONFIG_BCMDHD_PCIE */
};
EXPORT_SYMBOL(dhd_wlan_resources);

struct wifi_platform_data dhd_wlan_control = {
	.set_power	= dhd_wlan_power,
	.set_reset	= dhd_wlan_reset,
	.set_carddetect	= dhd_wlan_set_carddetect,
#ifdef DHD_COREDUMP
	.set_coredump = dhd_set_coredump,
#endif /* DHD_COREDUMP */
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	.mem_prealloc	= dhd_wlan_mem_prealloc,
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */
#ifdef GET_CUSTOM_MAC_ENABLE
        .get_mac_addr = dhd_wlan_get_mac_addr,
#endif /* GET_CUSTOM_MAC_ENABLE */
#ifdef BCMSDIO
	.get_wake_irq	= dhd_wlan_get_wake_irq,
#endif // endif
};
EXPORT_SYMBOL(dhd_wlan_control);

static inline void
dhd_wlan_set_reg_off(void) {
	if (gpio_is_valid(wlan_reg_on) && gpio_get_value(wlan_reg_on) != 0) {
		if (gpio_direction_output(wlan_reg_on, 0)) {
			DHD_ERROR(("%s: WL_REG_ON failed to pull down\n", __FUNCTION__));
		} else {
			DHD_ERROR(("%s: WL_REG_ON has been pulled down\n", __FUNCTION__));
		}
	}
}

int
dhd_wlan_init(void)
{
	int ret;

	DHD_INFO(("%s: START.......\n", __FUNCTION__));
#ifdef DHD_COREDUMP
	ramdump_dev = qcom_create_ramdump_device(DEVICE_NAME, NULL);
	if (!ramdump_dev)
		DHD_ERROR(("Unable to create %s ramdump device.\n", DEVICE_NAME));

#endif /* DHD_COREDUMP */

	ret = dhd_wifi_init_gpio();
	if (ret < 0) {
		DHD_ERROR(("%s: failed to initiate GPIO, ret=%d\n",
			__FUNCTION__, ret));
		goto fail;
	}
#ifdef CONFIG_BCMDHD_OOB_HOST_WAKE
	dhd_wlan_resources.start = wlan_host_wake_irq;
	dhd_wlan_resources.end = wlan_host_wake_irq;
#endif /* CONFIG_BCMDHD_OOB_HOST_WAKE */

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	ret = dhd_init_wlan_mem();
	if (ret < 0) {
		DHD_ERROR(("%s: failed to alloc reserved memory,"
					" ret=%d\n", __FUNCTION__, ret));
		goto fail;
	}
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

#ifdef GET_CUSTOM_MAC_ENABLE
	dhd_wlan_init_mac_addr();
#endif /* GET_CUSTOM_MAC_ENABLE */

#if defined(SUPPORT_MULTIPLE_NVRAM)
	dhd_wlan_init_hardware_info();
#endif /* SUPPORT_MULTIPLE_NVRAM */

fail:
	if (ret < 0) {
		/* wlan init failed, turn off WL_REG_ON */
		dhd_wlan_set_reg_off();
	}

	DHD_ERROR(("%s: FINISH.......\n", __FUNCTION__));
	return ret;
}

int
dhd_wlan_deinit(void)
{
	if (gpio_is_valid(wlan_host_wake_up)) {
		gpio_free(wlan_host_wake_up);
	}
	if (gpio_is_valid(wlan_reg_on)) {
		dhd_wlan_set_reg_off();
		gpio_free(wlan_reg_on);
	}

#ifdef DHD_COREDUMP
	if (ramdump_dev)
		qcom_destroy_ramdump_device(ramdump_dev);
#endif /* DHD_COREDUMP */
	return 0;
}
