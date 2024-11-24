#ifndef __NANOHUB_NANOHUB_H
#define __NANOHUB_NANOHUB_H
#include <linux/types.h>
struct nanohub_flash_bank {
	int bank;
	u32 address;
	size_t length;
};
struct nanohub_platform_data {
	u32 wakeup_gpio;
	u32 nreset_gpio;
	u32 boot0_gpio;
	u32 boot2_gpio;
	u32 irq1_gpio;
	u32 irq2_gpio;
	u32 spi_cs_gpio;
	struct regmap *pmic_regmap;
	u32 afe_control_reg;
	u32 vddcore_control_reg;
	u32 vddcore_enable_reg;
	u32 vddcore_voltage;
	u32 clk32_control_reg;
	u32 bob_ext_ctrl1_control_reg;
	u32 s5a_pgood_state_reg;
	int supplies_cnt;
	struct regulator **supplies;
	u32 reg_en_gpio;
#ifdef CONFIG_NANOHUB_BL_ST
	u32 bl_max_speed_hz;
	u32 bl_addr;
	u32 num_flash_banks;
	struct nanohub_flash_bank *flash_banks;
	u32 num_shared_flash_banks;
	struct nanohub_flash_bank *shared_flash_banks;
#endif
	const char *firmware_name;
};
#endif /* __NANOHUB_NANOHUB_H */
