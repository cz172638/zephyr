/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * AP/MCU JTAG (SWD) on the micro SD pins, see rockchip,rk3399-sd-jtag.
 * The GRF is written directly: flat mapped on the Cortex-A images, in the
 * 0x40000000 peripheral window on the Cortex-M0 images.
 */

#define DT_DRV_COMPAT rockchip_rk3399_sd_jtag

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util.h>

/* GRF_GPIO4B_IOMUX: 2 bits per pin, function 2 = JTAG; upper half write mask */
#define IOMUX_AP_MASK   0x00f0U /* B2, B3 */
#define IOMUX_AP_JTAG   0x00a0U
#define IOMUX_MCU_MASK  0x0f00U /* B4, B5 */
#define IOMUX_MCU_JTAG  0x0a00U
#define FORCE_JTAG      BIT(12) /* GRF_SOC_CON7 */

static int rk3399_sd_jtag_init(void)
{
	uint32_t mask = 0U;
	uint32_t val = 0U;

	if (DT_INST_PROP(0, ap_jtag)) {
		mask |= IOMUX_AP_MASK;
		val |= IOMUX_AP_JTAG;
	}
	if (DT_INST_PROP(0, mcu_jtag)) {
		mask |= IOMUX_MCU_MASK;
		val |= IOMUX_MCU_JTAG;
	}
	if (mask != 0U) {
		sys_write32((mask << 16) | val, DT_INST_REG_ADDR_BY_IDX(0, 0));
	}
	sys_write32((FORCE_JTAG << 16) | FORCE_JTAG, DT_INST_REG_ADDR_BY_IDX(0, 1));

	return 0;
}

SYS_INIT(rk3399_sd_jtag_init, PRE_KERNEL_1, 0);
