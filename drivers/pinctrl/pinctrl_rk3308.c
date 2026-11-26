/*
 * Copyright (c) 2026 Jiri Kastner <cz172638@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip RK3308 pin controller (GRF iomux and pull fields).
 */

#define DT_DRV_COMPAT rockchip_rk3308_pinctrl

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/init.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

/* GRF_GPIO<bank><port>_P: 2 bits per pin, 8 pins per register */
#define GRF_GPIO_P(bank, pin) (0x00a0U + (bank) * 0x10U + ((pin) / 8U) * 4U)

/* GRF register block, mapped once before any consumer applies its pins. */
static mm_reg_t grf_base;

static int pinctrl_rk3308_init(void)
{
	device_map(&grf_base, DT_REG_ADDR(DT_INST_PARENT(0)), DT_REG_SIZE(DT_INST_PARENT(0)),
		   K_MEM_CACHE_NONE);

	return 0;
}

SYS_INIT(pinctrl_rk3308_init, PRE_KERNEL_1, 0);

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (; pin_cnt > 0U; pin_cnt--, pins++) {
		uint32_t cfg = pins->cfg;
		uint32_t off = cfg & 0xfffU;
		uint32_t shift = (cfg >> 12) & 0xfU;
		uint32_t width = (cfg >> 16) & 0x7U;
		uint32_t mux = (cfg >> 19) & 0xfU;
		uint32_t bank = (cfg >> 23) & 0x7U;
		uint32_t pin = (cfg >> 26) & 0x1fU;
		uint32_t mask = BIT(width) - 1U;

		/* hiword-masked write: enable only the field being changed */
		sys_write32((mask << (shift + 16U)) | ((mux & mask) << shift), grf_base + off);

		if (pins->pull != RK3308_PULL_KEEP) {
			shift = (pin % 8U) * 2U;
			sys_write32((0x3U << (shift + 16U)) | ((uint32_t)pins->pull << shift),
				    grf_base + GRF_GPIO_P(bank, pin));
		}
	}

	return 0;
}
