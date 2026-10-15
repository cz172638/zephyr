/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Marvell Armada 37xx group pin mux (north/south bridge selection register).
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

#define A37XX_SELECTION 0x30

#define NB_NODE DT_NODELABEL(pinctrl_nb)
#define SB_NODE DT_NODELABEL(pinctrl_sb)

static mm_reg_t a37xx_base(uint8_t bridge)
{
	static mm_reg_t base[2];

	if (base[0] == 0) {
		device_map(&base[A37XX_NB], DT_REG_ADDR(NB_NODE), DT_REG_SIZE(NB_NODE),
			   K_MEM_CACHE_NONE);
		device_map(&base[A37XX_SB], DT_REG_ADDR(SB_NODE), DT_REG_SIZE(SB_NODE),
			   K_MEM_CACHE_NONE);
	}
	return base[bridge & 1];
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0; i < pin_cnt; i++) {
		mm_reg_t sel = a37xx_base(pins[i].bridge) + A37XX_SELECTION;
		uint32_t v = sys_read32(sel);

		v = (v & ~pins[i].mask) | (pins[i].value & pins[i].mask);
		sys_write32(v, sel);
	}
	return 0;
}
