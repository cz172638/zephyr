/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip RK3399 iomux and pulls. Banks 0-1 live in the PMUGRF, banks 2-4
 * in the GRF; 2 bits per pin, 8 pins per register, upper half-word is the
 * write enable.
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

#define PMUGRF_IOMUX 0x0000U
#define PMUGRF_PULL  0x0040U
#define GRF_IOMUX    0xe000U
#define GRF_PULL     0xe040U

#define GRF_NODE    DT_NODELABEL(grf)
#define PMUGRF_NODE DT_NODELABEL(pmugrf)

static mm_reg_t bank_base(unsigned int bank, uint32_t *iomux, uint32_t *pull)
{
	static mm_reg_t grf, pmugrf;

	if (grf == 0) {
		device_map(&grf, DT_REG_ADDR(GRF_NODE), DT_REG_SIZE(GRF_NODE), K_MEM_CACHE_NONE);
		device_map(&pmugrf, DT_REG_ADDR(PMUGRF_NODE), DT_REG_SIZE(PMUGRF_NODE),
			   K_MEM_CACHE_NONE);
	}
	if (bank < 2U) {
		*iomux = PMUGRF_IOMUX + bank * 0x10U;
		*pull = PMUGRF_PULL + bank * 0x10U;
		return pmugrf;
	}
	*iomux = GRF_IOMUX + (bank - 2U) * 0x10U;
	*pull = GRF_PULL + (bank - 2U) * 0x10U;
	return grf;
}

/*
 * Pull register value. Bank 0 ports A/B and bank 2 ports C/D are the
 * 1.8 V-only type (0 none, 1 down, 3 up); the rest 0 none, 1 up, 2 down.
 */
static uint32_t pull_bits(unsigned int bank, unsigned int pin, unsigned int pull)
{
	bool v18 = (bank == 0U && pin < 16U) || (bank == 2U && pin >= 16U);

	switch (pull) {
	case RK_PULL_UP:
		return v18 ? 3U : 1U;
	case RK_PULL_DOWN:
		return v18 ? 1U : 2U;
	default:
		return 0U;
	}
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0; i < pin_cnt; i++) {
		unsigned int bank = (pins[i] >> 24) & 0x7U;
		unsigned int pin = (pins[i] >> 16) & 0x1fU;
		unsigned int func = (pins[i] >> 8) & 0xfU;
		unsigned int pull = pins[i] & 0xfU;
		unsigned int sh = (pin % 8U) * 2U;
		uint32_t iomux, pullr;
		mm_reg_t base;

		if (bank > 4U || func > 3U) {
			return -EINVAL;
		}
		base = bank_base(bank, &iomux, &pullr);
		sys_write32((3U << (sh + 16U)) | (func << sh), base + iomux + (pin / 8U) * 4U);
		if (pull != RK_PULL_KEEP) {
			sys_write32((3U << (sh + 16U)) | (pull_bits(bank, pin, pull) << sh),
				    base + pullr + (pin / 8U) * 4U);
		}
	}
	return 0;
}
