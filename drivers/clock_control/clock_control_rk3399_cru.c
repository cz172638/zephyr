/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip RK3399 CRU/PMUCRU clock gate controller, plus rates of the few
 * composite clocks drivers need (SPI, SDMMC). PLLs are left as the boot
 * loader set them (U-Boot SPL: GPLL 594 MHz, CPLL 384 MHz).
 */

#define DT_DRV_COMPAT rockchip_rk3399_cru_clk

#include <errno.h>
#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/rk3399-cru.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

/* Highest CLKGATE_CON index in the CRU */
#define RK3399_CRU_CLKGATE_NUM 35U

struct clock_control_rk3399_config {
	DEVICE_MMIO_ROM;
	uint32_t gate_offset;
};

struct clock_control_rk3399_data {
	DEVICE_MMIO_RAM;
};

static int rk3399_cru_gate(const struct device *dev, clock_control_subsys_t sys, bool enable)
{
	const struct clock_control_rk3399_config *cfg = dev->config;
	uint32_t id = (uint32_t)(uintptr_t)sys;
	uint32_t con = id / 16U;
	uint32_t bit = id % 16U;

	if (con >= RK3399_CRU_CLKGATE_NUM) {
		return -EINVAL;
	}

	/* Gate bit set = clock off; the upper half-word is the write mask. */
	sys_write32((BIT(bit) << 16) | (enable ? 0U : BIT(bit)),
		    DEVICE_MMIO_GET(dev) + cfg->gate_offset + (con * 4U));

	return 0;
}

static int clock_control_rk3399_on(const struct device *dev, clock_control_subsys_t sys)
{
	return rk3399_cru_gate(dev, sys, true);
}

static int clock_control_rk3399_off(const struct device *dev, clock_control_subsys_t sys)
{
	return rk3399_cru_gate(dev, sys, false);
}

static enum clock_control_status clock_control_rk3399_get_status(const struct device *dev,
								 clock_control_subsys_t sys)
{
	const struct clock_control_rk3399_config *cfg = dev->config;
	uint32_t id = (uint32_t)(uintptr_t)sys;
	uint32_t val;

	if ((id / 16U) >= RK3399_CRU_CLKGATE_NUM) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	val = sys_read32(DEVICE_MMIO_GET(dev) + cfg->gate_offset + ((id / 16U) * 4U));

	return ((val & BIT(id % 16U)) != 0U) ? CLOCK_CONTROL_STATUS_OFF : CLOCK_CONTROL_STATUS_ON;
}

#define OSC_HZ        24000000U
#define CRU_CLKSEL(n) (0x100U + (n) * 4U)
#define CRU_CPLL      0x60U
#define CRU_GPLL      0x80U
#define CRU_NPLL      0xa0U

enum rk_src {
	SRC_CPLL,
	SRC_GPLL,
	SRC_NPLL,
	SRC_NONE, /* PPLL: PMUCRU, not handled here */
	SRC_UPLL,
	SRC_24M,
};

enum rk_parents {
	PAR_CPLL_GPLL,       /* SPI, ACLK_EMMC */
	PAR_SDMMC,           /* cpll gpll npll ppll upll 24m */
	PAR_EMMC,            /* cpll gpll npll upll 24m */
};

static const uint8_t rk_parent_src[][6] = {
	[PAR_CPLL_GPLL] = {SRC_CPLL, SRC_GPLL},
	[PAR_SDMMC] = {SRC_CPLL, SRC_GPLL, SRC_NPLL, SRC_NONE, SRC_UPLL, SRC_24M},
	[PAR_EMMC] = {SRC_CPLL, SRC_GPLL, SRC_NPLL, SRC_UPLL, SRC_24M},
};

struct rk_comp {
	uint16_t id;
	uint8_t sel;
	uint8_t mux_shift;
	uint8_t mux_width;
	uint8_t div_shift;
	uint8_t div_width;
	uint8_t parents;     /* enum rk_parents */
	uint8_t gpll_idx;    /* mux value of GPLL */
	uint8_t osc_idx;     /* mux value of 24 MHz, 0xff: none */
};

static const struct rk_comp rk_comps[] = {
	{SCLK_SPI0, 59, 7, 1, 0, 7, PAR_CPLL_GPLL, 1, 0xff},
	{SCLK_SPI1, 59, 15, 1, 8, 7, PAR_CPLL_GPLL, 1, 0xff},
	{SCLK_SPI2, 60, 7, 1, 0, 7, PAR_CPLL_GPLL, 1, 0xff},
	{SCLK_SPI4, 60, 15, 1, 8, 7, PAR_CPLL_GPLL, 1, 0xff},
	{SCLK_SPI5, 58, 15, 1, 8, 7, PAR_CPLL_GPLL, 1, 0xff},
	{SCLK_SDMMC, 16, 8, 3, 0, 7, PAR_SDMMC, 1, 5},
	{SCLK_EMMC, 22, 8, 3, 0, 7, PAR_EMMC, 1, 4},
	{ACLK_EMMC, 21, 7, 1, 0, 5, PAR_CPLL_GPLL, 1, 0xff},
};

static const struct rk_comp *rk_comp_find(const struct device *dev, uint32_t id)
{
	const struct clock_control_rk3399_config *cfg = dev->config;

	if (cfg->gate_offset != 0x300U) {
		return NULL; /* PMUCRU */
	}
	for (size_t i = 0; i < ARRAY_SIZE(rk_comps); i++) {
		if (rk_comps[i].id == id) {
			return &rk_comps[i];
		}
	}
	return NULL;
}

static uint32_t rk_pll_rate(const struct device *dev, uint32_t off)
{
	mm_reg_t b = DEVICE_MMIO_GET(dev) + off;
	uint32_t c0 = sys_read32(b), c1 = sys_read32(b + 4), c2 = sys_read32(b + 8);
	uint32_t c3 = sys_read32(b + 12);
	uint64_t fb = c0 & 0xfffU, ref = c1 & 0x3fU;
	uint64_t pd = ((c1 >> 8) & 0x7U) * ((c1 >> 12) & 0x7U);
	uint64_t hz;

	if (((c3 >> 8) & 0x3U) != 1U) {
		return OSC_HZ; /* slow mode */
	}
	if (ref == 0 || pd == 0) {
		return 0;
	}
	hz = (uint64_t)OSC_HZ * fb;
	if ((c3 & BIT(3)) == 0U) {
		hz += ((uint64_t)OSC_HZ * (c2 & 0xffffffU)) >> 24;
	}
	return (uint32_t)(hz / ref / pd);
}

static uint32_t rk_src_rate(const struct device *dev, uint32_t s)
{
	switch (s) {
	case SRC_CPLL:
		return rk_pll_rate(dev, CRU_CPLL);
	case SRC_GPLL:
		return rk_pll_rate(dev, CRU_GPLL);
	case SRC_NPLL:
		return rk_pll_rate(dev, CRU_NPLL);
	case SRC_UPLL:
		return 480000000U;
	case SRC_24M:
		return OSC_HZ;
	default:
		return 0;
	}
}

static int clock_control_rk3399_get_rate(const struct device *dev, clock_control_subsys_t sys,
					 uint32_t *rate)
{
	const struct rk_comp *c = rk_comp_find(dev, (uint32_t)(uintptr_t)sys);
	uint32_t v;

	if (c == NULL) {
		return -ENOTSUP;
	}
	v = sys_read32(DEVICE_MMIO_GET(dev) + CRU_CLKSEL(c->sel));
	*rate = rk_src_rate(dev, rk_parent_src[c->parents][(v >> c->mux_shift) &
							      BIT_MASK(c->mux_width)]) /
		(((v >> c->div_shift) & BIT_MASK(c->div_width)) + 1U);
	return 0;
}

/* highest rate not above the request: GPLL, or 24 MHz when GPLL can't divide that low */
static int clock_control_rk3399_set_rate(const struct device *dev, clock_control_subsys_t sys,
					 clock_control_subsys_rate_t rate)
{
	const struct rk_comp *c = rk_comp_find(dev, (uint32_t)(uintptr_t)sys);
	uint32_t want = (uint32_t)(uintptr_t)rate;
	uint32_t src, div, mask, max, mux;

	if (c == NULL) {
		return -ENOTSUP;
	}
	if (want == 0U) {
		return -EINVAL;
	}
	max = BIT(c->div_width);
	mux = c->gpll_idx;
	src = rk_src_rate(dev, SRC_GPLL);
	div = DIV_ROUND_UP(src, want);
	if (div > max && c->osc_idx != 0xffU) {
		mux = c->osc_idx;
		src = OSC_HZ;
		div = DIV_ROUND_UP(src, want);
	}
	if (div > max || src == 0U) {
		return -ERANGE;
	}
	mask = (BIT_MASK(c->mux_width) << c->mux_shift) | (BIT_MASK(c->div_width) << c->div_shift);
	sys_write32((mask << 16) | (mux << c->mux_shift) | ((div - 1U) << c->div_shift),
		    DEVICE_MMIO_GET(dev) + CRU_CLKSEL(c->sel));
	return 0;
}

static int clock_control_rk3399_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	return 0;
}

static DEVICE_API(clock_control, clock_control_rk3399_api) = {
	.on = clock_control_rk3399_on,
	.off = clock_control_rk3399_off,
	.get_status = clock_control_rk3399_get_status,
	.get_rate = clock_control_rk3399_get_rate,
	.set_rate = clock_control_rk3399_set_rate,
};

#define RK3399_CRU_CLK_INIT(n)                                                                     \
	static struct clock_control_rk3399_data rk3399_cru_clk_data_##n;                          \
	static const struct clock_control_rk3399_config rk3399_cru_clk_config_##n = {             \
		DEVICE_MMIO_ROM_INIT(DT_INST_PARENT(n)),                                           \
		.gate_offset = DT_INST_PROP(n, rockchip_gate_offset),                              \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, clock_control_rk3399_init, NULL, &rk3399_cru_clk_data_##n,      \
			      &rk3399_cru_clk_config_##n, PRE_KERNEL_1,                            \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_rk3399_api);

DT_INST_FOREACH_STATUS_OKAY(RK3399_CRU_CLK_INIT)
