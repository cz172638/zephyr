/*
 * Copyright (c) 2026 Jiri Kastner <cz172638@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip RK3308 CRU clock gate controller, with the rate of the SDMMC and
 * SPI clocks (mux and divider from DPLL, VPLL0, VPLL1 or the oscillator, the
 * PLLs as the boot loader left them).
 */

#define DT_DRV_COMPAT rockchip_rk3308_cru_clk

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/rk3308-cru.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(clock_control_rk3308_cru, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

#define RK3308_CRU_CLKGATE_CON(n) ((n) * 4U + 0x300U)
#define RK3308_CRU_CLKSEL_CON(n)  ((n) * 4U + 0x100U)
#define RK3308_CRU_PLL_CON0(n)    (0x20U * (n))
#define RK3308_CRU_PLL_CON1(n)    (0x20U * (n) + 0x04U)
#define RK3308_CRU_PLL_CON2(n)    (0x20U * (n) + 0x08U)
#define RK3308_CRU_MODE           0x00a0U

#define PLL_CON0_BYPASS   BIT(15)
#define PLL_CON0_POSTDIV1 GENMASK(14, 12)
#define PLL_CON0_FBDIV    GENMASK(11, 0)
#define PLL_CON1_DSMPD    BIT(12)
#define PLL_CON1_POSTDIV2 GENMASK(8, 6)
#define PLL_CON1_REFDIV   GENMASK(5, 0)
#define PLL_CON2_FRACDIV  GENMASK(23, 0)
#define CRU_MODE_NORMAL   1U

#define OSC_HZ 24000000ULL

/* clock parents: PLL index as used by RK3308_CRU_PLL_CONx(), or the oscillator */
enum rk3308_src {
	SRC_DPLL = 1,
	SRC_VPLL0 = 2,
	SRC_VPLL1 = 3,
	SRC_XIN24M,
};

/* a mux + divider in one CLKSEL register; mux value = index in src[] */
struct rk3308_comp {
	uint16_t id;
	uint8_t con;
	uint8_t mux_shift;
	uint8_t div_bits;
	uint8_t nsrc;
	uint8_t src[4];
};

/* Linux clk-rk3308.c: clk_sdmmc_div, clk_spi0..2 */
static const struct rk3308_comp rk3308_comps[] = {
	{SCLK_SDMMC, 39, 8, 8, 4, {SRC_DPLL, SRC_VPLL0, SRC_VPLL1, SRC_XIN24M}},
	{SCLK_SPI0, 30, 14, 7, 3, {SRC_DPLL, SRC_VPLL0, SRC_XIN24M}},
	{SCLK_SPI1, 31, 14, 7, 3, {SRC_DPLL, SRC_VPLL0, SRC_XIN24M}},
	{SCLK_SPI2, 32, 14, 7, 3, {SRC_DPLL, SRC_VPLL0, SRC_XIN24M}},
};

/* clk_sdmmc: CLKSEL_CON39 bit 15 selects clk_sdmmc_div (0) or _div50 */
#define SDMMC_SEL_DIV50 BIT(15)
#define SDMMC_DIV_GATE  RK3308_CLK_GATE(8, 6)

struct clock_control_rk3308_config {
	DEVICE_MMIO_ROM;
};

struct clock_control_rk3308_data {
	DEVICE_MMIO_RAM;
};

/*
 * Each clock id encodes CRU_CLKGATE_CON index in [15:8] and the gate bit in
 * [7:0]. Rockchip gate registers are hiword-masked: write the mask in the top
 * 16 bits. A set gate bit disables the clock, so "on" clears it.
 */
static void rk3308_cru_gate_write(mm_reg_t base, uint16_t clk_id, bool enable)
{
	uint32_t con = (clk_id >> 8) & 0xffU;
	uint32_t bit = clk_id & 0xffU;
	uint32_t off = RK3308_CRU_CLKGATE_CON(con);
	uint32_t val;

	if (enable) {
		val = BIT(bit) << 16;
	} else {
		val = (BIT(bit) << 16) | BIT(bit);
	}

	sys_write32(val, base + off);
}

static int clock_control_rk3308_on(const struct device *dev, clock_control_subsys_t sys)
{
	if ((uint16_t)(uintptr_t)sys == SCLK_SDMMC) {
		rk3308_cru_gate_write(DEVICE_MMIO_GET(dev), SDMMC_DIV_GATE, true);
	}
	rk3308_cru_gate_write(DEVICE_MMIO_GET(dev), (uint16_t)(uintptr_t)sys, true);

	return 0;
}

static int clock_control_rk3308_off(const struct device *dev, clock_control_subsys_t sys)
{
	rk3308_cru_gate_write(DEVICE_MMIO_GET(dev), (uint16_t)(uintptr_t)sys, false);

	return 0;
}

static uint64_t rk3308_src_rate(mm_reg_t base, enum rk3308_src src)
{
	uint32_t mode, con0, con1, con2;
	uint64_t fbdiv, div;

	if (src == SRC_XIN24M) {
		return OSC_HZ;
	}
	mode = sys_read32(base + RK3308_CRU_MODE);
	con0 = sys_read32(base + RK3308_CRU_PLL_CON0(src));
	con1 = sys_read32(base + RK3308_CRU_PLL_CON1(src));
	con2 = sys_read32(base + RK3308_CRU_PLL_CON2(src));
	if ((((mode >> (2U * src)) & 0x3U) != CRU_MODE_NORMAL) ||
	    ((con0 & PLL_CON0_BYPASS) != 0U)) {
		return OSC_HZ;
	}
	/* feedback divider in 24-bit fixed point */
	fbdiv = (uint64_t)FIELD_GET(PLL_CON0_FBDIV, con0) << 24;
	if ((con1 & PLL_CON1_DSMPD) == 0U) {
		fbdiv += FIELD_GET(PLL_CON2_FRACDIV, con2);
	}
	div = (uint64_t)FIELD_GET(PLL_CON1_REFDIV, con1) * FIELD_GET(PLL_CON0_POSTDIV1, con0) *
	      FIELD_GET(PLL_CON1_POSTDIV2, con1);
	if (div == 0U) {
		return 0;
	}
	return ((OSC_HZ * fbdiv) / div) >> 24;
}

static const struct rk3308_comp *rk3308_comp_find(uint16_t id)
{
	for (size_t i = 0; i < ARRAY_SIZE(rk3308_comps); i++) {
		if (rk3308_comps[i].id == id) {
			return &rk3308_comps[i];
		}
	}
	return NULL;
}

static uint32_t rk3308_comp_rate(mm_reg_t base, const struct rk3308_comp *c)
{
	uint32_t v = sys_read32(base + RK3308_CRU_CLKSEL_CON(c->con));
	uint32_t mux = (v >> c->mux_shift) & 0x3U;
	uint32_t div = (v & (BIT(c->div_bits) - 1U)) + 1U;

	if (mux >= c->nsrc) {
		return 0;
	}
	return (uint32_t)(rk3308_src_rate(base, c->src[mux]) / div);
}

/* highest rate not above the request, over all parents and dividers */
static int rk3308_comp_set_rate(mm_reg_t base, const struct rk3308_comp *c, uint32_t want)
{
	uint32_t max_div = BIT(c->div_bits);
	uint64_t best = 0;
	uint32_t best_mux = 0, best_div = 0, mask;

	for (uint32_t mux = 0; mux < c->nsrc; mux++) {
		uint64_t parent = rk3308_src_rate(base, c->src[mux]);
		uint64_t div = DIV_ROUND_UP(parent, want);
		uint64_t rate;

		if (parent == 0U || div > max_div) {
			continue;
		}
		div = MAX(div, 1U);
		rate = parent / div;
		if (rate > best) {
			best = rate;
			best_mux = mux;
			best_div = (uint32_t)div;
		}
	}
	if (best_div == 0U) {
		return -ERANGE;
	}
	mask = (0x3U << c->mux_shift) | (max_div - 1U);
	if (c->id == SCLK_SDMMC) {
		/* clk_sdmmc from clk_sdmmc_div */
		mask |= SDMMC_SEL_DIV50;
	}
	sys_write32((mask << 16) | (best_mux << c->mux_shift) | (best_div - 1U),
		    base + RK3308_CRU_CLKSEL_CON(c->con));
	LOG_DBG("clk %04x: %llu Hz (asked %u)", c->id, best, want);
	return 0;
}

static int clock_control_rk3308_set_rate(const struct device *dev, clock_control_subsys_t sys,
					 clock_control_subsys_rate_t rate)
{
	const struct rk3308_comp *c = rk3308_comp_find((uint16_t)(uintptr_t)sys);
	uint32_t want = (uint32_t)(uintptr_t)rate;

	if (c == NULL) {
		return -ENOTSUP;
	}
	if (want == 0U) {
		return -EINVAL;
	}
	return rk3308_comp_set_rate(DEVICE_MMIO_GET(dev), c, want);
}

static int clock_control_rk3308_get_rate(const struct device *dev, clock_control_subsys_t sys,
					 uint32_t *rate)
{
	uint16_t clk_id = (uint16_t)(uintptr_t)sys;
	const struct rk3308_comp *c = rk3308_comp_find(clk_id);

	if (c != NULL) {
		*rate = rk3308_comp_rate(DEVICE_MMIO_GET(dev), c);
		return *rate != 0U ? 0 : -EIO;
	}

	switch (clk_id) {
	case SCLK_UART0:
	case SCLK_UART1:
	case SCLK_UART2:
	case SCLK_UART3:
	case SCLK_UART4:
		/* Left on the 24 MHz oscillator by the boot loader. */
		*rate = 24000000U;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int clock_control_rk3308_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	return 0;
}

static DEVICE_API(clock_control, clock_control_rk3308_api) = {
	.on = clock_control_rk3308_on,
	.off = clock_control_rk3308_off,
	.get_rate = clock_control_rk3308_get_rate,
	.set_rate = clock_control_rk3308_set_rate,
};

static struct clock_control_rk3308_data rk3308_cru_data;

static const struct clock_control_rk3308_config clock_control_rk3308_config = {
	DEVICE_MMIO_ROM_INIT(DT_INST_PARENT(0)),
};

DEVICE_DT_INST_DEFINE(0, clock_control_rk3308_init, NULL, &rk3308_cru_data,
		      &clock_control_rk3308_config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_rk3308_api);
