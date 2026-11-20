/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip RK3399 CRU/PMUCRU clock gate controller, plus rates of the few
 * composite clocks drivers need (SPI, SDMMC, the PMU I2Cs). PLLs are left as
 * the boot loader set them (U-Boot SPL: GPLL 594 MHz, CPLL 384 MHz, LPLL and
 * BPLL 600 MHz; TF-A: PPLL 676 MHz), except the cluster PLLs, which set_rate
 * on ARMCLKL/ARMCLKB reprograms. The caller owns the core supply: the rate
 * must be valid at the voltage the cluster runs at.
 */

#define DT_DRV_COMPAT rockchip_rk3399_cru_clk

#include <errno.h>
#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
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

	if (id >= RK3399_CLK_NOGATE(0)) {
		return 0;
	}
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

/* PMUCRU: the I2Cs in the PMU domain, PPLL divided by CLKSEL_CON[sel][shift+6:shift] + 1 */
#define PMUCRU_PPLL      0x00U
#define PMUCRU_CLKSEL(n) (0x80U + (n) * 4U)

static const struct {
	uint16_t id;
	uint8_t sel;
	uint8_t shift;
} pmu_i2c[] = {
	{SCLK_I2C0_PMU, 2, 0},
	{SCLK_I2C8_PMU, 2, 8},
	{SCLK_I2C4_PMU, 3, 0},
};

static int pmu_i2c_find(const struct device *dev, uint32_t id)
{
	const struct clock_control_rk3399_config *cfg = dev->config;

	if (cfg->gate_offset != 0x100U) {
		return -1;
	}
	for (size_t i = 0; i < ARRAY_SIZE(pmu_i2c); i++) {
		if (pmu_i2c[i].id == id) {
			return (int)i;
		}
	}
	return -1;
}

/*
 * Cluster core clocks: clk_core_l/b = PLL selected by CLKSEL_CON0/2[7:6]
 * divided by [4:0] + 1. aclkm (CLKSEL_CON0/2[12:8]), atclk and pclk_dbg
 * (CLKSEL_CON1/3[4:0] and [12:8]) divide the core clock; the values are the
 * ones of the Linux clk-rk3399 cpuclk tables. All PLL outputs from 24 MHz
 * with REFDIV 1 in integer mode.
 */
#define CRU_LPLL           0x00U
#define CRU_BPLL           0x20U
#define CRU_DPLL           0x40U
#define PLL_CON3_MODE      (0x3U << 8)
#define PLL_CON3_MODE_NORM (0x1U << 8)
#define PLL_CON3_DSMPD     BIT(3)
#define PLL_CON2_LOCK      BIT(31)
#define PLL_LOCK_US        1000U
#define HIWORD(mask, val)  (((mask) << 16) | (val))

static const struct rk_armclk_rate {
	uint16_t mhz;
	uint8_t fbdiv;
	uint8_t postdiv1;
	uint8_t postdiv2;
	uint8_t atclk; /* atclk and pclk_dbg divider - 1 */
} rk_armclk_rates[] = {
	{96, 64, 4, 4, 1},    {216, 72, 4, 2, 1},   {312, 52, 2, 2, 1},   {408, 68, 2, 2, 2},
	{600, 75, 3, 1, 3},   {696, 58, 2, 1, 3},   {816, 68, 2, 1, 4},   {1008, 84, 2, 1, 5},
	{1200, 50, 1, 1, 5},  {1416, 59, 1, 1, 6},  {1488, 62, 1, 1, 6},  {1512, 63, 1, 1, 7},
	{1608, 67, 1, 1, 7},  {1704, 71, 1, 1, 8},  {1800, 75, 1, 1, 8},
};

static struct k_spinlock armclk_lock;

static bool rk_is_armclk(const struct device *dev, uint32_t id)
{
	const struct clock_control_rk3399_config *cfg = dev->config;

	return cfg->gate_offset == 0x300U && (id == ARMCLKL || id == ARMCLKB);
}

/* ARMCLKL: PLL LPLL, CLKSEL_CON0/1, mux 0; ARMCLKB: BPLL, CLKSEL_CON2/3, mux 1 */
static uint32_t rk_armclk_get(const struct device *dev, uint32_t id)
{
	static const uint32_t mux_pll[] = {CRU_LPLL, CRU_BPLL, CRU_DPLL, CRU_GPLL};
	uint32_t v = sys_read32(DEVICE_MMIO_GET(dev) + CRU_CLKSEL(id == ARMCLKL ? 0U : 2U));

	return rk_pll_rate(dev, mux_pll[(v >> 6) & 0x3U]) / ((v & 0x1fU) + 1U);
}

static void rk_armclk_divs(mm_reg_t sel, const struct rk_armclk_rate *r, uint32_t mux)
{
	sys_write32(HIWORD((0x1fU << 8) | (0x3U << 6) | 0x1fU, (1U << 8) | (mux << 6)), sel);
	sys_write32(HIWORD((0x1fU << 8) | 0x1fU, ((uint32_t)r->atclk << 8) | r->atclk), sel + 4U);
}

/*
 * Slow mode runs the cluster from the 24 MHz oscillator while the PLL
 * relocks (some 10 us). Dividers go first when the rate rises and last when
 * it falls, so aclkm, atclk and pclk_dbg never run above their table rate.
 */
static int rk_armclk_set(const struct device *dev, uint32_t id, uint32_t want)
{
	mm_reg_t pll = DEVICE_MMIO_GET(dev) + (id == ARMCLKL ? CRU_LPLL : CRU_BPLL);
	mm_reg_t sel = DEVICE_MMIO_GET(dev) + CRU_CLKSEL(id == ARMCLKL ? 0U : 2U);
	uint32_t mux = id == ARMCLKL ? 0U : 1U;
	const struct rk_armclk_rate *r = NULL;
	k_spinlock_key_t key;
	bool up;
	int ret = 0;

	for (size_t i = 0; i < ARRAY_SIZE(rk_armclk_rates); i++) {
		if ((uint32_t)rk_armclk_rates[i].mhz * 1000000U <= want) {
			r = &rk_armclk_rates[i];
		}
	}
	if (r == NULL) {
		return -ERANGE;
	}

	key = k_spin_lock(&armclk_lock);
	up = (uint32_t)r->mhz * 1000000U > rk_armclk_get(dev, id);
	if (up) {
		rk_armclk_divs(sel, r, mux);
	}
	sys_write32(HIWORD(PLL_CON3_MODE, 0U), pll + 12U);
	sys_write32(HIWORD(PLL_CON3_DSMPD, PLL_CON3_DSMPD), pll + 12U);
	sys_write32(HIWORD(0xfffU, r->fbdiv), pll);
	sys_write32(HIWORD((0x7U << 12) | (0x7U << 8) | 0x3fU,
			   ((uint32_t)r->postdiv2 << 12) | ((uint32_t)r->postdiv1 << 8) | 1U),
		    pll + 4U);
	for (uint32_t us = 0; (sys_read32(pll + 8U) & PLL_CON2_LOCK) == 0U; us++) {
		if (us == PLL_LOCK_US) {
			ret = -ETIMEDOUT; /* left in slow mode, at 24 MHz */
			break;
		}
		k_busy_wait(1);
	}
	if (ret == 0) {
		sys_write32(HIWORD(PLL_CON3_MODE, PLL_CON3_MODE_NORM), pll + 12U);
		if (!up) {
			rk_armclk_divs(sel, r, mux);
		}
	}
	k_spin_unlock(&armclk_lock, key);
	return ret;
}

static int clock_control_rk3399_get_rate(const struct device *dev, clock_control_subsys_t sys,
					 uint32_t *rate)
{
	const struct clock_control_rk3399_config *cfg = dev->config;
	const struct rk_comp *c = rk_comp_find(dev, (uint32_t)(uintptr_t)sys);
	int p = pmu_i2c_find(dev, (uint32_t)(uintptr_t)sys);
	uint32_t v;

	if (rk_is_armclk(dev, (uint32_t)(uintptr_t)sys)) {
		*rate = rk_armclk_get(dev, (uint32_t)(uintptr_t)sys);
		return 0;
	}
	if ((uint32_t)(uintptr_t)sys == PCLK_WDT && cfg->gate_offset == 0x300U) {
		/* pclk_alive: GPLL / (CLKSEL_CON57[4:0] + 1) */
		v = sys_read32(DEVICE_MMIO_GET(dev) + CRU_CLKSEL(57));
		*rate = rk_src_rate(dev, SRC_GPLL) / ((v & 0x1fU) + 1U);
		return 0;
	}
	if (p >= 0) {
		v = sys_read32(DEVICE_MMIO_GET(dev) + PMUCRU_CLKSEL(pmu_i2c[p].sel));
		*rate = rk_pll_rate(dev, PMUCRU_PPLL) / (((v >> pmu_i2c[p].shift) & 0x7fU) + 1U);
		return 0;
	}
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
	int p = pmu_i2c_find(dev, (uint32_t)(uintptr_t)sys);

	if (want == 0U) {
		return -EINVAL;
	}
	if (rk_is_armclk(dev, (uint32_t)(uintptr_t)sys)) {
		return rk_armclk_set(dev, (uint32_t)(uintptr_t)sys, want);
	}
	if (p >= 0) {
		src = rk_pll_rate(dev, PMUCRU_PPLL);
		div = DIV_ROUND_UP(src, want);
		if (src == 0U || div > 128U) {
			return -ERANGE;
		}
		sys_write32((0x7fU << (16U + pmu_i2c[p].shift)) | ((div - 1U) << pmu_i2c[p].shift),
			    DEVICE_MMIO_GET(dev) + PMUCRU_CLKSEL(pmu_i2c[p].sel));
		return 0;
	}
	if (c == NULL) {
		return -ENOTSUP;
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
