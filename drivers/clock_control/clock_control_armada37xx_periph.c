/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Marvell Armada 37xx clocks:
 *
 * - TBG (time base generator): four PLL outputs derived from the crystal,
 *   read-only (TBG-A-P, TBG-B-P, TBG-A-S, TBG-B-S).
 * - North / south bridge peripheral clocks: gates can be switched, muxes
 *   and dividers are read-only and keep the setup of the boot firmware;
 *   get_rate() walks mux -> divider(s) -> gate.
 *
 * The periph clock specifier is the index of the Linux armada-37xx-periph
 * table, so devicetree fragments can be shared with Linux. The topology
 * follows drivers/clk/mvebu/armada-37xx-{tbg,periph}.c.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

/* ---------------------------------------------------------------------- */
/* TBG                                                                     */

#define TBG_CTRL0    0x04
#define TBG_CTRL1    0x08
#define TBG_CTRL7    0x20
#define TBG_CTRL8    0x30
#define TBG_DIV_MASK 0x1ff
#define TBG_NUM      4

struct tbg_def {
	uint8_t refdiv_shift;
	uint8_t fbdiv_shift;
	uint8_t vcodiv_reg;
	uint8_t vcodiv_shift;
};

static const struct tbg_def tbg_defs[TBG_NUM] = {
	{0, 2, TBG_CTRL8, 1},    /* TBG-A-P */
	{16, 18, TBG_CTRL8, 17}, /* TBG-B-P */
	{0, 2, TBG_CTRL1, 0},    /* TBG-A-S */
	{16, 18, TBG_CTRL1, 16}, /* TBG-B-S */
};

struct a37xx_tbg_config {
	DEVICE_MMIO_ROM;
	uint32_t xtal_hz;
};

struct a37xx_tbg_data {
	DEVICE_MMIO_RAM;
};

static uint32_t a37xx_tbg_rate(const struct device *dev, unsigned int idx)
{
	const struct a37xx_tbg_config *cfg = dev->config;
	const struct tbg_def *t = &tbg_defs[idx];
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	uint64_t mult;
	uint32_t div;

	mult = ((sys_read32(base + TBG_CTRL0) >> t->fbdiv_shift) & TBG_DIV_MASK) << 2;
	div = (sys_read32(base + TBG_CTRL7) >> t->refdiv_shift) & TBG_DIV_MASK;
	if (div == 0) {
		div = 1;
	}
	div <<= (sys_read32(base + t->vcodiv_reg) >> t->vcodiv_shift) & TBG_DIV_MASK;

	return (uint32_t)(cfg->xtal_hz * mult / div);
}

static int a37xx_tbg_on(const struct device *dev, clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);

	return (uintptr_t)sys < TBG_NUM ? 0 : -EINVAL;
}

static int a37xx_tbg_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate)
{
	uintptr_t idx = (uintptr_t)sys;

	if (idx >= TBG_NUM) {
		return -EINVAL;
	}
	*rate = a37xx_tbg_rate(dev, idx);

	return 0;
}

static int a37xx_tbg_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	return 0;
}

static DEVICE_API(clock_control, a37xx_tbg_api) = {
	.on = a37xx_tbg_on,
	.off = a37xx_tbg_on,
	.get_rate = a37xx_tbg_get_rate,
};

#define A37XX_TBG_DEFINE(node)                                                                     \
	static const struct a37xx_tbg_config a37xx_tbg_cfg_##node = {                              \
		DEVICE_MMIO_ROM_INIT(node),                                                        \
		.xtal_hz = DT_PROP(DT_CLOCKS_CTLR(node), clock_frequency),                         \
	};                                                                                         \
	static struct a37xx_tbg_data a37xx_tbg_data_##node;                                        \
	DEVICE_DT_DEFINE(node, a37xx_tbg_init, NULL, &a37xx_tbg_data_##node,                       \
			 &a37xx_tbg_cfg_##node, PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,  \
			 &a37xx_tbg_api);

DT_FOREACH_STATUS_OKAY(marvell_armada_3700_tbg_clock, A37XX_TBG_DEFINE)

/* ---------------------------------------------------------------------- */
/* Peripheral clocks                                                       */

#define TBG_SEL  0x00
#define DIV_SEL0 0x04
#define DIV_SEL1 0x08
#define DIV_SEL2 0x0c
#define CLK_DIS  0x14 /* 1 = gated */

#define NO_GATE   0xff
#define NO_PARENT 0xff

enum a37xx_clk_kind {
	CLK_MUX_DD,    /* TBG mux, two 3-bit dividers, no gate */
	CLK_FULL_DD,   /* TBG mux, two 3-bit dividers, gate */
	CLK_FULL,      /* TBG mux, one 3-bit divider, gate */
	CLK_GATE,      /* gate on a parent of the same bridge or the crystal */
	CLK_GATE_DIV,  /* gate and a 1-bit divider on a parent */
	CLK_FIXED_TBG, /* gate and a 1-bit divider on a fixed TBG output */
	CLK_UNKNOWN,   /* rate not readable here (CPU clock under DVFS) */
};

struct a37xx_clk_desc {
	uint8_t kind;
	uint8_t gate;
	uint8_t mux;  /* TBG_SEL shift */
	uint8_t reg1; /* divider register */
	uint8_t shift1;
	uint8_t reg2;
	uint8_t shift2;
	uint8_t parent; /* index in this table, NO_PARENT = crystal, or TBG */
	uint8_t div0;   /* 1-bit divider: divisor for bit = 0 (bit = 1 doubles) */
};

#define MUX_DD(m, r1, s1, r2, s2)                                                                  \
	{                                                                                          \
		.kind = CLK_MUX_DD, .gate = NO_GATE, .mux = m, .reg1 = r1, .shift1 = s1,           \
		.reg2 = r2, .shift2 = s2                                                           \
	}
#define FULL_DD(g, m, r1, s1, r2, s2)                                                              \
	{                                                                                          \
		.kind = CLK_FULL_DD, .gate = g, .mux = m, .reg1 = r1, .shift1 = s1, .reg2 = r2,    \
		.shift2 = s2                                                                       \
	}
#define FULL(g, m, r, s)                                                                           \
	{                                                                                          \
		.kind = CLK_FULL, .gate = g, .mux = m, .reg1 = r, .shift1 = s                      \
	}
#define GATE(g, p)                                                                                 \
	{                                                                                          \
		.kind = CLK_GATE, .gate = g, .parent = p                                           \
	}
#define GATE_DIV(g, p, r, s, d0)                                                                   \
	{                                                                                          \
		.kind = CLK_GATE_DIV, .gate = g, .parent = p, .reg1 = r, .shift1 = s, .div0 = d0   \
	}
#define FIXED_TBG(g, t, r, s, d0)                                                                  \
	{                                                                                          \
		.kind = CLK_FIXED_TBG, .gate = g, .parent = t, .reg1 = r, .shift1 = s, .div0 = d0  \
	}

/* South bridge, Linux data_sb[] order */
static const struct a37xx_clk_desc sb_clks[] = {
	MUX_DD(6, DIV_SEL2, 6, DIV_SEL2, 9),         /* 0 gbe_50 */
	MUX_DD(8, DIV_SEL1, 18, DIV_SEL1, 21),       /* 1 gbe_core */
	MUX_DD(10, DIV_SEL1, 6, DIV_SEL1, 9),        /* 2 gbe_125 */
	GATE(0, 0),                                  /* 3 gbe1_50 */
	GATE(1, 0),                                  /* 4 gbe0_50 */
	GATE(2, 2),                                  /* 5 gbe1_125 */
	GATE(3, 2),                                  /* 6 gbe0_125 */
	/*
	 * gbe*_core and gbe_bm: Linux registers a 1-bit divider here with a
	 * zero-width field, i.e. never divides. On the ESPRESSObin DIV_SEL1
	 * bit 14 reads 1, yet the NETA rate limiter timed from this clock
	 * ran exactly 2x fast when the bit was applied, so the divider is not
	 * in the path (or the bit means something else): follow Linux.
	 */
	GATE(4, 1),                                  /* 7 gbe1_core */
	GATE(5, 1),                                  /* 8 gbe0_core */
	GATE(12, 1),                                 /* 9 gbe_bm */
	FULL_DD(11, 14, DIV_SEL0, 3, DIV_SEL0, 6),   /* 10 sdio */
	FULL_DD(16, 16, DIV_SEL0, 9, DIV_SEL0, 12),  /* 11 usb32_usb2_sys */
	FULL_DD(17, 18, DIV_SEL0, 15, DIV_SEL0, 18), /* 12 usb32_ss_sys */
	GATE(14, 1),                                 /* 13 pcie */
};

/* North bridge, Linux data_nb[] order */
static const struct a37xx_clk_desc nb_clks[] = {
	FULL_DD(2, 0, DIV_SEL2, 16, DIV_SEL2, 13),   /* 0 mmc */
	FULL_DD(3, 2, DIV_SEL2, 10, DIV_SEL2, 7),    /* 1 sata_host */
	FULL_DD(6, 4, DIV_SEL1, 3, DIV_SEL1, 0),     /* 2 sec_at */
	FULL_DD(7, 6, DIV_SEL1, 9, DIV_SEL1, 6),     /* 3 sec_dap */
	FULL_DD(8, 8, DIV_SEL1, 15, DIV_SEL1, 12),   /* 4 tscem */
	FULL(10, 10, DIV_SEL1, 18),                  /* 5 tscem_tmx */
	GATE(11, NO_PARENT),                         /* 6 avs */
	FULL_DD(12, 12, DIV_SEL1, 27, DIV_SEL1, 24), /* 7 sqf */
	FULL_DD(13, 14, DIV_SEL0, 3, DIV_SEL0, 0),   /* 8 pwm */
	GATE(16, NO_PARENT),                         /* 9 i2c_2 */
	GATE(17, NO_PARENT),                         /* 10 i2c_1 */
	FIXED_TBG(19, 2, DIV_SEL0, 18, 2),           /* 11 ddr_phy (TBG-A-S) */
	FULL_DD(21, 16, DIV_SEL0, 15, DIV_SEL0, 12), /* 12 ddr_fclk */
	FULL(22, 18, DIV_SEL0, 20),                  /* 13 trace */
	FULL(23, 20, DIV_SEL0, 23),                  /* 14 counter */
	FULL_DD(24, 24, DIV_SEL2, 22, DIV_SEL2, 19), /* 15 eip97 */
	{.kind = CLK_UNKNOWN, .gate = NO_GATE},      /* 16 cpu */
};

struct a37xx_clk_config {
	DEVICE_MMIO_ROM;
	const struct a37xx_clk_desc *clks;
	size_t nclks;
	const struct device *tbg;
	uint32_t xtal_hz;
};

struct a37xx_clk_data {
	DEVICE_MMIO_RAM;
	struct k_spinlock lock;
};

static int a37xx_clk_gate(const struct device *dev, clock_control_subsys_t sys, bool on)
{
	const struct a37xx_clk_config *cfg = dev->config;
	struct a37xx_clk_data *data = dev->data;
	uintptr_t idx = (uintptr_t)sys;
	k_spinlock_key_t key;
	uint32_t v;
	uint8_t bit;

	if (idx >= cfg->nclks) {
		return -EINVAL;
	}
	bit = cfg->clks[idx].gate;
	if (bit == NO_GATE) {
		return 0;
	}

	key = k_spin_lock(&data->lock);
	v = sys_read32(DEVICE_MMIO_GET(dev) + CLK_DIS);
	v = on ? (v & ~BIT(bit)) : (v | BIT(bit));
	sys_write32(v, DEVICE_MMIO_GET(dev) + CLK_DIS);
	k_spin_unlock(&data->lock, key);
	return 0;
}

static int a37xx_clk_on(const struct device *dev, clock_control_subsys_t sys)
{
	return a37xx_clk_gate(dev, sys, true);
}

static int a37xx_clk_off(const struct device *dev, clock_control_subsys_t sys)
{
	return a37xx_clk_gate(dev, sys, false);
}

static enum clock_control_status a37xx_clk_status(const struct device *dev,
						  clock_control_subsys_t sys)
{
	const struct a37xx_clk_config *cfg = dev->config;
	uintptr_t idx = (uintptr_t)sys;
	uint8_t bit;

	if (idx >= cfg->nclks) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}
	bit = cfg->clks[idx].gate;
	if (bit == NO_GATE) {
		return CLOCK_CONTROL_STATUS_ON;
	}
	return (sys_read32(DEVICE_MMIO_GET(dev) + CLK_DIS) & BIT(bit)) ? CLOCK_CONTROL_STATUS_OFF
								       : CLOCK_CONTROL_STATUS_ON;
}

/* 3-bit divider field, 1..6 valid */
static int a37xx_div3(mm_reg_t base, uint8_t reg, uint8_t shift, uint32_t *div)
{
	uint32_t v = (sys_read32(base + reg) >> shift) & 0x7;

	if (v == 0 || v > 6) {
		return -EIO;
	}
	*div = v;
	return 0;
}

static int a37xx_clk_rate(const struct device *dev, unsigned int idx, int depth, uint32_t *rate)
{
	const struct a37xx_clk_config *cfg = dev->config;
	const struct a37xx_clk_desc *c = &cfg->clks[idx];
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	uint32_t parent, d1 = 1, d2 = 1;
	unsigned int tbg;
	int ret;

	if (depth > 2) {
		return -ELOOP;
	}

	switch (c->kind) {
	case CLK_MUX_DD:
	case CLK_FULL_DD:
	case CLK_FULL:
		if (cfg->tbg == NULL) {
			return -ENOTSUP;
		}
		tbg = (sys_read32(base + TBG_SEL) >> c->mux) & 0x3;
		parent = a37xx_tbg_rate(cfg->tbg, tbg);
		ret = a37xx_div3(base, c->reg1, c->shift1, &d1);
		if (ret == 0 && c->kind != CLK_FULL) {
			ret = a37xx_div3(base, c->reg2, c->shift2, &d2);
		}
		if (ret < 0) {
			return ret;
		}
		*rate = DIV_ROUND_UP(parent, d1 * d2);
		return 0;

	case CLK_GATE:
		if (c->parent == NO_PARENT) {
			*rate = cfg->xtal_hz;
			return 0;
		}
		return a37xx_clk_rate(dev, c->parent, depth + 1, rate);

	case CLK_GATE_DIV:
		ret = a37xx_clk_rate(dev, c->parent, depth + 1, &parent);
		if (ret < 0) {
			return ret;
		}
		d1 = c->div0 << ((sys_read32(base + c->reg1) >> c->shift1) & 0x1);
		*rate = parent / d1;
		return 0;

	case CLK_FIXED_TBG:
		if (cfg->tbg == NULL) {
			return -ENOTSUP;
		}
		parent = a37xx_tbg_rate(cfg->tbg, c->parent);
		d1 = c->div0 << ((sys_read32(base + c->reg1) >> c->shift1) & 0x1);
		*rate = parent / d1;
		return 0;

	default:
		return -ENOTSUP;
	}
}

static int a37xx_clk_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate)
{
	const struct a37xx_clk_config *cfg = dev->config;
	uintptr_t idx = (uintptr_t)sys;

	if (idx >= cfg->nclks) {
		return -EINVAL;
	}

	return a37xx_clk_rate(dev, idx, 0, rate);
}

static int a37xx_clk_init(const struct device *dev)
{
	const struct a37xx_clk_config *cfg = dev->config;

	if (cfg->tbg != NULL && !device_is_ready(cfg->tbg)) {
		return -ENODEV;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	return 0;
}

static DEVICE_API(clock_control, a37xx_clk_api) = {
	.on = a37xx_clk_on,
	.off = a37xx_clk_off,
	.get_status = a37xx_clk_status,
	.get_rate = a37xx_clk_get_rate,
};

/* clocks = <&tbg 0>, <&tbg 1>, <&tbg 2>, <&tbg 3>, <&xtalclk> (as in Linux) */
#define A37XX_CLK_TBG(node)                                                                        \
	COND_CODE_1(DT_CLOCKS_HAS_IDX(node, 0), (DEVICE_DT_GET(DT_CLOCKS_CTLR_BY_IDX(node, 0))),  \
		    (NULL))
#define A37XX_CLK_XTAL(node)                                                                       \
	COND_CODE_1(DT_CLOCKS_HAS_IDX(node, 4),                                                    \
		    (DT_PROP(DT_CLOCKS_CTLR_BY_IDX(node, 4), clock_frequency)), (0))

#define A37XX_CLK_DEFINE(node, table)                                                              \
	static const struct a37xx_clk_config a37xx_clk_cfg_##node = {                              \
		DEVICE_MMIO_ROM_INIT(node),      .clks = table,                                    \
		.nclks = ARRAY_SIZE(table),      .tbg = A37XX_CLK_TBG(node),                       \
		.xtal_hz = A37XX_CLK_XTAL(node),                                                   \
	};                                                                                         \
	static struct a37xx_clk_data a37xx_clk_data_##node;                                        \
	DEVICE_DT_DEFINE(node, a37xx_clk_init, NULL, &a37xx_clk_data_##node,                       \
			 &a37xx_clk_cfg_##node, PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,  \
			 &a37xx_clk_api);

#define A37XX_SB_DEFINE(node) A37XX_CLK_DEFINE(node, sb_clks)
#define A37XX_NB_DEFINE(node) A37XX_CLK_DEFINE(node, nb_clks)

DT_FOREACH_STATUS_OKAY(marvell_armada_3700_periph_clock_sb, A37XX_SB_DEFINE)
DT_FOREACH_STATUS_OKAY(marvell_armada_3700_periph_clock_nb, A37XX_NB_DEFINE)
