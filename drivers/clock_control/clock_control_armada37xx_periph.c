/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Marvell Armada 37xx peripheral clock gates (north / south bridge). Only
 * gating is implemented; muxes and dividers keep their bootloader setup.
 * The clock specifier is the index of the Linux armada-37xx-periph table,
 * so devicetree fragments can be shared.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

#define CLK_DIS 0x14 /* 1 = gated */
#define NO_GATE 0xff

/* index -> CLK_DIS bit, south bridge */
static const uint8_t sb_gate_bit[] = {
	NO_GATE, NO_GATE, NO_GATE,     /* gbe_50, gbe_core, gbe_125 (muxes) */
	0,       1,       2,       3,  /* gbe1_50, gbe0_50, gbe1_125, gbe0_125 */
	4,       5,       12,          /* gbe1_core, gbe0_core, gbe_bm */
	11,      16,      17,      14, /* sdio, usb32_usb2_sys, usb32_ss_sys, pcie */
};

/* index -> CLK_DIS bit, north bridge */
static const uint8_t nb_gate_bit[] = {
	2,  3,  6,  7,      8, 10, /* mmc, sata_host, sec_at, sec_dap, tscem, tscem_tmx */
	11, 12, 13,                /* avs, sqf, pwm */
	16, 17, 19, 21,            /* i2c_2, i2c_1, ddr_phy, ddr_fclk */
	22, 23, 24, NO_GATE        /* trace, counter, eip97, cpu */
};

struct a37xx_clk_config {
	DEVICE_MMIO_ROM;
	const uint8_t *gate_bit;
	size_t ngates;
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

	if (idx >= cfg->ngates) {
		return -EINVAL;
	}
	bit = cfg->gate_bit[idx];
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

	if (idx >= cfg->ngates) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}
	if (cfg->gate_bit[idx] == NO_GATE) {
		return CLOCK_CONTROL_STATUS_ON;
	}
	return (sys_read32(DEVICE_MMIO_GET(dev) + CLK_DIS) & BIT(cfg->gate_bit[idx]))
		       ? CLOCK_CONTROL_STATUS_OFF
		       : CLOCK_CONTROL_STATUS_ON;
}

static int a37xx_clk_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	return 0;
}

static DEVICE_API(clock_control, a37xx_clk_api) = {
	.on = a37xx_clk_on,
	.off = a37xx_clk_off,
	.get_status = a37xx_clk_status,
};

#define A37XX_CLK_DEFINE(node, table)                                                              \
	static const struct a37xx_clk_config a37xx_clk_cfg_##node = {                              \
		DEVICE_MMIO_ROM_INIT(node),                                                        \
		.gate_bit = table,                                                                 \
		.ngates = ARRAY_SIZE(table),                                                       \
	};                                                                                         \
	static struct a37xx_clk_data a37xx_clk_data_##node;                                        \
	DEVICE_DT_DEFINE(node, a37xx_clk_init, NULL, &a37xx_clk_data_##node,                       \
			 &a37xx_clk_cfg_##node, PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,  \
			 &a37xx_clk_api);

#define A37XX_SB_DEFINE(node) A37XX_CLK_DEFINE(node, sb_gate_bit)
#define A37XX_NB_DEFINE(node) A37XX_CLK_DEFINE(node, nb_gate_bit)

DT_FOREACH_STATUS_OKAY(marvell_armada_3700_periph_clock_sb, A37XX_SB_DEFINE)
DT_FOREACH_STATUS_OKAY(marvell_armada_3700_periph_clock_nb, A37XX_NB_DEFINE)
