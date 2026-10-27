/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip RK3399 CRU/PMUCRU clock gate controller.
 */

#define DT_DRV_COMPAT rockchip_rk3399_cru_clk

#include <errno.h>
#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
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

static int clock_control_rk3399_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	return 0;
}

static DEVICE_API(clock_control, clock_control_rk3399_api) = {
	.on = clock_control_rk3399_on,
	.off = clock_control_rk3399_off,
	.get_status = clock_control_rk3399_get_status,
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
