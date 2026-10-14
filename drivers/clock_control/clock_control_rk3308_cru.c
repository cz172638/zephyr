/*
 * Copyright (c) 2026 Jiri Kastner <cz172638@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip RK3308 CRU clock gate controller.
 */

#define DT_DRV_COMPAT rockchip_rk3308_cru_clk

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/rk3308-cru.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>

LOG_MODULE_REGISTER(clock_control_rk3308_cru, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

#define RK3308_CRU_CLKGATE_CON(n) ((n) * 4U + 0x300U)

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
	rk3308_cru_gate_write(DEVICE_MMIO_GET(dev), (uint16_t)(uintptr_t)sys, true);

	return 0;
}

static int clock_control_rk3308_off(const struct device *dev, clock_control_subsys_t sys)
{
	rk3308_cru_gate_write(DEVICE_MMIO_GET(dev), (uint16_t)(uintptr_t)sys, false);

	return 0;
}

static int clock_control_rk3308_get_rate(const struct device *dev, clock_control_subsys_t sys,
					 uint32_t *rate)
{
	uint16_t clk_id = (uint16_t)(uintptr_t)sys;

	ARG_UNUSED(dev);

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
};

static struct clock_control_rk3308_data rk3308_cru_data;

static const struct clock_control_rk3308_config clock_control_rk3308_config = {
	DEVICE_MMIO_ROM_INIT(DT_INST_PARENT(0)),
};

DEVICE_DT_INST_DEFINE(0, clock_control_rk3308_init, NULL, &rk3308_cru_data,
		      &clock_control_rk3308_config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_rk3308_api);
