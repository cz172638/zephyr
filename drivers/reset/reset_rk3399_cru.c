/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip RK3399 CRU/PMUCRU soft reset controller.
 */

#define DT_DRV_COMPAT rockchip_rk3399_cru_rst

#include <errno.h>
#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

/* Highest SOFTRST_CON index in the CRU */
#define RK3399_CRU_SOFTRST_NUM 21U

struct reset_rk3399_config {
	DEVICE_MMIO_ROM;
	uint32_t softrst_offset;
};

struct reset_rk3399_data {
	DEVICE_MMIO_RAM;
};

static int reset_rk3399_reg(const struct device *dev, uint32_t id, mm_reg_t *reg)
{
	const struct reset_rk3399_config *cfg = dev->config;

	if ((id / 16U) >= RK3399_CRU_SOFTRST_NUM) {
		return -EINVAL;
	}

	*reg = DEVICE_MMIO_GET(dev) + cfg->softrst_offset + ((id / 16U) * 4U);

	return 0;
}

static int reset_rk3399_status(const struct device *dev, uint32_t id, uint8_t *status)
{
	mm_reg_t reg;
	int ret = reset_rk3399_reg(dev, id, &reg);

	if (ret != 0) {
		return ret;
	}

	*status = ((sys_read32(reg) & BIT(id % 16U)) != 0U) ? 1U : 0U;

	return 0;
}

static int reset_rk3399_set(const struct device *dev, uint32_t id, bool assert)
{
	mm_reg_t reg;
	int ret = reset_rk3399_reg(dev, id, &reg);

	if (ret != 0) {
		return ret;
	}

	sys_write32((BIT(id % 16U) << 16) | (assert ? BIT(id % 16U) : 0U), reg);

	return 0;
}

static int reset_rk3399_line_assert(const struct device *dev, uint32_t id)
{
	return reset_rk3399_set(dev, id, true);
}

static int reset_rk3399_line_deassert(const struct device *dev, uint32_t id)
{
	return reset_rk3399_set(dev, id, false);
}

static int reset_rk3399_line_toggle(const struct device *dev, uint32_t id)
{
	int ret = reset_rk3399_set(dev, id, true);

	if (ret != 0) {
		return ret;
	}

	return reset_rk3399_set(dev, id, false);
}

static int reset_rk3399_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	return 0;
}

static DEVICE_API(reset, reset_rk3399_api) = {
	.status = reset_rk3399_status,
	.line_assert = reset_rk3399_line_assert,
	.line_deassert = reset_rk3399_line_deassert,
	.line_toggle = reset_rk3399_line_toggle,
};

#define RK3399_CRU_RST_INIT(n)                                                                     \
	static struct reset_rk3399_data reset_rk3399_data_##n;                                     \
	static const struct reset_rk3399_config reset_rk3399_config_##n = {                       \
		DEVICE_MMIO_ROM_INIT(DT_INST_PARENT(n)),                                           \
		.softrst_offset = DT_INST_PROP(n, rockchip_softrst_offset),                        \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, reset_rk3399_init, NULL, &reset_rk3399_data_##n,                \
			      &reset_rk3399_config_##n, PRE_KERNEL_1, CONFIG_RESET_INIT_PRIORITY,  \
			      &reset_rk3399_api);

DT_INST_FOREACH_STATUS_OKAY(RK3399_CRU_RST_INIT)
