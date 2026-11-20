/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Silergy SYR827 step-down regulator (FAN53555 register layout).
 */

#define DT_DRV_COMPAT silergy_syr827

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/linear_range.h>
#include <zephyr/sys/util.h>

#define SYR827_VSEL(n)    (0x00U + (n))
#define SYR827_VSEL_EN    BIT(7)
#define SYR827_VSEL_MASK  0x3fU
#define SYR827_CONTROL    0x02U
/* CONTROL[6:4]: output slew rate after a voltage change */
#define SYR827_SLEW_SHIFT 4U
#define SYR827_SLEW_MASK  (0x7U << SYR827_SLEW_SHIFT)

struct regulator_syr827_config {
	struct regulator_common_config common;
	struct i2c_dt_spec i2c;
	uint8_t vsel_reg;
};

struct regulator_syr827_data {
	struct regulator_common_data data;
};

static const struct linear_range syr827_range = LINEAR_RANGE_INIT(712500, 12500U, 0U, 63U);
/* 64 mV/us down to 0.5 mV/us */
static const uint16_t slew_uv_per_us[] = {64000U, 32000U, 16000U, 8000U, 4000U, 2000U, 1000U, 500U};

static unsigned int regulator_syr827_count_voltages(const struct device *dev)
{
	ARG_UNUSED(dev);

	return linear_range_values_count(&syr827_range);
}

static int regulator_syr827_list_voltage(const struct device *dev, unsigned int idx,
					 int32_t *volt_uv)
{
	ARG_UNUSED(dev);

	return linear_range_get_value(&syr827_range, idx, volt_uv);
}

static int regulator_syr827_set_voltage(const struct device *dev, int32_t min_uv, int32_t max_uv)
{
	const struct regulator_syr827_config *cfg = dev->config;
	uint8_t vsel, ctrl;
	uint16_t idx;
	int ret;

	ret = linear_range_get_win_index(&syr827_range, min_uv, max_uv, &idx);
	if (ret == -EINVAL) {
		return ret;
	}
	ret = i2c_reg_read_byte_dt(&cfg->i2c, cfg->vsel_reg, &vsel);
	if (ret == 0) {
		ret = i2c_reg_read_byte_dt(&cfg->i2c, SYR827_CONTROL, &ctrl);
	}
	if (ret == 0) {
		ret = i2c_reg_write_byte_dt(&cfg->i2c, cfg->vsel_reg,
					    (vsel & ~SYR827_VSEL_MASK) | (uint8_t)idx);
	}
	if (ret != 0) {
		return ret;
	}
	/* wait for the output to settle at the configured slew rate */
	k_busy_wait(DIV_ROUND_UP(abs((int32_t)idx - (int32_t)(vsel & SYR827_VSEL_MASK)) * 12500,
				 slew_uv_per_us[(ctrl & SYR827_SLEW_MASK) >> SYR827_SLEW_SHIFT]));
	return 0;
}

static int regulator_syr827_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_syr827_config *cfg = dev->config;
	uint8_t vsel;
	int ret;

	ret = i2c_reg_read_byte_dt(&cfg->i2c, cfg->vsel_reg, &vsel);
	if (ret != 0) {
		return ret;
	}
	return linear_range_get_value(&syr827_range, vsel & SYR827_VSEL_MASK, volt_uv);
}

static int regulator_syr827_enable(const struct device *dev)
{
	const struct regulator_syr827_config *cfg = dev->config;

	return i2c_reg_update_byte_dt(&cfg->i2c, cfg->vsel_reg, SYR827_VSEL_EN, SYR827_VSEL_EN);
}

static int regulator_syr827_disable(const struct device *dev)
{
	const struct regulator_syr827_config *cfg = dev->config;

	return i2c_reg_update_byte_dt(&cfg->i2c, cfg->vsel_reg, SYR827_VSEL_EN, 0U);
}

static DEVICE_API(regulator, regulator_syr827_api) = {
	.enable = regulator_syr827_enable,
	.disable = regulator_syr827_disable,
	.count_voltages = regulator_syr827_count_voltages,
	.list_voltage = regulator_syr827_list_voltage,
	.set_voltage = regulator_syr827_set_voltage,
	.get_voltage = regulator_syr827_get_voltage,
};

static int regulator_syr827_init(const struct device *dev)
{
	const struct regulator_syr827_config *cfg = dev->config;
	uint8_t vsel;
	int ret;

	regulator_common_data_init(dev);

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		return -ENODEV;
	}
	ret = i2c_reg_read_byte_dt(&cfg->i2c, cfg->vsel_reg, &vsel);
	if (ret != 0) {
		return ret;
	}
	return regulator_common_init(dev, (vsel & SYR827_VSEL_EN) != 0U);
}

#define REGULATOR_SYR827_DEFINE(inst)                                                              \
	static struct regulator_syr827_data data_##inst;                                           \
                                                                                                   \
	static const struct regulator_syr827_config config_##inst = {                              \
		.common = REGULATOR_DT_INST_COMMON_CONFIG_INIT(inst),                              \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.vsel_reg = SYR827_VSEL(DT_INST_PROP(inst, silergy_vsel)),                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, regulator_syr827_init, NULL, &data_##inst, &config_##inst,     \
			      POST_KERNEL, CONFIG_REGULATOR_SYR827_INIT_PRIORITY,                  \
			      &regulator_syr827_api);

DT_INST_FOREACH_STATUS_OKAY(REGULATOR_SYR827_DEFINE)
