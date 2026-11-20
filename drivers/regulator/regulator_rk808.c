/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip RK808 PMIC: the DCDC_REG1 and DCDC_REG2 bucks. The regulators
 * node is a child of the PMIC node, which sits on the I2C bus.
 */

#define DT_DRV_COMPAT rockchip_rk808_regulator

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/linear_range.h>
#include <zephyr/sys/util.h>

#define RK808_DCDC_EN    0x23U
#define RK808_VSEL_MASK  0x3fU
/* BUCKn_CONFIG[4:3]: ramp rate after a voltage change, 2/4/6/10 mV/us */
#define RK808_RAMP_SHIFT 3U
#define RK808_RAMP_MASK  (0x3U << RK808_RAMP_SHIFT)
/*
 * At most 100 mV per write: a larger step in one go can overshoot (Linux
 * rk808-regulator).
 */
#define RK808_MAX_STEPS  8U

struct regulator_rk808_desc {
	uint8_t config_reg;
	uint8_t vsel_reg;
	uint8_t en_mask;
};

struct regulator_rk808_config {
	struct regulator_common_config common;
	struct i2c_dt_spec i2c;
	const struct regulator_rk808_desc *desc;
};

struct regulator_rk808_data {
	struct regulator_common_data data;
};

static const struct linear_range buck_range = LINEAR_RANGE_INIT(712500, 12500U, 0U, 63U);
static const uint16_t ramp_uv_per_us[] = {2000U, 4000U, 6000U, 10000U};

static const struct regulator_rk808_desc dcdc_reg1_desc = {
	.config_reg = 0x2eU,
	.vsel_reg = 0x2fU,
	.en_mask = BIT(0),
};

static const struct regulator_rk808_desc dcdc_reg2_desc = {
	.config_reg = 0x32U,
	.vsel_reg = 0x33U,
	.en_mask = BIT(1),
};

static unsigned int regulator_rk808_count_voltages(const struct device *dev)
{
	ARG_UNUSED(dev);

	return linear_range_values_count(&buck_range);
}

static int regulator_rk808_list_voltage(const struct device *dev, unsigned int idx,
					int32_t *volt_uv)
{
	ARG_UNUSED(dev);

	return linear_range_get_value(&buck_range, idx, volt_uv);
}

static int regulator_rk808_set_voltage(const struct device *dev, int32_t min_uv, int32_t max_uv)
{
	const struct regulator_rk808_config *cfg = dev->config;
	uint8_t vsel, conf;
	uint16_t idx, cur;
	int ret;

	ret = linear_range_get_win_index(&buck_range, min_uv, max_uv, &idx);
	if (ret == -EINVAL) {
		return ret;
	}
	ret = i2c_reg_read_byte_dt(&cfg->i2c, cfg->desc->vsel_reg, &vsel);
	if (ret == 0) {
		ret = i2c_reg_read_byte_dt(&cfg->i2c, cfg->desc->config_reg, &conf);
	}
	if (ret != 0) {
		return ret;
	}

	cur = vsel & RK808_VSEL_MASK;
	while (cur != idx) {
		uint16_t next = (idx > cur) ? MIN(idx, cur + RK808_MAX_STEPS)
					    : MAX(idx, cur - RK808_MAX_STEPS);

		ret = i2c_reg_write_byte_dt(&cfg->i2c, cfg->desc->vsel_reg,
					    (vsel & ~RK808_VSEL_MASK) | (uint8_t)next);
		if (ret != 0) {
			return ret;
		}
		/* wait for the output to settle at the configured ramp rate */
		k_busy_wait(
			DIV_ROUND_UP(abs((int32_t)next - (int32_t)cur) * 12500,
				     ramp_uv_per_us[(conf & RK808_RAMP_MASK) >> RK808_RAMP_SHIFT]));
		cur = next;
	}
	return 0;
}

static int regulator_rk808_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_rk808_config *cfg = dev->config;
	uint8_t vsel;
	int ret;

	ret = i2c_reg_read_byte_dt(&cfg->i2c, cfg->desc->vsel_reg, &vsel);
	if (ret != 0) {
		return ret;
	}
	return linear_range_get_value(&buck_range, vsel & RK808_VSEL_MASK, volt_uv);
}

static int regulator_rk808_enable(const struct device *dev)
{
	const struct regulator_rk808_config *cfg = dev->config;

	return i2c_reg_update_byte_dt(&cfg->i2c, RK808_DCDC_EN, cfg->desc->en_mask,
				      cfg->desc->en_mask);
}

static int regulator_rk808_disable(const struct device *dev)
{
	const struct regulator_rk808_config *cfg = dev->config;

	return i2c_reg_update_byte_dt(&cfg->i2c, RK808_DCDC_EN, cfg->desc->en_mask, 0U);
}

static DEVICE_API(regulator, regulator_rk808_api) = {
	.enable = regulator_rk808_enable,
	.disable = regulator_rk808_disable,
	.count_voltages = regulator_rk808_count_voltages,
	.list_voltage = regulator_rk808_list_voltage,
	.set_voltage = regulator_rk808_set_voltage,
	.get_voltage = regulator_rk808_get_voltage,
};

static int regulator_rk808_init(const struct device *dev)
{
	const struct regulator_rk808_config *cfg = dev->config;
	uint8_t en;
	int ret;

	regulator_common_data_init(dev);

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		return -ENODEV;
	}
	ret = i2c_reg_read_byte_dt(&cfg->i2c, RK808_DCDC_EN, &en);
	if (ret != 0) {
		return ret;
	}
	return regulator_common_init(dev, (en & cfg->desc->en_mask) != 0U);
}

#define REGULATOR_RK808_DEFINE(node_id, id, name)                                                  \
	static struct regulator_rk808_data data_##id;                                              \
                                                                                                   \
	static const struct regulator_rk808_config config_##id = {                                 \
		.common = REGULATOR_DT_COMMON_CONFIG_INIT(node_id),                                \
		.i2c = I2C_DT_SPEC_GET(DT_GPARENT(node_id)),                                       \
		.desc = &name##_desc,                                                              \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_DEFINE(node_id, regulator_rk808_init, NULL, &data_##id, &config_##id,            \
			 POST_KERNEL, CONFIG_REGULATOR_RK808_INIT_PRIORITY, &regulator_rk808_api);

#define REGULATOR_RK808_DEFINE_COND(inst, child)                                                   \
	COND_CODE_1(DT_NODE_EXISTS(DT_INST_CHILD(inst, child)),                                    \
		    (REGULATOR_RK808_DEFINE(DT_INST_CHILD(inst, child), child##inst, child)), ())

#define REGULATOR_RK808_DEFINE_ALL(inst)                                                           \
	REGULATOR_RK808_DEFINE_COND(inst, dcdc_reg1)                                               \
	REGULATOR_RK808_DEFINE_COND(inst, dcdc_reg2)

DT_INST_FOREACH_STATUS_OKAY(REGULATOR_RK808_DEFINE_ALL)
