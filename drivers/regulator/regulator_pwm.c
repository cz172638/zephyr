/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Regulator with its output voltage set by the duty cycle of a PWM
 * (continuous mode of the Linux pwm-regulator binding): the voltage moves
 * linearly from regulator-min-microvolt to regulator-max-microvolt while
 * the duty cycle goes over pwm-dutycycle-range. The output is left alone
 * until the first voltage is set (or regulator-init-microvolt), so a
 * supply the boot loader left running keeps its voltage.
 */

#define DT_DRV_COMPAT pwm_regulator

#include <errno.h>
#include <stdint.h>

#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(regulator_pwm, CONFIG_REGULATOR_LOG_LEVEL);

struct regulator_pwm_config {
	struct regulator_common_config common;
	struct pwm_dt_spec pwm;
	int32_t min_uv;
	int32_t max_uv;
	uint32_t duty_first;
	uint32_t duty_last;
	uint32_t duty_unit;
	uint32_t settling_up_us;
};

struct regulator_pwm_data {
	struct regulator_common_data common;
	int32_t uv; /* INT32_MIN: not set yet */
};

static int regulator_pwm_set_voltage(const struct device *dev, int32_t min_uv, int32_t max_uv)
{
	const struct regulator_pwm_config *cfg = dev->config;
	struct regulator_pwm_data *data = dev->data;
	int32_t span = cfg->max_uv - cfg->min_uv;
	int64_t duty, period, pulse;
	uint64_t cps;
	int32_t uv;
	int ret;

	uv = CLAMP(min_uv, cfg->min_uv, cfg->max_uv);
	if (uv > max_uv) {
		return -EINVAL;
	}
	ret = pwm_get_cycles_per_sec(cfg->pwm.dev, cfg->pwm.channel, &cps);
	if (ret != 0) {
		return ret;
	}
	period = (int64_t)cfg->pwm.period * (int64_t)cps / NSEC_PER_SEC;
	if (period <= 0) {
		return -EINVAL;
	}
	/* duty in pwm-dutycycle-unit, then rounded up to whole PWM cycles */
	duty = (int64_t)cfg->duty_first;
	if (span > 0) {
		duty += ((int64_t)cfg->duty_last - (int64_t)cfg->duty_first) *
			(uv - cfg->min_uv) / span;
	}
	pulse = DIV_ROUND_UP(duty * period, (int64_t)cfg->duty_unit);
	ret = pwm_set_cycles(cfg->pwm.dev, cfg->pwm.channel, (uint32_t)period,
			     (uint32_t)CLAMP(pulse, 0, period), cfg->pwm.flags);
	if (ret != 0) {
		return ret;
	}
	if ((data->uv == INT32_MIN || uv > data->uv) && cfg->settling_up_us > 0U) {
		k_busy_wait(cfg->settling_up_us);
	}
	data->uv = uv;
	return 0;
}

static int regulator_pwm_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_pwm_data *data = dev->data;

	if (data->uv == INT32_MIN) {
		return -ENODATA;
	}
	*volt_uv = data->uv;
	return 0;
}

static DEVICE_API(regulator, regulator_pwm_api) = {
	.set_voltage = regulator_pwm_set_voltage,
	.get_voltage = regulator_pwm_get_voltage,
};

static int regulator_pwm_init(const struct device *dev)
{
	const struct regulator_pwm_config *cfg = dev->config;

	regulator_common_data_init(dev);
	if (!pwm_is_ready_dt(&cfg->pwm)) {
		return -ENODEV;
	}
	return regulator_common_init(dev, true);
}

#define REGULATOR_PWM_DEFINE(n)                                                                    \
	BUILD_ASSERT(DT_INST_PROP_LEN(n, pwm_dutycycle_range) == 2,                                \
		     "pwm-dutycycle-range: <first last>");                                         \
	static struct regulator_pwm_data regulator_pwm_data_##n = {                                \
		.uv = INT32_MIN,                                                                   \
	};                                                                                         \
	static const struct regulator_pwm_config regulator_pwm_config_##n = {                      \
		.common = REGULATOR_DT_INST_COMMON_CONFIG_INIT(n),                                 \
		.pwm = PWM_DT_SPEC_INST_GET(n),                                                    \
		.min_uv = DT_INST_PROP(n, regulator_min_microvolt),                                \
		.max_uv = DT_INST_PROP(n, regulator_max_microvolt),                                \
		.duty_first = DT_INST_PROP_BY_IDX(n, pwm_dutycycle_range, 0),                      \
		.duty_last = DT_INST_PROP_BY_IDX(n, pwm_dutycycle_range, 1),                       \
		.duty_unit = DT_INST_PROP(n, pwm_dutycycle_unit),                                  \
		.settling_up_us = DT_INST_PROP(n, regulator_settling_time_up_us),                  \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, regulator_pwm_init, NULL, &regulator_pwm_data_##n,                \
			      &regulator_pwm_config_##n, POST_KERNEL,                              \
			      CONFIG_REGULATOR_PWM_INIT_PRIORITY, &regulator_pwm_api);

DT_INST_FOREACH_STATUS_OKAY(REGULATOR_PWM_DEFINE)
