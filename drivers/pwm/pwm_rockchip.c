/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip PWM (the RK3328 generation, also in RK3308): one channel per
 * node, continuous left-aligned output, period and duty in clock cycles of
 * the PWM clock, updated together under the lock bit. The pin is muxed to
 * the PWM only by the first set_cycles: until then whatever the pin drives
 * (e.g. the feedback of a PWM-set regulator) is left as the boot loader left
 * it.
 */

#define DT_DRV_COMPAT rockchip_rk3328_pwm

#include <errno.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#define PWM_PERIOD 0x04U
#define PWM_DUTY   0x08U
#define PWM_CTRL   0x0cU

#define CTRL_ENABLE        BIT(0)
#define CTRL_CONTINUOUS    BIT(1)
#define CTRL_DUTY_POS      BIT(3)
#define CTRL_INACTIVE_POS  BIT(4)
#define CTRL_LOCK          BIT(6)
#define CTRL_MODE_MASK     (GENMASK(2, 0) | BIT(5) | BIT(8))

struct pwm_rockchip_config {
	DEVICE_MMIO_ROM;
	const struct device *clk_dev;
	clock_control_subsys_t clk;
	clock_control_subsys_t pclk;
	uint32_t clk_rate;
	const struct pinctrl_dev_config *pcfg;
};

struct pwm_rockchip_data {
	DEVICE_MMIO_RAM;
	bool pinmuxed;
};

static int pwm_rockchip_set_cycles(const struct device *dev, uint32_t channel,
				   uint32_t period, uint32_t pulse, pwm_flags_t flags)
{
	const struct pwm_rockchip_config *cfg = dev->config;
	struct pwm_rockchip_data *data = dev->data;
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	int ret;
	uint32_t ctrl;

	if (channel != 0U) {
		return -EINVAL;
	}
	if (period == 0U) {
		sys_write32(sys_read32(base + PWM_CTRL) & ~CTRL_ENABLE, base + PWM_CTRL);
		return 0;
	}
	ctrl = sys_read32(base + PWM_CTRL) & ~(CTRL_MODE_MASK | CTRL_DUTY_POS | CTRL_INACTIVE_POS);
	/* active level for pulse cycles, the other one for the rest */
	ctrl |= (flags & PWM_POLARITY_INVERTED) != 0U ? CTRL_INACTIVE_POS : CTRL_DUTY_POS;
	ctrl |= CTRL_CONTINUOUS | CTRL_ENABLE;

	sys_write32(sys_read32(base + PWM_CTRL) | CTRL_LOCK, base + PWM_CTRL);
	sys_write32(period, base + PWM_PERIOD);
	sys_write32(MIN(pulse, period), base + PWM_DUTY);
	/* unlock: the new period and duty take effect together */
	sys_write32(ctrl & ~CTRL_LOCK, base + PWM_CTRL);

	if (!data->pinmuxed) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret != 0 && ret != -ENOENT) {
			return ret;
		}
		data->pinmuxed = true;
	}
	return 0;
}

static int pwm_rockchip_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					   uint64_t *cycles)
{
	const struct pwm_rockchip_config *cfg = dev->config;
	uint32_t rate;

	if (channel != 0U) {
		return -EINVAL;
	}
	if (clock_control_get_rate(cfg->clk_dev, cfg->clk, &rate) != 0) {
		return -EIO;
	}
	*cycles = rate;
	return 0;
}

static DEVICE_API(pwm, pwm_rockchip_api) = {
	.set_cycles = pwm_rockchip_set_cycles,
	.get_cycles_per_sec = pwm_rockchip_get_cycles_per_sec,
};

static int pwm_rockchip_init(const struct device *dev)
{
	const struct pwm_rockchip_config *cfg = dev->config;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	if (!device_is_ready(cfg->clk_dev)) {
		return -ENODEV;
	}
	(void)clock_control_on(cfg->clk_dev, cfg->pclk);
	(void)clock_control_on(cfg->clk_dev, cfg->clk);
	ret = clock_control_set_rate(cfg->clk_dev, cfg->clk,
				     (clock_control_subsys_rate_t)(uintptr_t)cfg->clk_rate);
	return ret == -ENOTSUP ? 0 : ret;
}

#define PWM_ROCKCHIP_INIT(n)                                                                       \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct pwm_rockchip_config pwm_rockchip_config_##n = {                        \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, pwm)),                     \
		.clk = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, pwm, clkid),         \
		.pclk = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, pclk, clkid),       \
		.clk_rate = DT_INST_PROP(n, clock_frequency),                                      \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
	};                                                                                         \
	static struct pwm_rockchip_data pwm_rockchip_data_##n;                                     \
	DEVICE_DT_INST_DEFINE(n, pwm_rockchip_init, NULL, &pwm_rockchip_data_##n,                  \
			      &pwm_rockchip_config_##n, POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,     \
			      &pwm_rockchip_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_ROCKCHIP_INIT)
