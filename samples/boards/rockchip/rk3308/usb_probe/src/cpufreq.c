/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * "cpu freq [MHz]": core clock at one of the operating points of the Linux
 * RK3308 devicetree, vdd_core (PWM regulator) set to the voltage of the
 * point first when the clock goes up and after it when it goes down.
 * "cpu vdd <mV>" sets vdd_core alone (to check it with a meter).
 * Until either is used, vdd_core runs at the voltage strapped on the board.
 */

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/dt-bindings/clock/rk3308-cru.h>
#include <zephyr/shell/shell.h>

struct opp {
	uint16_t mhz;
	uint16_t mv;
};

static const struct opp opps[] = {
	{408, 950},
	{600, 950},
	{816, 1025},
	{1008, 1125},
};

static const struct device *const cru = DEVICE_DT_GET(DT_NODELABEL(cru_clk));
static const struct device *const vdd = DEVICE_DT_GET(DT_NODELABEL(vdd_core));

static void show(const struct shell *sh)
{
	uint32_t hz = 0;
	int32_t uv;

	(void)clock_control_get_rate(cru, (clock_control_subsys_t)ARMCLK, &hz);
	if (regulator_get_voltage(vdd, &uv) == 0) {
		shell_print(sh, "cores %u MHz, vdd_core %d mV", hz / 1000000U, uv / 1000);
	} else {
		shell_print(sh, "cores %u MHz, vdd_core as strapped (not set)", hz / 1000000U);
	}
}

static int set_mv(const struct shell *sh, uint32_t mv)
{
	int ret = regulator_set_voltage(vdd, mv * 1000, mv * 1000);

	if (ret != 0) {
		shell_error(sh, "vdd_core %u mV: %d", mv, ret);
	}
	return ret;
}

static int cmd_freq(const struct shell *sh, size_t argc, char **argv)
{
	const struct opp *o = NULL;
	uint32_t old = 0, mhz;
	int ret;

	if (argc < 2) {
		show(sh);
		return 0;
	}
	mhz = strtoul(argv[1], NULL, 0);
	for (size_t i = 0; i < ARRAY_SIZE(opps); i++) {
		if (opps[i].mhz == mhz) {
			o = &opps[i];
		}
	}
	if (o == NULL) {
		shell_error(sh, "operating points: 408 600 816 1008 MHz");
		return -EINVAL;
	}
	(void)clock_control_get_rate(cru, (clock_control_subsys_t)ARMCLK, &old);
	if (mhz * 1000000U >= old) {
		ret = set_mv(sh, o->mv);
		if (ret != 0) {
			return ret;
		}
	}
	ret = clock_control_set_rate(cru, (clock_control_subsys_t)ARMCLK,
				     (clock_control_subsys_rate_t)(uintptr_t)(mhz * 1000000U));
	if (ret != 0) {
		shell_error(sh, "core clock %u MHz: %d", mhz, ret);
		return ret;
	}
	if (mhz * 1000000U < old) {
		ret = set_mv(sh, o->mv);
		if (ret != 0) {
			return ret;
		}
	}
	show(sh);
	return 0;
}

static int cmd_vdd(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (set_mv(sh, strtoul(argv[1], NULL, 0)) != 0) {
		return -EINVAL;
	}
	show(sh);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cpu,
	SHELL_CMD_ARG(freq, NULL, "[408|600|816|1008]: core clock and vdd_core", cmd_freq, 1, 1),
	SHELL_CMD_ARG(vdd, NULL, "<mV>: vdd_core alone (827..1340)", cmd_vdd, 2, 0),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(cpu, &sub_cpu, "Core clock and supply", NULL);
