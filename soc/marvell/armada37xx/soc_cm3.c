/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Armada 3720 Cortex-M3: measure the CM3 clock from the north bridge
 * clock tree (TBG and dividers, as A3700-utils wtmi/common/delay.c)
 * so SysTick and k_busy_wait() are right whatever preset is set.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/platform/hooks.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <soc.h>

#include "wtmi/a3700_wtmi.h"

#define NB_CLK          (A3700_CM3_SOC_REGS + 0x13000)
#define NB_TBG_SEL      (NB_CLK + 0x0)
#define NB_DIV_SEL1     (NB_CLK + 0x8)
#define NB_CLK_SEL      (NB_CLK + 0x10)
#define NB_TBG_CTRL0    (NB_CLK + 0x204)
#define NB_TBG_CTRL1    (NB_CLK + 0x208)
#define NB_TBG_CTRL7    (NB_CLK + 0x220)
#define NB_TBG_CTRL8    (NB_CLK + 0x230)
#define NB_TEST_PIN     (NB_CLK + 0x808)
#define XTAL_40MHZ      BIT(9)

enum tbg { TBG_A_P, TBG_B_P, TBG_A_S, TBG_B_S };

uint32_t a3700_ref_clk_hz(void)
{
	return (sys_read32(NB_TEST_PIN) & XTAL_40MHZ) ? MHZ(40) : MHZ(25);
}

/* Ftbg = N * Fref * 4 / M / 2^vcodiv */
static uint32_t tbg_hz(enum tbg t)
{
	bool a = t == TBG_A_P || t == TBG_A_S;
	uint32_t m = sys_read32(NB_TBG_CTRL7);
	uint32_t n = sys_read32(NB_TBG_CTRL0);
	uint32_t vco;

	m = a ? (m & 0x1ff) : ((m >> 16) & 0x1ff);
	n = a ? ((n >> 2) & 0x1ff) : ((n >> 18) & 0x1ff);
	if (m == 0) {
		m = 1;
	}

	if (t == TBG_A_S || t == TBG_B_S) {
		vco = sys_read32(NB_TBG_CTRL1);
		vco = a ? (vco & 0x1ff) : ((vco >> 16) & 0x1ff);
	} else {
		vco = sys_read32(NB_TBG_CTRL8);
		vco = a ? ((vco >> 1) & 0x1ff) : ((vco >> 17) & 0x1ff);
	}
	if (vco > 7) {
		return 0;
	}

	return (uint32_t)(((uint64_t)n * a3700_ref_clk_hz() * 4) / (m << vco));
}

uint32_t a3700_cm3_clk_hz(void)
{
	uint32_t div = sys_read32(NB_DIV_SEL1);
	uint32_t d1 = (div >> 15) & 0x7;
	uint32_t d2 = (div >> 12) & 0x7;
	uint32_t tbg;

	/* CM3 on the crystal */
	if (((sys_read32(NB_CLK_SEL) >> 4) & 0x1) == 0) {
		return a3700_ref_clk_hz();
	}

	tbg = tbg_hz((sys_read32(NB_TBG_SEL) >> 8) & 0x3);
	if (tbg == 0 || d1 == 0 || d1 == 7 || d2 == 0 || d2 == 7) {
		return 0;
	}
	return tbg / (d1 * d2);
}

void soc_early_init_hook(void)
{
	uint32_t hz;

	if (IS_ENABLED(CONFIG_SOC_ARMADA3720_CM3_SYS_INIT)) {
		(void)a3700_wtmi_sys_init();
	}
	if (IS_ENABLED(CONFIG_SOC_ARMADA3720_CM3_START_AP)) {
		a3700_wtmi_start_ap(CONFIG_SOC_ARMADA3720_CM3_AP_ENTRY);
	}

	hz = a3700_cm3_clk_hz();

	if (hz != 0) {
		z_sys_clock_hw_cycles_per_sec_update(hz);
	}
}
