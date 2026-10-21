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

#define UART0      (A3700_CM3_SOC_REGS + 0x12000)
#define UART_STAT  (UART0 + 0x0c)
#define UART_BRDV  (UART0 + 0x10)
#define UART_OSAMP (UART0 + 0x14)
#define UART_TX_EMP BIT(6)

/*
 * The BootROM leaves UART0 at whatever the loader used (mox-imager: up
 * to 6 Mbaud); set 115200 from the crystal, as the WTMI uart_init(), so
 * the output before the console driver is readable too.
 */
static void early_uart(uint32_t baud)
{
	uint32_t div = DIV_ROUND_CLOSEST(a3700_ref_clk_hz(), 16 * baud);
	int wait = 100000;

	while (--wait > 0 && (sys_read32(UART_STAT) & UART_TX_EMP) == 0) {
	}
	sys_write32((sys_read32(UART_BRDV) & ~(0x3ff | BIT(19))) | div, UART_BRDV);
	sys_write32(0, UART_OSAMP);
}

#ifdef CONFIG_SOC_ARMADA3720_CM3_REBOOT
#include <zephyr/drivers/mbox/mbox_mvebu_rwtm.h>

#define NB_WARM_RESET       (A3700_CM3_SOC_REGS + 0x13840)
#define NB_WARM_RESET_MAGIC 0x1d1e

uint32_t mvebu_rwtm_sp_legacy_cmd(uint16_t cmd, const uint32_t args[MVEBU_RWTM_NARGS],
				  uint32_t out[MVEBU_RWTM_NARGS])
{
	ARG_UNUSED(out);

	if (cmd == MVEBU_RWTM_CMD_REBOOT) {
		if (args[0] != MVEBU_RWTM_REBOOT_MAGIC) {
			return MVEBU_RWTM_STS(cmd, 0, FAIL);
		}
		int wait = 100000;

		printk("Zephyr CM3: reboot requested by the A53\n");
		/* let the UART drain, the reset would cut the line */
		while (--wait > 0 && (sys_read32(UART_STAT) & UART_TX_EMP) == 0) {
			k_busy_wait(1);
		}
		sys_write32(NB_WARM_RESET_MAGIC, NB_WARM_RESET);
		for (;;) {
		}
	}
	return MVEBU_RWTM_STS(cmd, 0, BADCMD);
}
#endif

void soc_early_init_hook(void)
{
	uint32_t hz;

	early_uart(115200);

#ifdef CONFIG_SOC_ARMADA3720_CM3_SYS_INIT
	(void)a3700_wtmi_sys_init();
#endif
#ifdef CONFIG_SOC_ARMADA3720_CM3_START_AP
	a3700_wtmi_start_ap(CONFIG_SOC_ARMADA3720_CM3_AP_ENTRY);
#endif

	hz = a3700_cm3_clk_hz();

	if (hz != 0) {
		z_sys_clock_hw_cycles_per_sec_update(hz);
	}
}
