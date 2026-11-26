/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * RK3308 early setup the drivers do not do themselves: the enabled UARTs
 * get their clocks ungated and run from xin24m undivided (the devicetree
 * gives them 24 MHz, the boot loader only sets up the console UART), and
 * UART2 is routed to its M0 pins (GPIO1_C6/C7; M1 is on the SD card pins).
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>

#define CRU_NODE DT_NODELABEL(cru)
#define GRF_NODE DT_NODELABEL(grf)

/* CRU_CLKSEL_CON(10 + 3n): clk_uartn_src mux [15:13], 4 = xin24m; div [4:0] */
#define CRU_CLKSEL(n)        (0x100U + 4U * (n))
#define UART_SRC_CON(u)      CRU_CLKSEL(10U + 3U * (u))
#define UART_SRC_XIN24M      (4U << 13)
#define UART_SRC_MASK        (GENMASK(15, 13) | GENMASK(4, 0))
/* CRU_CLKSEL_CON(11 + 3n): clk_uartn mux [15:14], 0 = clk_uartn_src */
#define UART_SEL_CON(u)      CRU_CLKSEL(11U + 3U * (u))
#define UART_SEL_MASK        GENMASK(15, 14)

/*
 * CRU_CLKGATE_CON: four gates per UART from CON1 bit 9 on (src, np5, frac,
 * clk_uartn), counted across the registers; PCLK_UARTn in CON5 bit 10 + n
 */
#define CRU_CLKGATE(n)       (0x300U + 4U * (n))
#define UART_GATE_FIRST(u)   (16U + 9U + 4U * (u))
#define UART_PCLK_GATE(u)    (5U * 16U + 10U + (u))

/* GRF_SOC_CON5 (0x314): UART2 IO select [3:2], 0 = M0 */
#define GRF_SOC_CON5         0x314U
#define UART2_IO_SEL_MASK    GENMASK(3, 2)

#define UART_OKAY(u) DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(uart##u))

/* gates are hiword-masked, 0 = clock running */
static void ungate(mm_reg_t cru, uint32_t gate)
{
	sys_write32(BIT(16U + gate % 16U), cru + CRU_CLKGATE(gate / 16U));
}

static int rk3308_soc_init(void)
{
	static const bool uart_on[] = {
		UART_OKAY(0), UART_OKAY(1), UART_OKAY(2), UART_OKAY(3), UART_OKAY(4),
	};
	mm_reg_t cru, grf;

	device_map(&cru, DT_REG_ADDR(CRU_NODE), DT_REG_SIZE(CRU_NODE), K_MEM_CACHE_NONE);
	device_map(&grf, DT_REG_ADDR(GRF_NODE), DT_REG_SIZE(GRF_NODE), K_MEM_CACHE_NONE);

	for (uint32_t u = 0; u < ARRAY_SIZE(uart_on); u++) {
		if (!uart_on[u]) {
			continue;
		}
		ungate(cru, UART_PCLK_GATE(u));
		/* src (from xin24m below) and the final clk_uartn mux */
		ungate(cru, UART_GATE_FIRST(u));
		ungate(cru, UART_GATE_FIRST(u) + 3U);
		/* hiword-masked registers */
		sys_write32((UART_SRC_MASK << 16) | UART_SRC_XIN24M, cru + UART_SRC_CON(u));
		sys_write32(UART_SEL_MASK << 16, cru + UART_SEL_CON(u));
	}
	if (uart_on[2]) {
		sys_write32(UART2_IO_SEL_MASK << 16, grf + GRF_SOC_CON5);
	}
	return 0;
}

/* before the UART drivers */
SYS_INIT(rk3308_soc_init, PRE_KERNEL_1, 0);
