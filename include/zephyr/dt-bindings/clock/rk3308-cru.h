/*
 * Copyright (c) 2026 Jiri Kastner <cz172638@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RK3308_CRU_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RK3308_CRU_H_

/*
 * Clock IDs for the RK3308 CRU gate controller. Each ID directly encodes the
 * gate location: CRU_CLKGATE_CON index in bits [15:8], bit position in [7:0]
 * (CRU_CLKGATE_CON<n> is at CRU + 0x0300 + n * 4). Source: RK3308 TRM v1.1
 * Part1, CRU_CLKGATE_CON0..14.
 */
#define RK3308_CLK_GATE(con, bit) (((con) << 8) | (bit))

/* UART functional clocks (SCLK_UARTn) */
#define SCLK_UART0	RK3308_CLK_GATE(1, 12)
#define SCLK_UART1	RK3308_CLK_GATE(1, 0)
#define SCLK_UART2	RK3308_CLK_GATE(2, 4)
#define SCLK_UART3	RK3308_CLK_GATE(2, 8)
#define SCLK_UART4	RK3308_CLK_GATE(2, 12)

/* UART APB clocks (PCLK_UARTn) */
#define PCLK_UART0	RK3308_CLK_GATE(5, 10)
#define PCLK_UART1	RK3308_CLK_GATE(5, 11)
#define PCLK_UART2	RK3308_CLK_GATE(5, 12)
#define PCLK_UART3	RK3308_CLK_GATE(5, 13)
#define PCLK_UART4	RK3308_CLK_GATE(5, 14)

/* GPIO APB clocks (PCLK_GPIOn) */
#define PCLK_GPIO0	RK3308_CLK_GATE(6, 12)
#define PCLK_GPIO1	RK3308_CLK_GATE(6, 13)
#define PCLK_GPIO2	RK3308_CLK_GATE(6, 14)
#define PCLK_GPIO3	RK3308_CLK_GATE(6, 15)
#define PCLK_GPIO4	RK3308_CLK_GATE(7, 0)

/* MAC (MAC2IO 10/100) */
#define SCLK_MAC	RK3308_CLK_GATE(8, 14)
#define ACLK_MAC	RK3308_CLK_GATE(9, 4)
#define PCLK_MAC	RK3308_CLK_GATE(9, 15)
#define SCLK_MAC_REF	RK3308_CLK_GATE(9, 1)
#define SCLK_MAC_RXTX	RK3308_CLK_GATE(9, 0)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RK3308_CRU_H_ */
