/*
 * Copyright (c) 2026 Jiri Kastner <cz172638@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RK3308_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RK3308_PINCTRL_H_

/*
 * RK3308 iomux is irregular (mixed 2- and 4-bit fields, some banks split into
 * _L/_H registers), so each pin encodes its GRF iomux field explicitly instead
 * of deriving it from the bank/pin number; bank and pin locate its pull
 * field (GRF_GPIO<bank><port>_P, regular: 2 bits per pin):
 *
 *   bits [11:0]  GRF iomux register offset (from GRF base 0xff000000; the
 *                recalculated fields of some pins lie at 0x6xx)
 *   bits [15:12] field shift within that 32-bit register (0..15)
 *   bits [18:16] field width in bits (1..4)
 *   bits [22:19] mux function value
 *   bits [25:23] GPIO bank (0..4)
 *   bits [30:26] pin within the bank (0..31, A0 = 0, D7 = 31)
 *
 * The field must lie within the low 16 bits of its register; GRF iomux
 * registers are hiword-masked, so the driver writes the write-enable mask in
 * bits [31:16]. Offsets and fields: RK3308 TRM v1.1 Part1, GRF_GPIO*_IOMUX*.
 */
#define RK3308_PINMUX(bank, pin, off, shift, width, mux)                                           \
	(((off) & 0xfff) | (((shift) & 0xf) << 12) | (((width) & 0x7) << 16) |                      \
	 (((mux) & 0xf) << 19) | (((bank) & 0x7) << 23) | (((pin) & 0x1f) << 26))

/* the same pin with its GPIO function (mux 0) */
#define RK3308_GPIO_FUNC(pinmux) ((pinmux) & ~(0xf << 19))

/* pin number within a bank: port A..D (0..3), pin 0..7 */
#define RK3308_PIN(port, n) ((port) * 8 + (n))

/* UART0 console: GPIO2_A0/A1, GRF_GPIO2A_IOMUX (0x0040), func 1 */
#define UART0_RX_M0	RK3308_PINMUX(2, RK3308_PIN(0, 0), 0x40, 0, 2, 1)
#define UART0_TX_M0	RK3308_PINMUX(2, RK3308_PIN(0, 1), 0x40, 2, 2, 1)

/* UART1: GPIO1_D0/D1, GRF_GPIO1D_IOMUX (0x0038), func 1 */
#define UART1_RX	RK3308_PINMUX(1, RK3308_PIN(3, 0), 0x38, 0, 2, 1)
#define UART1_TX	RK3308_PINMUX(1, RK3308_PIN(3, 1), 0x38, 2, 2, 1)

/* UART2 M0: GPIO1_C6/C7, GRF_GPIO1CH_IOMUX (0x0034), 4-bit fields, func 2 */
#define UART2_RX_M0	RK3308_PINMUX(1, RK3308_PIN(2, 6), 0x34, 4, 4, 2)
#define UART2_TX_M0	RK3308_PINMUX(1, RK3308_PIN(2, 7), 0x34, 8, 4, 2)

/* UART3 M0: GPIO3_B4/B5, GRF_GPIO3B_IOMUX (0x0068), 4-bit fields, func 4 */
#define UART3_RX_M0	RK3308_PINMUX(3, RK3308_PIN(1, 4), 0x68, 8, 4, 4)
#define UART3_TX_M0	RK3308_PINMUX(3, RK3308_PIN(1, 5), 0x68, 12, 4, 4)

/*
 * SDMMC: GPIO4_D0..D5, GRF_GPIO4D_IOMUX (0x0098), func 1; card detect
 * GPIO0_A3, GRF_GPIO0A_IOMUX (0x0000), func 1 (Linux rk3308.dtsi)
 */
#define SDMMC_D0	RK3308_PINMUX(4, RK3308_PIN(3, 0), 0x98, 0, 2, 1)
#define SDMMC_D1	RK3308_PINMUX(4, RK3308_PIN(3, 1), 0x98, 2, 2, 1)
#define SDMMC_D2	RK3308_PINMUX(4, RK3308_PIN(3, 2), 0x98, 4, 2, 1)
#define SDMMC_D3	RK3308_PINMUX(4, RK3308_PIN(3, 3), 0x98, 6, 2, 1)
#define SDMMC_CMD	RK3308_PINMUX(4, RK3308_PIN(3, 4), 0x98, 8, 2, 1)
#define SDMMC_CLK	RK3308_PINMUX(4, RK3308_PIN(3, 5), 0x98, 10, 2, 1)
#define SDMMC_DET	RK3308_PINMUX(0, RK3308_PIN(0, 3), 0x00, 6, 2, 1)

/* PWM0: GPIO0_B5, GRF_GPIO0B_IOMUX (0x0008), func 1 (vdd_core on ROCK Pi S) */
#define PWM0		RK3308_PINMUX(0, RK3308_PIN(1, 5), 0x08, 10, 2, 1)

/*
 * SPI1 (header pins 42 MISO, 41 CLK, 40 MOSI, 39 CSN0): GPIO3_B2/B3 have
 * recalculated 3-bit fields in 0x0610, B4/B5 4-bit fields in GRF_GPIO3B_IOMUX
 * (0x0068); func 3 (Linux pinctrl-rockchip rk3308_mux_recalced_data)
 */
#define SPI1_MISO	RK3308_PINMUX(3, RK3308_PIN(1, 2), 0x610, 0, 3, 3)
#define SPI1_CLK	RK3308_PINMUX(3, RK3308_PIN(1, 3), 0x610, 4, 3, 3)
#define SPI1_MOSI	RK3308_PINMUX(3, RK3308_PIN(1, 4), 0x68, 8, 4, 3)
#define SPI1_CSN0	RK3308_PINMUX(3, RK3308_PIN(1, 5), 0x68, 12, 4, 3)

/*
 * To add a group, read its pins' GRF_GPIO*_IOMUX* field from the TRM and build
 * one RK3308_PINMUX() per pin, e.g. the MAC2IO RMII function (func 4) lives on
 * GPIO1B/GPIO1C (GRF_GPIO1B_IOMUX_L 0x0028, _H 0x002c; GPIO1C_IOMUX_L 0x0030,
 * _H 0x0034). Pull: bias-pull-up, bias-pull-down or bias-disable on the group;
 * drive strength is not configured.
 */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RK3308_PINCTRL_H_ */
