/*
 * Copyright (c) 2026 Jiri Kastner <cz172638@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RK3308_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RK3308_PINCTRL_H_

/*
 * RK3308 iomux is irregular (mixed 2- and 4-bit fields, some banks split into
 * _L/_H registers), so each pin encodes its GRF iomux field explicitly instead
 * of being derived from a bank/pin number:
 *
 *   bits [15:0]  GRF iomux register offset (from GRF base 0xff000000)
 *   bits [19:16] field shift within that 32-bit register (0..15)
 *   bits [22:20] field width in bits (1..4)
 *   bits [26:24] mux function value
 *
 * The field must lie within the low 16 bits of its register; GRF iomux
 * registers are hiword-masked, so the driver writes the write-enable mask in
 * bits [31:16]. Offsets and fields: RK3308 TRM v1.1 Part1, GRF_GPIO*_IOMUX*.
 */
#define RK3308_PINMUX(off, shift, width, mux)                                                      \
	(((off) & 0xffff) | (((shift) & 0xf) << 16) | (((width) & 0x7) << 20) |                     \
	 (((mux) & 0xf) << 24))

/* UART0 console: GPIO2_A0/A1, GRF_GPIO2A_IOMUX (0x0040), func 1 */
#define UART0_RX_M0	RK3308_PINMUX(0x0040, 0, 2, 1)
#define UART0_TX_M0	RK3308_PINMUX(0x0040, 2, 2, 1)

/*
 * To add a group, read its pins' GRF_GPIO*_IOMUX* field from the TRM and build
 * one RK3308_PINMUX() per pin, e.g. the MAC2IO RMII function (func 4) lives on
 * GPIO1B/GPIO1C (GRF_GPIO1B_IOMUX_L 0x0028, _H 0x002c; GPIO1C_IOMUX_L 0x0030,
 * _H 0x0034). Mux-only for now; pull/drive are not configured.
 */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RK3308_PINCTRL_H_ */
