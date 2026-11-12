/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RK3399_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RK3399_PINCTRL_H_

/* pin within a bank: RK_PA0..RK_PD7 */
#define RK_PA0 0
#define RK_PA1 1
#define RK_PA2 2
#define RK_PA3 3
#define RK_PA4 4
#define RK_PA5 5
#define RK_PA6 6
#define RK_PA7 7
#define RK_PB0 8
#define RK_PB1 9
#define RK_PB2 10
#define RK_PB3 11
#define RK_PB4 12
#define RK_PB5 13
#define RK_PB6 14
#define RK_PB7 15
#define RK_PC0 16
#define RK_PC1 17
#define RK_PC2 18
#define RK_PC3 19
#define RK_PC4 20
#define RK_PC5 21
#define RK_PC6 22
#define RK_PC7 23
#define RK_PD0 24
#define RK_PD1 25
#define RK_PD2 26
#define RK_PD3 27
#define RK_PD4 28
#define RK_PD5 29
#define RK_PD6 30
#define RK_PD7 31

#define RK_FUNC_GPIO 0

/* logical pull; the driver translates for the 1.8 V-only pull type */
#define RK_PULL_KEEP 0 /* leave the pull register alone */
#define RK_PULL_NONE 1
#define RK_PULL_UP   2
#define RK_PULL_DOWN 3

/* bank 0..4, pin 0..31, function 0..3, pull RK_PULL_* */
#define RK_PINMUX(bank, pin, func, pull)                                                           \
	((((bank) & 0x7) << 24) | (((pin) & 0x1f) << 16) | (((func) & 0xf) << 8) | ((pull) & 0xf))

#endif
