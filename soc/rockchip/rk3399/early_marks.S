/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * CONFIG_RK3399_EARLY_MARKS: raw characters on UART2 (left at 1.5 Mbaud by
 * U-Boot SPL) from the reset path, before any driver: "Z3"/"Z2"/"Z1" =
 * entered at EL3/EL2/EL1. No stack here; x23..x25 must survive.
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>

#define RK3399_UART2 0xff1a0000

.macro putc ch
	ldr	x0, =RK3399_UART2
1:	ldr	w1, [x0, #0x14]		/* LSR */
	tbz	w1, #5, 1b		/* THRE */
	mov	w1, #\ch
	str	w1, [x0]		/* THR, 32-bit access */
.endm

GTEXT(z_arm64_el3_plat_prep_c)
SECTION_FUNC(TEXT, z_arm64_el3_plat_prep_c)
	putc	'Z'
	putc	'3'
	ret

GTEXT(z_arm64_el2_plat_prep_c)
SECTION_FUNC(TEXT, z_arm64_el2_plat_prep_c)
	putc	'Z'
	putc	'2'
	ret

GTEXT(z_arm64_el1_plat_prep_c)
SECTION_FUNC(TEXT, z_arm64_el1_plat_prep_c)
	putc	'Z'
	putc	'1'
	ret
