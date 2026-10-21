/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Entry for the BootROM's CRV2 core release (TIM EXEC_ADDR). The Marvell
 * WTMI points EXEC_ADDR at a table whose first word is the reset handler,
 * which works both if the BootROM loads the word and jumps to it and if
 * it jumps to EXEC_ADDR itself: the words are 0x1fffxxxx | 1 with xxxx <
 * 0x4000, which decode as harmless Thumb ALU instructions (LSLS/LSRS/ADDS
 * ..., then SUBS r7, r7, #7). Do the same: the pointer, then a real jump.
 */

#include <zephyr/toolchain.h>

	.syntax unified
	.thumb
	.section .a3700_cm3_entry, "ax"
	.align 2
	.global a3700_cm3_entry
a3700_cm3_entry:
	.word z_arm_reset
	ldr r0, 1f
	bx r0
	.align 2
1:	.word z_arm_reset
