/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * CONFIG_RK3399_EARLY_MARKS, C part: "P" in soc_prep_hook (MMU still off),
 * "M" at EARLY (MMU on, UART2 identity mapped by mmu_regions.c), "K" at
 * PRE_KERNEL_1 0, then one letter after each band of PRE_KERNEL drivers:
 *   a  PRE_KERNEL_1 31   after the CRU clocks (30)
 *   b  PRE_KERNEL_1 36   after the CRU resets (35)
 *   c  PRE_KERNEL_1 41   after the GIC (40)
 *   d  PRE_KERNEL_1 51   after syscons and the console UART (50)
 *   e  PRE_KERNEL_2 0
 *   f  PRE_KERNEL_2 99   after the system timer
 *   g  POST_KERNEL 0
 */

#include <stdint.h>

#include <zephyr/init.h>
#include <zephyr/platform/hooks.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/sys_io.h>

#define RK3399_UART2 0xff1a0000UL

static void mark(char c)
{
	while ((sys_read32(RK3399_UART2 + 0x14) & 0x20U) == 0U) {
	}
	sys_write32((uint32_t)c, RK3399_UART2);
}

void soc_prep_hook(void)
{
	mark('P');
}

static int mark_early(void)
{
	mark('M');
	return 0;
}
SYS_INIT(mark_early, EARLY, 0);

static int mark_pre1(void)
{
	mark('K');
	return 0;
}
SYS_INIT(mark_pre1, PRE_KERNEL_1, 0);

#define BAND(c, level, prio)                                                                       \
	static int mark_##level##_##prio(void)                                                     \
	{                                                                                          \
		mark(c);                                                                           \
		return 0;                                                                          \
	}                                                                                          \
	SYS_INIT(mark_##level##_##prio, level, prio)

BAND('a', PRE_KERNEL_1, 31);
BAND('b', PRE_KERNEL_1, 36);
BAND('c', PRE_KERNEL_1, 41);
BAND('d', PRE_KERNEL_1, 51);
BAND('e', PRE_KERNEL_2, 0);
BAND('f', PRE_KERNEL_2, 99);

static int mark_post(void)
{
	mark('g');
	mark('\r');
	mark('\n');
	return 0;
}
SYS_INIT(mark_post, POST_KERNEL, 0);
