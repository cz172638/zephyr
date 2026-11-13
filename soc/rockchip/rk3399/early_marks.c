/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * CONFIG_RK3399_EARLY_MARKS, C part: "P" in soc_prep_hook (MMU still off),
 * "M" at EARLY (MMU on, UART2 identity mapped by mmu_regions.c), "K" at
 * PRE_KERNEL_1 0 (before the console driver). A boot that prints
 * "Z2PMK" and then nothing hangs in a PRE_KERNEL driver.
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
	mark('\r');
	mark('\n');
	return 0;
}
SYS_INIT(mark_pre1, PRE_KERNEL_1, 0);
