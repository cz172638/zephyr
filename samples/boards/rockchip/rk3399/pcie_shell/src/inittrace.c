/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Markers between init priorities (CONFIG_SAMPLE_INIT_TRACE): the last
 * one printed brackets the device whose init does not return.
 * RK3399 drivers here: reset 35, GPIO 40, syscon 50, SDHC 70 (eMMC, SD),
 * SPI/SPI NOR 70/80, eMMC mount 76, role 81, EPC 82, EPF 83, RC 84.
 */

#include <zephyr/init.h>
#include <zephyr/sys/printk.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/sys_io.h>

/*
 * With CONFIG_RK3399_EARLY_MARKS the marker also goes straight to UART2
 * (identity mapped then), so it shows even when printk output is held back
 * (shell log backend not started yet) or the console driver misbehaves.
 */
static void raw(const char *s)
{
#if defined(CONFIG_RK3399_EARLY_MARKS)
	for (; *s != '\0'; s++) {
		while ((sys_read32(0xff1a0014UL) & 0x20U) == 0U) {
		}
		sys_write32((uint32_t)*s, 0xff1a0000UL);
	}
#else
	ARG_UNUSED(s);
#endif
}

#define MARK(level, prio)                                                                          \
	static int mark_##level##_##prio(void)                                                     \
	{                                                                                          \
		raw("\r\n[raw] init: " #level " " #prio "\r\n");                                    \
		printk("init: " #level " " #prio "\n");                                            \
		return 0;                                                                          \
	}                                                                                          \
	SYS_INIT(mark_##level##_##prio, level, prio)

MARK(PRE_KERNEL_2, 0);
MARK(POST_KERNEL, 0);
MARK(POST_KERNEL, 36);
MARK(POST_KERNEL, 41);
MARK(POST_KERNEL, 51);
MARK(POST_KERNEL, 69);
MARK(POST_KERNEL, 71);
MARK(POST_KERNEL, 75);
MARK(POST_KERNEL, 77);
MARK(POST_KERNEL, 80);
MARK(POST_KERNEL, 82);
MARK(POST_KERNEL, 83);
MARK(POST_KERNEL, 84);
MARK(POST_KERNEL, 85);
MARK(POST_KERNEL, 99);
MARK(APPLICATION, 0);
