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
 *   g  POST_KERNEL 0 (first thing in the main thread), followed by
 *      R/N  console UART device ready / not ready
 *      H    uart_poll_out() through the driver (its own mapping)
 *      "[printk]" through printk, then i
 *   z  APPLICATION 99   after the boot banner
 *   POST_KERNEL 1: GIC cpu interface, virtual timer, distributor and own
 *      redistributor registers, then a 5 ms k_timer (prints T from its
 *      expiry) under k_busy_wait(20 ms), the same dump again, then
 *   S..s  POST_KERNEL 1: around k_msleep(10), i.e. whether the system
 *      timer interrupt arrives (the first sleep of a boot hangs if not)
 */

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
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
	const struct device *con = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	mark('g');
	if (device_is_ready(con)) {
		mark('R');
		uart_poll_out(con, 'H');
	} else {
		mark('N');
	}
	printk("[printk]");
	mark('i');
	mark('\r');
	mark('\n');
	return 0;
}
SYS_INIT(mark_post, POST_KERNEL, 0);

static void hex(const char *name, uint64_t v)
{
	static const char digits[] = "0123456789abcdef";

	while (*name != 0) {
		mark(*name++);
	}
	mark('=');
	for (int i = 60; i >= 0; i -= 4) {
		if ((v >> i) != 0U || i == 0) {
			mark(digits[(v >> i) & 0xfU]);
		}
	}
	mark(' ');
}

#define SYSREG(r) hex(#r, read_sysreg(r))

#define GICD     0xfee00000UL
#define GICR_SGI(rd) ((rd) + 0x10000UL)

static void gic_dump(void)
{
	uintptr_t rd = 0;

	SYSREG(CurrentEL);
	SYSREG(daif);
	SYSREG(ICC_SRE_EL1);
	SYSREG(ICC_PMR_EL1);
	SYSREG(ICC_IGRPEN1_EL1);
	SYSREG(ICC_CTLR_EL1);
	SYSREG(ICC_RPR_EL1);
	SYSREG(ICC_HPPIR1_EL1);
	mark('\r');
	mark('\n');
	SYSREG(cntfrq_el0);
	SYSREG(cntv_ctl_el0);
	SYSREG(cntv_cval_el0);
	SYSREG(cntvct_el0);
	SYSREG(mpidr_el1);
	mark('\r');
	mark('\n');
	hex("GICD_CTLR", sys_read32(GICD));
	hex("TYPER", sys_read32(GICD + 4));
	for (int i = 0; i < 6; i++) {
		uintptr_t f = 0xfef00000UL + i * 0x20000UL;
		uint64_t t = sys_read64(f + 8);

		if (rd == 0 && (t >> 32) == ((read_sysreg(mpidr_el1) & 0xffffffU) |
					     ((read_sysreg(mpidr_el1) >> 8) & 0xff000000U))) {
			rd = f;
		}
	}
	hex("RD", rd);
	if (rd != 0) {
		hex("CTLR", sys_read32(rd));
		hex("WAKER", sys_read32(rd + 0x14));
		hex("TYPER", sys_read64(rd + 8));
		mark('\r');
		mark('\n');
		hex("IGROUPR0", sys_read32(GICR_SGI(rd) + 0x80));
		hex("ISENABLER0", sys_read32(GICR_SGI(rd) + 0x100));
		hex("ISPENDR0", sys_read32(GICR_SGI(rd) + 0x200));
		hex("ISACTIVER0", sys_read32(GICR_SGI(rd) + 0x300));
		hex("PRIO27", sys_read8(GICR_SGI(rd) + 0x400 + 27));
		hex("ICFGR1", sys_read32(GICR_SGI(rd) + 0xc04));
		hex("IGRPMODR0", sys_read32(GICR_SGI(rd) + 0xd00));
	}
	mark('\r');
	mark('\n');
}

static void tick(struct k_timer *t)
{
	ARG_UNUSED(t);
	mark('T');
}
K_TIMER_DEFINE(probe_timer, tick, NULL);

static int mark_sleep(void)
{
	mark('\r');
	mark('\n');
	gic_dump();
	k_timer_start(&probe_timer, K_MSEC(5), K_NO_WAIT);
	k_busy_wait(20000);
	gic_dump();
	mark('S');
	k_msleep(10);
	mark('s');
	mark('\r');
	mark('\n');
	return 0;
}
SYS_INIT(mark_sleep, POST_KERNEL, 1);

static int mark_app(void)
{
	mark('z');
	mark('\r');
	mark('\n');
	return 0;
}
SYS_INIT(mark_app, APPLICATION, 99);
