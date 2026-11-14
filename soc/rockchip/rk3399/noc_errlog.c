/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * RK3399 interconnect (Arteris FlexNoC) error loggers, TRM Part 1 8.3.2 and
 * 8.4.3: rk3399_noc_errlog_dump() prints what the NoC logged (error class,
 * opcode, address, master ID), e.g. from k_sys_fatal_error_handler() after
 * an SError caused by a bus error.
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/fatal.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_io.h>

#define ERRLOG_VLD  0x0c
#define ERRLOG_LOG0 0x14 /* [27:16] Len1, [10:8] ErrCode, [4:1] Opc, [0] Lock */
#define ERRLOG_LOG1 0x18 /* RouteID */
#define ERRLOG_LOG3 0x20 /* address */
#define ERRLOG_LOG5 0x28 /* [31:16] user, [15:10] master ID, [9:0] AXI ID */
#define ERRLOG_LOG7 0x30 /* security */

static const struct {
	const char *name;
	uintptr_t base;
} loggers[] = {
	{"slv0 (masters -> slaves)", 0xffa64000},
	{"slv1 (pmu M0 -> pmu domain)", 0xffa68080},
	{"msch0", 0xffa87c80},
	{"msch1", 0xffa8fc80},
};

/* FlexNoC ErrCode / Opc encodings */
static const char *const err_code[8] = {"SLV", "DEC", "UNS", "DISC", "SEC", "HIDE", "TMO", "?"};
static const char *const opc[16] = {"RD",  "RDW", "RDL", "RDX", "WR", "WRW", "WRC", "?7",
				    "PRE", "URG", "?a",  "?b",  "?c", "?d",  "?e",  "?f"};

void rk3399_noc_errlog_dump(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(loggers); i++) {
		uintptr_t page = ROUND_DOWN(loggers[i].base, 4096);
		mm_reg_t va;
		uint32_t l0, l5;

		device_map(&va, page, 4096, K_MEM_CACHE_NONE);
		va += loggers[i].base - page;
		if ((sys_read32(va + ERRLOG_VLD) & 1U) == 0U) {
			continue;
		}
		l0 = sys_read32(va + ERRLOG_LOG0);
		l5 = sys_read32(va + ERRLOG_LOG5);
		printk("NoC %s: %s %s addr 0x%08x len1 %u master %u axi-id 0x%x user 0x%04x "
		       "route 0x%05x sec %u\n",
		       loggers[i].name, err_code[(l0 >> 8) & 7U], opc[(l0 >> 1) & 0xfU],
		       sys_read32(va + ERRLOG_LOG3), (l0 >> 16) & 0xfffU, (l5 >> 10) & 0x3fU,
		       l5 & 0x3ffU, l5 >> 16, sys_read32(va + ERRLOG_LOG1) & 0x7ffffU,
		       sys_read32(va + ERRLOG_LOG7) & 1U);
	}
}
