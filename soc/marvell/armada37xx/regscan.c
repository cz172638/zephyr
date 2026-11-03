/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 'regs': map an internal register range and read it with the probing
 * load (an external abort becomes ????????), for mapping units nobody
 * documented: which words decode, which are non-zero. Reads only; read
 * side effects (FIFOs, read-to-clear status) are the caller's concern.
 * 'regs clk' gates a peripheral clock first: a unit with its clock off
 * may not answer at all.
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/shell/shell.h>

#include <soc.h>

#define REGS_MAX (1024 * 1024)

static int parse(const struct shell *sh, const char *s, unsigned long *v)
{
	char *end;

	*v = strtoul(s, &end, 0);
	if (*end != '\0') {
		shell_error(sh, "bad number: %s", s);
		return -EINVAL;
	}
	return 0;
}

/* pa/len from argv[1..2], mapped; caller unmaps */
static int map_range(const struct shell *sh, char **argv, uintptr_t *pa, size_t *len,
		     uint8_t **base, size_t *msize, uintptr_t *off)
{
	unsigned long a, l;
	uintptr_t page;

	if (parse(sh, argv[1], &a) != 0 || parse(sh, argv[2], &l) != 0) {
		return -EINVAL;
	}
	if (l == 0 || l > REGS_MAX) {
		shell_error(sh, "length 4..0x%x", REGS_MAX);
		return -EINVAL;
	}
	a &= ~3UL;
	page = a & ~(uintptr_t)(CONFIG_MMU_PAGE_SIZE - 1);
	*off = a - page;
	*msize = ROUND_UP(*off + l, CONFIG_MMU_PAGE_SIZE);
	k_mem_map_phys_bare(base, page, *msize, K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	*pa = a;
	*len = l;
	return 0;
}

static int cmd_dump(const struct shell *sh, size_t argc, char **argv)
{
	uintptr_t pa, off;
	size_t len, msize;
	uint8_t *base;
	char line[64];

	ARG_UNUSED(argc);
	if (map_range(sh, argv, &pa, &len, &base, &msize, &off) != 0) {
		return -EINVAL;
	}
	for (size_t o = 0; o < len; o += 16) {
		int n = 0;

		for (size_t i = o; i < o + 16 && i < len; i += 4) {
			uint32_t v;

			if (a3700_probe_read32((uintptr_t)(base + off + i), &v) == 0) {
				n += snprintf(line + n, sizeof(line) - n, " %08x", v);
			} else {
				n += snprintf(line + n, sizeof(line) - n, " ????????");
			}
		}
		shell_print(sh, "%08lx:%s", (unsigned long)(pa + o), line);
	}
	k_mem_unmap_phys_bare(base, msize);
	return 0;
}

static int cmd_nz(const struct shell *sh, size_t argc, char **argv)
{
	uintptr_t pa, off;
	size_t len, msize;
	uint8_t *base;
	uint32_t nz = 0, ab = 0;

	ARG_UNUSED(argc);
	if (map_range(sh, argv, &pa, &len, &base, &msize, &off) != 0) {
		return -EINVAL;
	}
	for (size_t o = 0; o < len; o += 4) {
		uint32_t v;

		if (a3700_probe_read32((uintptr_t)(base + off + o), &v) != 0) {
			ab++;
		} else if (v != 0) {
			shell_print(sh, "%08lx: %08x", (unsigned long)(pa + o), v);
			nz++;
		}
	}
	shell_print(sh, "%u non-zero, %u aborted of %zu words", nz, ab, len / 4);
	k_mem_unmap_phys_bare(base, msize);
	return 0;
}

/* Runs of words that decode / abort: the layout of a unit at a glance */
static int cmd_scan(const struct shell *sh, size_t argc, char **argv)
{
	uintptr_t pa, off;
	size_t len, msize;
	uint8_t *base;
	size_t start = 0;
	int state = -1; /* 0 aborts, 1 decodes */
	uint32_t nz = 0;

	ARG_UNUSED(argc);
	if (map_range(sh, argv, &pa, &len, &base, &msize, &off) != 0) {
		return -EINVAL;
	}
	for (size_t o = 0; o <= len; o += 4) {
		uint32_t v = 0;
		int s = -2;

		if (o < len) {
			s = a3700_probe_read32((uintptr_t)(base + off + o), &v) == 0 ? 1 : 0;
		}
		if (s != state) {
			if (state == 1) {
				shell_print(sh, "%08lx-%08lx  decodes, %u non-zero",
					    (unsigned long)(pa + start),
					    (unsigned long)(pa + o - 1), nz);
			} else if (state == 0) {
				shell_print(sh, "%08lx-%08lx  aborts", (unsigned long)(pa + start),
					    (unsigned long)(pa + o - 1));
			}
			state = s;
			start = o;
			nz = 0;
		}
		if (s == 1 && v != 0) {
			nz++;
		}
	}
	k_mem_unmap_phys_bare(base, msize);
	return 0;
}

static int cmd_clk(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *nb = DEVICE_DT_GET(DT_NODELABEL(nb_periph_clk));
	const struct device *sb = DEVICE_DT_GET(DT_NODELABEL(sb_periph_clk));
	const struct device *d;
	unsigned long idx;
	int ret;

	ARG_UNUSED(argc);
	if (strcmp(argv[1], "nb") == 0) {
		d = nb;
	} else if (strcmp(argv[1], "sb") == 0) {
		d = sb;
	} else {
		shell_error(sh, "nb|sb");
		return -EINVAL;
	}
	if (parse(sh, argv[2], &idx) != 0) {
		return -EINVAL;
	}
	if (strcmp(argv[3], "on") == 0) {
		ret = clock_control_on(d, (clock_control_subsys_t)idx);
	} else if (strcmp(argv[3], "off") == 0) {
		ret = clock_control_off(d, (clock_control_subsys_t)idx);
	} else {
		shell_error(sh, "on|off");
		return -EINVAL;
	}
	shell_print(sh, "%s clock %lu %s: %d", argv[1], idx, argv[3], ret);
	return ret;
}

/*
 * Snapshot and diff: what a mode switch (USB host/device, PCIe role,
 * a clock) changes in a unit. Aborting words are kept as a separate
 * state, so a word that starts or stops decoding shows up as well.
 */
#define SNAP_WORDS (64 * 1024 / 4)
#define SNAP_ABORT 0xdeadabedU

static uint32_t snap_val[SNAP_WORDS];
static uint8_t snap_ab[SNAP_WORDS / 8];
static uintptr_t snap_pa;
static size_t snap_len;

static int cmd_snap(const struct shell *sh, size_t argc, char **argv)
{
	uintptr_t pa, off;
	size_t len, msize;
	uint8_t *base;
	uint32_t ab = 0;

	ARG_UNUSED(argc);
	if (map_range(sh, argv, &pa, &len, &base, &msize, &off) != 0) {
		return -EINVAL;
	}
	if (len > SNAP_WORDS * 4) {
		shell_error(sh, "snapshot at most 0x%x bytes", SNAP_WORDS * 4);
		k_mem_unmap_phys_bare(base, msize);
		return -EINVAL;
	}
	memset(snap_ab, 0, sizeof(snap_ab));
	for (size_t i = 0; i < len / 4; i++) {
		if (a3700_probe_read32((uintptr_t)(base + off + i * 4), &snap_val[i]) != 0) {
			snap_ab[i / 8] |= BIT(i % 8);
			snap_val[i] = SNAP_ABORT;
			ab++;
		}
	}
	snap_pa = pa;
	snap_len = len & ~3U;
	k_mem_unmap_phys_bare(base, msize);
	shell_print(sh, "snapshot %08lx + 0x%zx: %zu words, %u aborted", (unsigned long)pa, snap_len,
		    snap_len / 4, ab);
	return 0;
}

static int cmd_diff(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t *base;
	uintptr_t page, off;
	size_t msize;
	uint32_t n = 0;
	bool update = argc > 1 && strcmp(argv[1], "update") == 0;

	if (snap_len == 0) {
		shell_error(sh, "no snapshot (regs snap <pa> <len>)");
		return -ENOENT;
	}
	page = snap_pa & ~(uintptr_t)(CONFIG_MMU_PAGE_SIZE - 1);
	off = snap_pa - page;
	msize = ROUND_UP(off + snap_len, CONFIG_MMU_PAGE_SIZE);
	k_mem_map_phys_bare(&base, page, msize, K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	for (size_t i = 0; i < snap_len / 4; i++) {
		uint32_t v;
		bool was_ab = (snap_ab[i / 8] & BIT(i % 8)) != 0;
		bool is_ab = a3700_probe_read32((uintptr_t)(base + off + i * 4), &v) != 0;

		if (was_ab == is_ab && (is_ab || v == snap_val[i])) {
			continue;
		}
		n++;
		if (was_ab || is_ab) {
			shell_print(sh, "%08lx: %s -> %s", (unsigned long)(snap_pa + i * 4),
				    was_ab ? "abort" : "value", is_ab ? "abort" : "value");
			if (!is_ab) {
				shell_print(sh, "          now %08x", v);
			}
		} else {
			shell_print(sh, "%08lx: %08x -> %08x  (set %08x clr %08x)",
				    (unsigned long)(snap_pa + i * 4), snap_val[i], v,
				    v & ~snap_val[i], snap_val[i] & ~v);
		}
		if (update) {
			snap_val[i] = is_ab ? SNAP_ABORT : v;
			snap_ab[i / 8] = is_ab ? (snap_ab[i / 8] | BIT(i % 8))
					       : (snap_ab[i / 8] & ~BIT(i % 8));
		}
	}
	k_mem_unmap_phys_bare(base, msize);
	shell_print(sh, "%u words differ%s", n, update ? " (snapshot updated)" : "");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(regs_cmds,
	SHELL_CMD_ARG(snap, NULL, "<pa> <len> Snapshot a range (at most 64 KiB)", cmd_snap, 3, 0),
	SHELL_CMD_ARG(diff, NULL, "[update] Words changed since the snapshot", cmd_diff, 1, 1),
	SHELL_CMD_ARG(scan, NULL, "<pa> <len> Runs of decoding / aborting words", cmd_scan, 3, 0),
	SHELL_CMD_ARG(nz, NULL, "<pa> <len> Non-zero words", cmd_nz, 3, 0),
	SHELL_CMD_ARG(dump, NULL, "<pa> <len> Dump, ???????? where the read aborts", cmd_dump, 3,
		      0),
	SHELL_CMD_ARG(clk, NULL,
		      "nb|sb <index> on|off Peripheral clock gate (Linux indices; sb 11 usb32_usb2, 12 usb32_ss, 13 pcie, "
		      "nb 1 sata)", cmd_clk, 4, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(regs, &regs_cmds, "Armada 3700 register ranges (abort-safe reads)", NULL);
