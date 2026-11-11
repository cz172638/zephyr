/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * pcie debug ibmap: what the rest of the Aardvark inbound block does
 * (EP side). Known: 0x5800/0x5804 BAR0, 0x5808/0x580c BAR2, 0x5810/0x5814
 * BAR4 hold the 64-bit AXI base of each BAR. The entries at +0x20,
 * +0x40, +0x80, +0xa0 and the blocks at 0x5a00/0x5c00/0x5e00 look the
 * same; candidates for more windows per BAR (as an iATU in address
 * match mode), other functions, or the ROM.
 *
 * Method: tag DDR (every step bytes a word saying where it is), point a
 * candidate register at the tagged area, let the host read its BAR at a
 * few offsets and decode where the reads landed. With a Zephyr RC as the
 * host the reads go through the lab agent (devmem on the peer);
 * otherwise read on the host by hand and "decode" the values.
 *
 * Only sprintf: image 150 exports no snprintf (that would need a new image).
 *
 * Keep the tagged area inside the PCIe IO decode window (carveout):
 * a translation that leaves it ends in a completer abort on the host.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/pcie/aardvark.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/lab/lab.h>
#include <zephyr/lab/mod.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/sys_io.h>

#include <soc.h>

#define ADVK_PHYS  0xd0070000UL
#define ADVK_SIZE  0x20000
#define IB_FIRST   0x5800
#define IB_LAST    0x5ffc
#define IODEC_PHYS 0xd000c000UL
#define TAG_MAGIC  0x1ba00000U
#define TAG_MASK   0xfff00000U

static uint8_t *advk, *iodec;
static uint64_t zone, zone_size, step = 0x1000;
static uint64_t host_off; /* host CPU address = BAR bus address + host_off */

static int num(const struct shell *sh, const char *s, unsigned long long *v)
{
	char *end;

	*v = strtoull(s, &end, 0);
	if (*end != '\0') {
		shell_error(sh, "bad number '%s'", s);
		return -EINVAL;
	}
	return 0;
}

static int rd(uint32_t off, uint32_t *v)
{
	return a3700_probe_read32((uintptr_t)advk + off, v);
}

static void wr(uint32_t off, uint32_t v)
{
	sys_write32(v, (mem_addr_t)advk + off);
}

/* the window the host's inbound accesses must stay in */
static void iodec_win(uint64_t *base, uint64_t *size, bool *en)
{
	uint32_t ctrl = sys_read32((mem_addr_t)iodec + 0x500);

	*base = sys_read32((mem_addr_t)iodec + 0x504);
	*size = ((uint64_t)(ctrl >> 16) + 1) << 16;
	*en = ctrl & 1;
}

static int cmd_show(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long long from = IB_FIRST, to = IB_LAST;
	uint64_t b, s;
	bool en;

	if ((argc > 1 && num(sh, argv[1], &from) != 0) ||
	    (argc > 2 && num(sh, argv[2], &to) != 0)) {
		return -EINVAL;
	}
	for (uint32_t r = from & ~0x1fU; r <= to; r += 0x20) {
		bool any = false;
		char line[100];
		int o = sprintf(line, "%05x:", r);

		for (int i = 0; i < 8; i++) {
			uint32_t v;

			if (rd(r + 4 * i, &v) == 0) {
				o += sprintf(&line[o], " %08x", v);
				any |= v != 0;
			} else {
				o += sprintf(&line[o], " --------");
			}
		}
		if (any || argc > 1) {
			shell_print(sh, "%s", line);
		}
	}
	iodec_win(&b, &s, &en);
	shell_print(sh, "(zero rows hidden; -------- = abort)  IO decode 0: 0x%llx + 0x%llx%s",
		    (unsigned long long)b, (unsigned long long)s, en ? "" : " OFF");
	for (int bar = 0; bar <= 4; bar += 2) {
		uint32_t lo, hi;

		(void)rd(0x10 + 4 * bar, &lo);
		(void)rd(0x14 + 4 * bar, &hi);
		shell_print(sh, "BAR%d bus 0x%llx", bar,
			    (((unsigned long long)hi << 32) | lo) & ~0xfULL);
	}
	return 0;
}

/* tag <axi> <size> [step]: every step bytes: magic|index, then the address */
static int cmd_tag(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long long axi, size, st = 0x1000;
	uint64_t b, s;
	uint8_t *va;
	bool en;

	if (num(sh, argv[1], &axi) != 0 || num(sh, argv[2], &size) != 0 ||
	    (argc > 3 && num(sh, argv[3], &st) != 0) || (axi & 0xfff) || size == 0 ||
	    st < 8 || (st & 7) || size / st > 0xfffff) {
		return -EINVAL;
	}
	iodec_win(&b, &s, &en);
	if (axi < b || axi + size > b + s) {
		shell_warn(sh, "0x%llx+0x%llx is outside IO decode 0 (0x%llx+0x%llx): the host "
			   "cannot reach it", axi, size, (unsigned long long)b,
			   (unsigned long long)s);
	}
	k_mem_map_phys_bare(&va, axi, ROUND_UP(size, 0x1000), K_MEM_PERM_RW | K_MEM_CACHE_NONE);
	if (va == NULL) {
		return -ENOMEM;
	}
	for (uint64_t o = 0; o < size; o += st) {
		sys_write32(TAG_MAGIC | (uint32_t)(o / st), (mem_addr_t)va + o);
		sys_write32((uint32_t)(axi + o), (mem_addr_t)va + o + 4);
	}
	k_mem_unmap_phys_bare(va, ROUND_UP(size, 0x1000));
	zone = axi;
	zone_size = size;
	step = st;
	shell_print(sh, "tagged 0x%llx + 0x%llx, every 0x%llx bytes", axi, size, st);
	return 0;
}

static void decode(const struct shell *sh, const char *what, uint32_t v)
{
	if ((v & TAG_MASK) == TAG_MAGIC && zone_size != 0) {
		uint64_t a = zone + (uint64_t)(v & ~TAG_MASK) * step;

		shell_print(sh, "%s: %08x = tag, AXI 0x%llx (zone +0x%llx)", what, v,
			    (unsigned long long)a, (unsigned long long)(a - zone));
	} else if (v == 0xffffffff) {
		shell_print(sh, "%s: ffffffff (UR/abort or not mapped)", what);
	} else {
		shell_print(sh, "%s: %08x (not a tag)", what, v);
	}
}

static int cmd_decode(const struct shell *sh, size_t argc, char **argv)
{
	for (size_t i = 1; i < argc; i++) {
		unsigned long long v;

		if (num(sh, argv[i], &v) != 0) {
			return -EINVAL;
		}
		decode(sh, argv[i], (uint32_t)v);
	}
	return 0;
}

/* set <reg> <axi>: a lo/hi pair, hi first */
static int cmd_set(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long long reg, axi;
	uint32_t lo, hi;

	if (num(sh, argv[1], &reg) != 0 || num(sh, argv[2], &axi) != 0 || reg < IB_FIRST ||
	    reg > IB_LAST - 4 || (reg & 7)) {
		return -EINVAL;
	}
	wr(reg + 4, (uint32_t)(axi >> 32));
	wr(reg, (uint32_t)axi);
	(void)rd(reg, &lo);
	(void)rd(reg + 4, &hi);
	shell_print(sh, "%05llx: %08x %08x", reg, lo, hi);
	return 0;
}

static uint64_t bar_bus(int bar)
{
	uint32_t lo = 0, hi = 0;

	(void)rd(0x10 + 4 * bar, &lo);
	(void)rd(0x14 + 4 * bar, &hi);
	return (((uint64_t)hi << 32) | lo) & ~0xfULL;
}

/* one host read of BAR+off through the lab agent: value, or <0 */
static int64_t host_read(int bar, uint64_t off)
{
	char cmd[48], out[160];
	char *p;
	int ret;

	sprintf(cmd, "devmem 0x%llx 32",
		 (unsigned long long)(bar_bus(bar) + host_off + off));
	ret = lab_peer_exec(cmd, out, sizeof(out), K_SECONDS(2));
	if (ret != 0) {
		return ret < 0 ? ret : -EIO;
	}
	p = strstr(out, "Read value 0x");
	return p ? (int64_t)strtoull(p + 11, NULL, 16) : -EIO;
}

static int cmd_host(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long long bar, off;

	if (num(sh, argv[1], &bar) != 0 || (bar != 0 && bar != 2 && bar != 4)) {
		return -EINVAL;
	}
	for (size_t i = 2; i < argc; i++) {
		char what[32];
		int64_t v;

		if (num(sh, argv[i], &off) != 0) {
			return -EINVAL;
		}
		v = host_read((int)bar, off);
		sprintf(what, "BAR%llu+0x%llx", bar, off);
		if (v < 0) {
			shell_error(sh, "%s: peer read failed %lld", what, v);
			return (int)v;
		}
		decode(sh, what, (uint32_t)v);
	}
	return 0;
}

static int cmd_hostoff(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long long v;

	if (argc > 1) {
		if (num(sh, argv[1], &v) != 0) {
			return -EINVAL;
		}
		host_off = v;
	}
	shell_print(sh, "host CPU address = BAR bus address + 0x%llx",
		    (unsigned long long)host_off);
	return 0;
}

/*
 * hunt <from> <to> <bar> <offsets...>: for every lo/hi pair in from..to
 * (step 8, the BAR's own pair skipped): point it at the tag zone, read
 * BAR+offsets on the host, restore. A read that turns into a tag the BAR
 * pair alone would not give = that register translates.
 */
static int cmd_hunt(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long long from, to, bar;
	int64_t base[8];
	uint64_t offs[8];
	int n = 0;

	if (num(sh, argv[1], &from) != 0 || num(sh, argv[2], &to) != 0 ||
	    num(sh, argv[3], &bar) != 0 || (bar != 0 && bar != 2 && bar != 4) ||
	    (from & 7) || from < IB_FIRST || to > IB_LAST) {
		return -EINVAL;
	}
	if (zone_size == 0) {
		shell_error(sh, "tag a zone first (tag <axi> <size>)");
		return -ENOENT;
	}
	for (size_t i = 4; i < argc && n < ARRAY_SIZE(offs); i++, n++) {
		unsigned long long o;

		if (num(sh, argv[i], &o) != 0) {
			return -EINVAL;
		}
		offs[n] = o;
	}
	if (n == 0) {
		offs[n++] = 0;
	}
	/* reference: what the host reads with nothing changed */
	for (int i = 0; i < n; i++) {
		base[i] = host_read((int)bar, offs[i]);
		if (base[i] < 0) {
			shell_error(sh, "host read failed %lld (lab peer? devmem?)", base[i]);
			return (int)base[i];
		}
	}
	for (uint32_t r = from; r + 4 <= to + 4; r += 8) {
		uint32_t lo, hi;
		bool changed = false;

		if (r == IB_FIRST + 4 * bar || rd(r, &lo) != 0 || rd(r + 4, &hi) != 0) {
			continue;
		}
		wr(r + 4, (uint32_t)(zone >> 32));
		wr(r, (uint32_t)zone);
		for (int i = 0; i < n; i++) {
			int64_t v = host_read((int)bar, offs[i]);

			if (v != base[i]) {
				char what[48];

				sprintf(what, "%05x -> BAR%llu+0x%llx", r, bar,
					 (unsigned long long)offs[i]);
				if (v < 0) {
					shell_warn(sh, "%s: host read failed %lld", what, v);
				} else {
					decode(sh, what, (uint32_t)v);
				}
				changed = true;
			}
		}
		wr(r, lo);
		wr(r + 4, hi);
		if (!changed) {
			shell_print(sh, "%05x: no effect on BAR%llu", r, bar);
		}
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(ibmap_cmds,
	SHELL_CMD_ARG(show, NULL, "[from to]  Inbound block 0x5800-0x5fff, IO decode, BAR bus "
		      "addresses", cmd_show, 1, 2),
	SHELL_CMD_ARG(tag, NULL, "<axi> <size> [step]  Tag DDR (inside IO decode 0!)", cmd_tag,
		      3, 1),
	SHELL_CMD_ARG(set, NULL, "<reg> <axi>  Write a lo/hi pair", cmd_set, 3, 0),
	SHELL_CMD_ARG(host, NULL, "<bar> <off>...  Host reads BAR+off (lab peer devmem)",
		      cmd_host, 3, 6),
	SHELL_CMD_ARG(hostoff, NULL, "[delta]  host CPU address - bus address", cmd_hostoff, 1,
		      1),
	SHELL_CMD_ARG(decode, NULL, "<value>...  Where did a host read land", cmd_decode, 2, 7),
	SHELL_CMD_ARG(hunt, NULL, "<from> <to> <bar> [off...]  Try every pair against the tags",
		      cmd_hunt, 4, 7),
	SHELL_SUBCMD_SET_END);

static int ibmap_init(void)
{
	if (advk_mode_get() != ADVK_MODE_EP) {
		return -ENODEV; /* the inbound BAR block means something else in RC mode */
	}
	k_mem_map_phys_bare(&advk, ADVK_PHYS, ADVK_SIZE, K_MEM_PERM_RW | K_MEM_CACHE_NONE);
	k_mem_map_phys_bare(&iodec, IODEC_PHYS, 0x1000, K_MEM_PERM_RW | K_MEM_CACHE_NONE);
	return advk != NULL && iodec != NULL ? 0 : -ENOMEM;
}

static void ibmap_fini(void)
{
	if (advk != NULL) {
		k_mem_unmap_phys_bare(advk, ADVK_SIZE);
	}
	if (iodec != NULL) {
		k_mem_unmap_phys_bare(iodec, 0x1000);
	}
	advk = iodec = NULL;
}

LAB_DBG_CMD((pcie_debug), "pcie", ibmap, &ibmap_cmds,
	    "Aardvark inbound block: tag DDR, try registers, host reads via lab agent",
	    cmd_show, 1, 0, ibmap_init, ibmap_fini);
