/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * dbg mbusx: the Armada 3720 MBus decode windows, for using the EP as a
 * non-transparent bridge the way the Kirkwood spec suggests (6.20, and
 * 2.4: SATA DMA may target SDRAM *and* PCIe).
 *
 * - IO decode windows of every DMA master (TF-A io_addr_dec.c,
 *   marvell_plat_config.c): ctrl (+0: size/64K-1 [31:16], attr [15:8],
 *   target [7:4], enable), base (+4). Targets: 0 DRAM, 1 internal
 *   registers, 2 PCIe, ...
 * - CPU decode windows 0xd000cf00 (TF-A dram_win.c): ctrl (target,
 *   enable), size, base, remap; 0x10 apart.
 * - Aardvark outbound windows 0x4c00 (Linux OB_WIN_*).
 *
 * sata2host: the AHCI controller reads sectors straight into host
 * memory. Its IO decode window points an AXI range at the PCIe unit
 * (target 2), an outbound window maps that range to a host bus address,
 * the command goes into AHCI slot 1 next to the driver's slot 0 (only
 * while the driver is idle). The same sector is read into local DRAM
 * for comparison; with a Zephyr RC as the host, the lab agent reads the
 * host side back (devmem).
 *
 * Everything this module changes in the windows is restored by "restore"
 * and on unload.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/lab/lab.h>
#include <zephyr/lab/mod.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/sys_io.h>

#include <soc.h>

#define DEC_PHYS   0xd000c000UL
#define ADVK_PHYS  0xd0070000UL
#define AHCI_PHYS  0xd00e0000UL
#define OB_WIN(w, o) (0x4c00 + 0x20 * (w) + (o))
#define OB_COUNT   8

struct master {
	const char *name;
	uint16_t base;
	uint8_t wins;
	uint8_t stride;
};

/* TF-A marvell_plat_config.c io_dec_win_conf */
static const struct master masters[] = {
	{"usb", 0xc000, 2, 0x08},   {"usb3", 0xc100, 3, 0x10}, {"dma", 0xc200, 2, 0x10},
	{"neta0", 0xc300, 2, 0x10}, {"neta1", 0xc400, 2, 0x10}, {"pcie", 0xc500, 2, 0x10},
	{"sata", 0xc800, 3, 0x10},  {"sd", 0xca00, 3, 0x08},    {"emmc", 0xcb00, 3, 0x10},
	{"eip97", 0xce00, 2, 0x08},
};

/* saved originals: io decode (master, win) and OB windows */
#define SAVES 16
static struct {
	uint32_t reg; /* offset in its block, 0 = unused */
	bool ob;
	uint32_t val[7];
} saves[SAVES];

static uint8_t *dec, *advk, *ahci;

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

static uint32_t drd(uint32_t off)
{
	return sys_read32((mem_addr_t)dec + off);
}

static void dwr(uint32_t off, uint32_t v)
{
	sys_write32(v, (mem_addr_t)dec + off);
}

static uint32_t ard(uint32_t off)
{
	return sys_read32((mem_addr_t)advk + off);
}

static void awr(uint32_t off, uint32_t v)
{
	sys_write32(v, (mem_addr_t)advk + off);
}

static void save(uint32_t reg, bool ob)
{
	int free_slot = -1;

	for (int i = 0; i < SAVES; i++) {
		if (saves[i].reg == reg && saves[i].ob == ob) {
			return; /* keep the first original */
		}
		if (saves[i].reg == 0 && free_slot < 0) {
			free_slot = i;
		}
	}
	if (free_slot < 0) {
		return;
	}
	saves[free_slot].reg = reg;
	saves[free_slot].ob = ob;
	for (int i = 0; i < (ob ? 7 : 2); i++) {
		saves[free_slot].val[i] = ob ? ard(reg + 4 * i) : drd(reg + 4 * i);
	}
}

static int restore_all(void)
{
	int n = 0;

	for (int i = 0; i < SAVES; i++) {
		if (saves[i].reg == 0) {
			continue;
		}
		if (saves[i].ob) {
			awr(saves[i].reg, 0); /* disable while changing */
			for (int j = 6; j >= 0; j--) {
				awr(saves[i].reg + 4 * j, saves[i].val[j]);
			}
		} else {
			dwr(saves[i].reg, saves[i].val[0] & ~1U);
			dwr(saves[i].reg + 4, saves[i].val[1]);
			dwr(saves[i].reg, saves[i].val[0]);
		}
		saves[i].reg = 0;
		n++;
	}
	return n;
}

static const struct master *master_get(const struct shell *sh, const char *name)
{
	for (int i = 0; i < ARRAY_SIZE(masters); i++) {
		if (strcmp(masters[i].name, name) == 0) {
			return &masters[i];
		}
	}
	shell_error(sh, "masters: usb usb3 dma neta0 neta1 pcie sata sd emmc eip97");
	return NULL;
}

static void show_master(const struct shell *sh, const struct master *m)
{
	for (int w = 0; w < m->wins; w++) {
		uint32_t reg = m->base + w * m->stride;
		uint32_t c = drd(reg), b = drd(reg + 4);

		shell_print(sh, "%-6s win%d %04x: %s target %u attr 0x%02x base 0x%08x size 0x%x",
			    m->name, w, reg, (c & 1) ? "on " : "off", (c >> 4) & 0xf, (c >> 8) & 0xff,
			    b & 0xffff0000U, (((c >> 16) & 0xffff) + 1) << 16);
	}
}

static int cmd_show(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1) {
		const struct master *m = master_get(sh, argv[1]);

		if (m == NULL) {
			return -EINVAL;
		}
		show_master(sh, m);
		return 0;
	}
	for (int i = 0; i < ARRAY_SIZE(masters); i++) {
		show_master(sh, &masters[i]);
	}
	for (int w = 0; w < 5; w++) {
		uint32_t r = 0xf00 + 0x10 * w;
		uint32_t c = drd(r);

		shell_print(sh, "cpu    win%d %04x: %s target %u base 0x%08x size 0x%x remap 0x%08x", w,
			    0xc000 + r, (c & 1) ? "on " : "off", (c >> 4) & 0xf,
			    (drd(r + 8) & 0xffff) << 16, ((drd(r + 4) & 0xffff) + 1) << 16,
			    (drd(r + 0xc) & 0xffff) << 16);
	}
	for (int w = 0; w < OB_COUNT; w++) {
		uint32_t m = ard(OB_WIN(w, 0));

		shell_print(sh, "ob     win%d: %s match 0x%08x%08x remap 0x%08x%08x mask 0x%08x%08x "
			    "act %08x", w, (m & 1) ? "on " : "off", ard(OB_WIN(w, 4)), m & ~1U,
			    ard(OB_WIN(w, 0xc)), ard(OB_WIN(w, 8)), ard(OB_WIN(w, 0x14)),
			    ard(OB_WIN(w, 0x10)), ard(OB_WIN(w, 0x18)));
	}
	return 0;
}

static int iodec_set(const struct master *m, int w, uint32_t target, uint32_t attr,
		     uint32_t base, uint32_t size)
{
	uint32_t reg = m->base + w * m->stride;

	save(reg, false);
	dwr(reg, 0);
	dwr(reg + 4, base & 0xffff0000U);
	dwr(reg, ((size >> 16) - 1) << 16 | (attr & 0xff) << 8 | (target & 0xf) << 4 | 1);
	return 0;
}

/* set <master> <win> <target> <attr> <base> <size> */
static int cmd_set(const struct shell *sh, size_t argc, char **argv)
{
	const struct master *m = master_get(sh, argv[1]);
	unsigned long long w, t, a, b, s;

	if (m == NULL || num(sh, argv[2], &w) != 0 || num(sh, argv[3], &t) != 0 ||
	    num(sh, argv[4], &a) != 0 || num(sh, argv[5], &b) != 0 || num(sh, argv[6], &s) != 0 ||
	    w >= m->wins || t > 15 || a > 255 || s < 0x10000 || (s & (s - 1)) || (b & (s - 1))) {
		shell_error(sh, "win < %d, size 2^n >= 64K, base aligned to size", m ? m->wins : 0);
		return -EINVAL;
	}
	iodec_set(m, (int)w, t, a, b, s);
	show_master(sh, m);
	return 0;
}

static int cmd_off(const struct shell *sh, size_t argc, char **argv)
{
	const struct master *m = master_get(sh, argv[1]);
	unsigned long long w;
	uint32_t reg;

	if (m == NULL || num(sh, argv[2], &w) != 0 || w >= m->wins) {
		return -EINVAL;
	}
	reg = m->base + w * m->stride;
	save(reg, false);
	dwr(reg, drd(reg) & ~1U);
	show_master(sh, m);
	return 0;
}

static void ob_set(int w, uint64_t axi, uint64_t host, uint64_t size)
{
	uint64_t mask = ~(size - 1);

	save(OB_WIN(w, 0), true);
	awr(OB_WIN(w, 0), 0);
	awr(OB_WIN(w, 0x18), 0); /* memory, no attributes */
	awr(OB_WIN(w, 0x14), (uint32_t)(mask >> 32));
	awr(OB_WIN(w, 0x10), (uint32_t)mask);
	awr(OB_WIN(w, 0x0c), (uint32_t)(host >> 32));
	awr(OB_WIN(w, 0x08), (uint32_t)host);
	awr(OB_WIN(w, 0x04), (uint32_t)(axi >> 32));
	awr(OB_WIN(w, 0x00), (uint32_t)axi | 1);
}

static int ob_free(void)
{
	for (int w = OB_COUNT - 1; w >= 0; w--) {
		if (!(ard(OB_WIN(w, 0)) & 1)) {
			return w;
		}
	}
	return -1;
}

/* ob <win|free> <axi> <host bus> <size> */
static int cmd_ob(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long long w, axi, host, size;

	if (strcmp(argv[1], "free") == 0) {
		w = ob_free();
	} else if (num(sh, argv[1], &w) != 0) {
		return -EINVAL;
	}
	if (w >= OB_COUNT || num(sh, argv[2], &axi) != 0 || num(sh, argv[3], &host) != 0 ||
	    num(sh, argv[4], &size) != 0 || size < 0x10000 || (size & (size - 1)) ||
	    (axi & (size - 1)) || (host & (size - 1))) {
		shell_error(sh, "free window? size 2^n >= 64K, both addresses aligned");
		return -EINVAL;
	}
	ob_set((int)w, axi, host, size);
	shell_print(sh, "ob win%llu: axi 0x%llx -> host bus 0x%llx, 0x%llx", w, axi, host, size);
	return 0;
}

static int cmd_restore(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "%d windows restored", restore_all());
	return 0;
}

/* ---- AHCI slot 1 ---- */

#define P0(o)       (0x100 + (o))
#define PX_CLB      P0(0x00)
#define PX_IS       P0(0x10)
#define PX_CMD      P0(0x18)
#define PX_TFD      P0(0x20)
#define PX_SERR     P0(0x30)
#define PX_SACT     P0(0x34)
#define PX_CI       P0(0x38)
#define SLOT        1

struct ahci_tbl {
	uint8_t cfis[64];
	uint8_t acmd[16];
	uint8_t rsvd[48];
	struct {
		uint32_t dba, dbau, rsvd, dbc;
	} prd[1];
};

static uint32_t hrd(uint32_t off)
{
	return sys_read32((mem_addr_t)ahci + off);
}

static void hwr(uint32_t off, uint32_t v)
{
	sys_write32(v, (mem_addr_t)ahci + off);
}

/* READ DMA EXT of count sectors to the AXI address dst (as SATA sees it) */
static int ahci_read_to(const struct shell *sh, uint64_t dst, uint64_t lba, uint32_t count,
			uint8_t pmp, uint32_t *prdbc)
{
	uint64_t clb = ((uint64_t)hrd(PX_CLB + 4) << 32) | hrd(PX_CLB);
	uint8_t *clbv;
	volatile uint32_t *hdr;
	struct ahci_tbl *tbl;
	uint64_t tp;
	int64_t end;
	int ret = 0;

	if (!(hrd(PX_CMD) & BIT(0)) || ((hrd(0) >> 8) & 0x1f) < SLOT) {
		shell_error(sh, "AHCI port 0 not started, or no slot %d", SLOT);
		return -ENODEV;
	}
	if (hrd(PX_CI) != 0 || hrd(PX_SACT) != 0) {
		shell_error(sh, "AHCI busy (CI %08x SACT %08x): the driver is using it", hrd(PX_CI),
			    hrd(PX_SACT));
		return -EBUSY;
	}
	tbl = k_aligned_alloc(128, sizeof(*tbl));
	if (tbl == NULL) {
		return -ENOMEM;
	}
	k_mem_map_phys_bare(&clbv, clb & ~0xfffULL, 0x1000, K_MEM_PERM_RW | K_MEM_CACHE_NONE);
	if (clbv == NULL) {
		k_free(tbl);
		return -ENOMEM;
	}
	hdr = (volatile uint32_t *)(clbv + (clb & 0xfff) + 32 * SLOT);

	memset(tbl, 0, sizeof(*tbl));
	tbl->cfis[0] = 0x27;                 /* H2D register FIS */
	tbl->cfis[1] = 0x80 | (pmp & 0xf);   /* command */
	tbl->cfis[2] = 0x25;                 /* READ DMA EXT */
	tbl->cfis[4] = (uint8_t)lba;
	tbl->cfis[5] = (uint8_t)(lba >> 8);
	tbl->cfis[6] = (uint8_t)(lba >> 16);
	tbl->cfis[7] = 0x40;                 /* LBA */
	tbl->cfis[8] = (uint8_t)(lba >> 24);
	tbl->cfis[9] = (uint8_t)(lba >> 32);
	tbl->cfis[10] = (uint8_t)(lba >> 40);
	tbl->cfis[12] = (uint8_t)count;
	tbl->cfis[13] = (uint8_t)(count >> 8);
	tbl->prd[0].dba = (uint32_t)dst;
	tbl->prd[0].dbau = (uint32_t)(dst >> 32);
	tbl->prd[0].dbc = count * 512 - 1;
	sys_cache_data_flush_range(tbl, sizeof(*tbl));
	tp = k_mem_phys_addr(tbl);

	hdr[1] = 0;
	hdr[2] = (uint32_t)tp;
	hdr[3] = (uint32_t)(tp >> 32);
	hdr[0] = 5 | ((uint32_t)(pmp & 0xf) << 12) | (1U << 16); /* CFL 5, PRDTL 1, read */
	barrier_dsync_fence_full();
	hwr(PX_CI, BIT(SLOT));

	end = k_uptime_get() + 3000;
	while (hrd(PX_CI) & BIT(SLOT)) {
		if ((hrd(PX_IS) & 0x7d800000U) || (hrd(PX_TFD) & 1)) {
			ret = -EIO; /* error bits / task file error */
			break;
		}
		if (k_uptime_get() > end) {
			ret = -ETIMEDOUT;
			break;
		}
	}
	*prdbc = hdr[1];
	if (ret != 0) {
		shell_error(sh, "slot %d: %d, IS %08x TFD %08x SERR %08x CI %08x", SLOT, ret,
			    hrd(PX_IS), hrd(PX_TFD), hrd(PX_SERR), hrd(PX_CI));
	}
	/* our D2H FIS / errors are not news for the driver */
	hwr(PX_IS, hrd(PX_IS));
	k_mem_unmap_phys_bare(clbv, 0x1000);
	k_free(tbl);
	return ret;
}

/*
 * sata2host <host bus> <lba> [count] [pmp] [axi]: AHCI reads count
 * sectors (default 8) straight to the host
 */
static int cmd_sata2host(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long long host, lba, count = 8, pmp = 0, axi = 0xef000000ULL;
	const struct master *sata = &masters[6];
	uint32_t *local, prdbc = 0;
	int win = -1, ob, ret;
	int64_t t0;

	if (num(sh, argv[1], &host) != 0 || num(sh, argv[2], &lba) != 0 ||
	    (argc > 3 && num(sh, argv[3], &count) != 0) ||
	    (argc > 4 && num(sh, argv[4], &pmp) != 0) ||
	    (argc > 5 && num(sh, argv[5], &axi) != 0) || count == 0 || count > 2048 ||
	    (host & 0xfffff) || (axi & 0xfffff)) {
		shell_error(sh, "host bus and axi 1 MiB aligned, count 1..2048");
		return -EINVAL;
	}
	/* a free SATA IO decode window and a free outbound window */
	for (int w = sata->wins - 1; w >= 0; w--) {
		if (!(drd(sata->base + w * sata->stride) & 1)) {
			win = w;
			break;
		}
	}
	ob = ob_free();
	if (win < 0 || ob < 0) {
		shell_error(sh, "no free SATA decode window (%d) or outbound window (%d)", win, ob);
		return -EBUSY;
	}

	/* 1: the first sector into local DRAM, the usual way */
	local = k_aligned_alloc(512, 512);
	if (local == NULL) {
		return -ENOMEM;
	}
	sys_cache_data_flush_and_invd_range(local, 512);
	ret = ahci_read_to(sh, k_mem_phys_addr(local), lba, 1, (uint8_t)pmp, &prdbc);
	sys_cache_data_invd_range(local, 512);
	if (ret != 0) {
		k_free(local);
		return ret;
	}
	shell_print(sh, "local  lba %llu: %08x %08x %08x %08x", lba, local[0], local[1], local[2],
		    local[3]);

	/* 2: SATA AXI range -> PCIe unit -> outbound -> host */
	iodec_set(sata, win, 2, 0x00, (uint32_t)axi, 0x100000);
	ob_set(ob, axi, host, 0x100000);
	t0 = k_uptime_get();
	ret = ahci_read_to(sh, axi, lba, (uint32_t)count, (uint8_t)pmp, &prdbc);
	shell_print(sh, "host   %llu sectors -> bus 0x%llx (sata win%d target 2, ob win%d): %d, "
		    "PRDBC %u, %lld ms", count, host, win, ob, ret, prdbc, k_uptime_get() - t0);
	restore_all();

	/* 3: what the host has now (Zephyr RC through the lab agent) */
	if (ret == 0) {
		for (int i = 0; i < 4; i++) {
			char cmd[40], out[96];
			char *p;

			sprintf(cmd, "devmem 0x%llx 32", host + 4 * i);
			if (lab_peer_exec(cmd, out, sizeof(out), K_SECONDS(2)) != 0 ||
			    (p = strstr(out, "Read value 0x")) == NULL) {
				shell_print(sh, "(host check: no lab peer / devmem; compare by hand)");
				break;
			}
			uint32_t v = (uint32_t)strtoull(p + 11, NULL, 16);

			shell_print(sh, "host  +%d: %08x %s", 4 * i, v,
				    v == local[i] ? "= local" : "!= local");
		}
	}
	k_free(local);
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(mbusx_cmds,
	SHELL_CMD_ARG(set, NULL, "<master> <win> <target> <attr> <base> <size>  IO decode window",
		      cmd_set, 7, 0),
	SHELL_CMD_ARG(off, NULL, "<master> <win>", cmd_off, 3, 0),
	SHELL_CMD_ARG(ob, NULL, "<win|free> <axi> <host bus> <size>  Aardvark outbound window",
		      cmd_ob, 5, 0),
	SHELL_CMD_ARG(restore, NULL, "Everything changed back", cmd_restore, 1, 0),
	SHELL_CMD_ARG(sata2host, NULL, "<host bus> <lba> [count] [pmp] [axi]  AHCI DMA straight "
		      "into host memory", cmd_sata2host, 3, 3),
	SHELL_SUBCMD_SET_END);

static int mbusx_init(void)
{
	k_mem_map_phys_bare(&dec, DEC_PHYS, 0x1000, K_MEM_PERM_RW | K_MEM_CACHE_NONE);
	k_mem_map_phys_bare(&advk, ADVK_PHYS, 0x10000, K_MEM_PERM_RW | K_MEM_CACHE_NONE);
	k_mem_map_phys_bare(&ahci, AHCI_PHYS, 0x1000, K_MEM_PERM_RW | K_MEM_CACHE_NONE);
	return dec && advk && ahci ? 0 : -ENOMEM;
}

static void mbusx_fini(void)
{
	(void)restore_all();
	if (dec != NULL) {
		k_mem_unmap_phys_bare(dec, 0x1000);
	}
	if (advk != NULL) {
		k_mem_unmap_phys_bare(advk, 0x10000);
	}
	if (ahci != NULL) {
		k_mem_unmap_phys_bare(ahci, 0x1000);
	}
	dec = advk = ahci = NULL;
}

LAB_DBG_CMD((dbg), "dbg", mbusx, &mbusx_cmds,
	    "MBus windows (IO decode, CPU, PCIe outbound); SATA DMA straight to the host",
	    cmd_show, 1, 1, mbusx_init, mbusx_fini);
