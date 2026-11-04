/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Looking for the debug infrastructure from the running system.
 *
 * MDRAR_EL1 is deprecated in Armv8 (and 0 on the Armada 3720), so the ROM
 * table address is the SoC's (CONFIG_SOC_ARMADA37XX_DAP_ROM_BASE, unknown = 0 on the
 * Armada 3720). The DAP's APB-AP (JTAG) sees the debug APB at
 * 0x80000000 (OpenOCD: CTIs at 0x80820000/0x80920000), the system may
 * map it elsewhere, or not at all. Every read here goes through a probe
 * that turns an external abort into an error, so ranges can be scanned:
 *
 *   dap win                     CPU address decode windows (0xd000cf00)
 *   dap scan <start> <end> [step]  CoreSight CIDR at each step
 *   dap info <rom table>        walk: DAP-Lite style or full trace path
 *   dap rd <addr> [words]
 *
 * A scan reads offsets 0xff0..0xffc of every step; inside the internal
 * register space that may touch read-sensitive registers.
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/arch/arm64/exception.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/shell/shell.h>
#include <soc.h>
#include <zephyr/sys/util.h>

#define CS_DEVARCH 0xfbc
#define CS_DEVID   0xfc8
#define CS_DEVTYPE 0xfcc
#define CS_PIDR4   0xfd0
#define CS_PIDR0   0xfe0
#define CS_CIDR0   0xff0

#define CID_CLASS_ROM 0x1
#define CID_CLASS_CS  0x9

#define ARM_JEP106 0x43b

#define MAX_DEPTH      4
#define MAX_COMPONENTS 64
#define SCAN_CHUNK     0x100000

#define CPU_DEC_BASE 0xd000cf00UL

/* ------------------------------------------------------------------ */
/* Probing reads                                                       */

/* a3700_probe_read32() and the fault count live in probe.c */
#define probe_faults a3700_probe_faults

struct win {
	uint8_t *va;
	uintptr_t pa;
	size_t size;
};

static void win_map(struct win *w, uintptr_t pa, size_t size)
{
	w->pa = ROUND_DOWN(pa, 0x1000);
	w->size = ROUND_UP(pa + size, 0x1000) - w->pa;
	k_mem_map_phys_bare(&w->va, w->pa, w->size, K_MEM_CACHE_NONE | K_MEM_PERM_RW);
}

static void win_unmap(struct win *w)
{
	k_mem_unmap_phys_bare(w->va, w->size);
}

/*
 * 'dap smc on': reads of the debug window go through BL31
 * (TF-A A3700_DBG_SIP=1, secure accesses), the rest stays direct.
 */
#define SIP_DBG_READ 0x82000022
#define DBG_BASE     0x80000000UL
#define DBG_SIZE     0x01000000UL

static bool use_smc;

static int prd(const struct win *w, uintptr_t pa, uint32_t *v)
{
	if (use_smc && pa >= DBG_BASE && pa < DBG_BASE + DBG_SIZE) {
		struct arm_smccc_res res;

		arm_smccc_smc(SIP_DBG_READ, pa - DBG_BASE, 0, 0, 0, 0, 0, 0, &res);
		if (res.a0 != 0) {
			return -EIO;
		}
		*v = (uint32_t)res.a1;
		return 0;
	}
	return a3700_probe_read32((uintptr_t)w->va + (pa - w->pa), v);
}

/*
 * Is @p pa decoded for the CPU at all? An address no window covers does
 * not abort on the Armada 3700, the load just hangs: internal registers
 * (0xd0000000, 32 MiB) and the enabled CPU decode windows only.
 */
static bool cpu_decodes(uintptr_t pa)
{
	struct win w;
	bool yes = pa >= 0xd0000000UL && pa < 0xd2000000UL;

	win_map(&w, CPU_DEC_BASE, 0x100);
	for (int wn = 0; wn < 5 && !yes; wn++) {
		uint32_t ctrl, size, base;

		if (a3700_probe_read32((uintptr_t)w.va + 0x10 * wn, &ctrl) != 0 ||
		    a3700_probe_read32((uintptr_t)w.va + 0x10 * wn + 4, &size) != 0 ||
		    a3700_probe_read32((uintptr_t)w.va + 0x10 * wn + 8, &base) != 0) {
			continue;
		}
		if ((ctrl & 1) != 0 && pa >= ((uintptr_t)base << 16) &&
		    pa < ((uintptr_t)base << 16) + (((uintptr_t)size + 1) << 16)) {
			yes = true;
		}
	}
	win_unmap(&w);
	return yes;
}

/*
 * Before touching an address. Undecoded addresses hang instead of
 * aborting. The debug APB (0x80000000) is decoded (CPU window 4, target
 * 3) but every read of it hangs, from BL31 (A3700_DBG_SIP) too and with
 * the debug clocks on: the debug power domain is presumably off, and
 * only the DAP (CDBGPWRUPREQ over JTAG/SWD) turns it on. So the window
 * is read only after "dap force on", e.g. with OpenOCD attached.
 */
static bool dbg_force;

static int dbg_reachable(const struct shell *sh, uintptr_t pa)
{
	bool in_dbg = pa >= DBG_BASE && pa < DBG_BASE + DBG_SIZE;

	if (in_dbg && !dbg_force) {
		shell_error(sh, "0x%lx: CPU window 4 (target 3, 0x80000000) hangs on read; "
			    "\"dap force on\" to try anyway", (unsigned long)pa);
		return -EFAULT;
	}
	if (!cpu_decodes(pa) && !(in_dbg && use_smc)) {
		shell_error(sh, "0x%lx is not decoded for the CPU (dap win): a read would hang",
			    (unsigned long)pa);
		return -EFAULT;
	}
	return 0;
}

/*
 * Claim tags: set and cleared through system registers here, visible to
 * the DAP at core debug base + 0xfa0 (CLAIMSET) / 0xfa4 (CLAIMCLR reads
 * the tags). Set a pattern here, read it over JTAG: that names this
 * core's debug block in the APB-AP view; the other way round as well.
 */
static int cmd_dap_claim(const struct shell *sh, size_t argc, char **argv)
{
	uint64_t v;

	if (argc > 2) {
		uint64_t bits = strtoull(argv[2], NULL, 0) & 0xff;

		if (strcmp(argv[1], "set") == 0) {
			__asm__ volatile("msr dbgclaimset_el1, %0; isb" : : "r"(bits));
		} else if (strcmp(argv[1], "clr") == 0) {
			__asm__ volatile("msr dbgclaimclr_el1, %0; isb" : : "r"(bits));
		} else {
			shell_error(sh, "claim [set|clr <bits>]");
			return -EINVAL;
		}
	}
	__asm__ volatile("mrs %0, dbgclaimclr_el1" : "=r"(v));
	shell_print(sh, "CPU %u claim tags 0x%02llx (DAP: core debug base + 0xfa4)",
		    arch_curr_cpu()->id, (unsigned long long)(v & 0xff));
	return 0;
}

static int cmd_dap_force(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1) {
		dbg_force = strcmp(argv[1], "on") == 0;
	}
	shell_print(sh, "reads of 0x80000000.. %s", dbg_force ? "allowed (they hang)" : "refused");
	return 0;
}

/* the four ID bytes of a CIDR/PIDR group */
static int pid4(const struct win *w, uintptr_t pa, uint32_t *out)
{
	uint32_t v, r = 0;

	for (int i = 0; i < 4; i++) {
		if (prd(w, pa + 4 * i, &v) < 0) {
			return -EIO;
		}
		r |= (v & 0xff) << (8 * i);
	}
	*out = r;
	return 0;
}

static bool is_cidr(uint32_t cidr)
{
	return (cidr & 0xffff0fff) == 0xb105000d;
}

/* ------------------------------------------------------------------ */
/* Components                                                          */

struct dap_summary {
	int cores, ctis, pmus, etms, funnels, replicators, etf, etr, etb, tpiu, stm, other;
	int count;
};

static const char *arm_part_name(uint16_t part, uint32_t devid)
{
	switch (part) {
	case 0x4a1: return "Cortex-A53 ROM table";
	case 0xd03: return "Cortex-A53 debug";
	case 0x9d3: return "Cortex-A53 PMU";
	case 0x95d: return "Cortex-A53 ETM";
	case 0x9a8: return "Cortex-A53 CTI";
	case 0x906: return "CTI";
	case 0x907: return "ETB";
	case 0x908: return "trace funnel (CSTF)";
	case 0x909: return "replicator";
	case 0x912: return "TPIU";
	case 0x913: return "ITM";
	case 0x914: return "SWO";
	case 0x961:
		switch ((devid >> 6) & 0x3) {
		case 0: return "TMC (ETB)";
		case 1: return "TMC (ETR)";
		case 2: return "TMC (ETF)";
		default: return "TMC";
		}
	case 0x962: return "STM";
	case 0x963: return "STM-500";
	case 0x9e7: return "SoC-600 TPIU";
	case 0x9eb: return "SoC-600 funnel";
	case 0x9ec: return "SoC-600 replicator";
	case 0x9e8: return "SoC-600 TMC";
	case 0x9ed: return "SoC-600 CTI";
	default: return NULL;
	}
}

static void account(struct dap_summary *s, uint16_t part, uint32_t devid, uint32_t devtype)
{
	switch (part) {
	case 0xd03: s->cores++; return;
	case 0x9a8: case 0x906: case 0x9ed: s->ctis++; return;
	case 0x9d3: s->pmus++; return;
	case 0x95d: s->etms++; return;
	case 0x908: case 0x9eb: s->funnels++; return;
	case 0x909: case 0x9ec: s->replicators++; return;
	case 0x907: s->etb++; return;
	case 0x912: case 0x9e7: s->tpiu++; return;
	case 0x962: case 0x963: s->stm++; return;
	case 0x961: case 0x9e8:
		switch ((devid >> 6) & 0x3) {
		case 0: s->etb++; return;
		case 1: s->etr++; return;
		default: s->etf++; return;
		}
	default:
		break;
	}
	/* DEVTYPE major 2: link (funnel/replicator) */
	if ((devtype & 0xf) == 2) {
		s->funnels++;
	} else {
		s->other++;
	}
}

/* one line for the component at pa; false if there is none */
static bool describe(const struct shell *sh, uintptr_t pa, int depth, uint8_t *class_out,
		     uint16_t *part_out, uint16_t *des_out, uint32_t *devarch_out,
		     uint32_t *devid_out, uint32_t *devtype_out)
{
	struct win w;
	uint32_t cidr, p0, p4, devarch = 0, devid = 0, devtype = 0;
	uint64_t pidr;
	uint8_t class;
	uint16_t part, des;
	const char *name = NULL;
	bool ok = false;

	win_map(&w, pa, 0x1000);
	if (pid4(&w, pa + CS_CIDR0, &cidr) < 0) {
		shell_print(sh, "%*s0x%08lx: bus error", depth * 2, "", (unsigned long)pa);
		goto out;
	}
	if (!is_cidr(cidr)) {
		shell_print(sh, "%*s0x%08lx: no component (CIDR 0x%08x)", depth * 2, "",
			    (unsigned long)pa, cidr);
		goto out;
	}
	if (pid4(&w, pa + CS_PIDR0, &p0) < 0 || pid4(&w, pa + CS_PIDR4, &p4) < 0) {
		goto out;
	}
	pidr = p0 | ((uint64_t)p4 << 32);
	class = (cidr >> 12) & 0xf;
	part = pidr & 0xfff;
	des = ((pidr >> 12) & 0x7f) | (((pidr >> 32) & 0xf) << 8);
	if (class == CID_CLASS_CS) {
		(void)prd(&w, pa + CS_DEVARCH, &devarch);
		(void)prd(&w, pa + CS_DEVID, &devid);
		(void)prd(&w, pa + CS_DEVTYPE, &devtype);
	}
	if (des == ARM_JEP106) {
		name = arm_part_name(part, devid);
	}
	shell_print(sh, "%*s0x%08lx: class %x designer 0x%03x part 0x%03x rev %u%s%s"
		    " devarch 0x%08x devtype 0x%02x", depth * 2, "", (unsigned long)pa, class,
		    des, part, (unsigned int)((pidr >> 20) & 0xf), name ? " " : "",
		    name ? name : "", devarch, devtype & 0xff);
	*class_out = class;
	*part_out = part;
	*des_out = des;
	*devarch_out = devarch;
	*devid_out = devid;
	*devtype_out = devtype;
	ok = true;
out:
	win_unmap(&w);
	return ok;
}

static void walk(const struct shell *sh, uintptr_t pa, int depth, struct dap_summary *s)
{
	uint8_t class;
	uint16_t part, des;
	uint32_t devarch, devid, devtype;
	struct win w;

	if (++s->count > MAX_COMPONENTS ||
	    !describe(sh, pa, depth, &class, &part, &des, &devarch, &devid, &devtype)) {
		return;
	}

	if ((class == CID_CLASS_ROM ||
	     (class == CID_CLASS_CS && (devarch & 0xffff) == 0x0af7)) && depth < MAX_DEPTH) {
		uint32_t entries[64];
		int n = 0;

		/* read the entries first: the walk below remaps */
		win_map(&w, pa, 0x1000);
		for (int i = 0; i < ARRAY_SIZE(entries); i++) {
			if (prd(&w, pa + 4 * i, &entries[i]) < 0 || entries[i] == 0) {
				break;
			}
			n++;
		}
		win_unmap(&w);

		for (int i = 0; i < n; i++) {
			if ((entries[i] & BIT(0)) != 0) {
				walk(sh, pa + (int32_t)(entries[i] & 0xfffff000), depth + 1, s);
			}
		}
		return;
	}

	account(s, des == ARM_JEP106 ? part : 0, devid, devtype);
}

/* ------------------------------------------------------------------ */
/* Shell                                                               */

static uint32_t read_authstatus(void)
{
	uint64_t v;

	__asm__ volatile("mrs %0, dbgauthstatus_el1" : "=r"(v));
	return (uint32_t)v;
}

static uint64_t read_mdrar(void)
{
	uint64_t v;

	__asm__ volatile("mrs %0, mdrar_el1" : "=r"(v));
	return v;
}

static int cmd_dap_info(const struct shell *sh, size_t argc, char **argv)
{
	uint64_t mdrar = read_mdrar();
	uint32_t auth = read_authstatus();
	struct dap_summary s = {0};
	uintptr_t rom;
	bool trace;

	shell_print(sh, "MDRAR_EL1 0x%016llx (deprecated in Armv8, not used for the address)",
		    mdrar);
	shell_print(sh, "DBGAUTHSTATUS 0x%08x: NS invasive %s, NS non-invasive %s, "
		    "S invasive %s, S non-invasive %s", auth,
		    (auth & 0x3) == 0x3 ? "on" : "off", ((auth >> 2) & 0x3) == 0x3 ? "on" : "off",
		    ((auth >> 4) & 0x3) == 0x3 ? "on" : "off",
		    ((auth >> 6) & 0x3) == 0x3 ? "on" : "off");

	/* the ROM table where the SoC puts it (debug APB), or the one given */
	rom = argc > 1 ? (uintptr_t)strtoull(argv[1], NULL, 0)
		       : (uintptr_t)CONFIG_SOC_ARMADA37XX_DAP_ROM_BASE;
	if (rom == 0) {
		shell_print(sh, "CPU address of the ROM table unknown (0x80000000 is the DAP's "
			    "APB-AP view): dap info <addr>; to match both views see dap claim");
		return 0;
	}
	if (dbg_reachable(sh, rom) != 0) {
		return -EFAULT;
	}
	shell_print(sh, "ROM table at 0x%lx (%s)", (unsigned long)rom,
		    argc > 1 ? "given" : "CONFIG_SOC_ARMADA37XX_DAP_ROM_BASE");

	walk(sh, rom, 0, &s);

	shell_print(sh, "\n%d components: %d cores, %d CTI, %d PMU, %d ETM, %d funnel, "
		    "%d replicator, %d ETF, %d ETR, %d ETB, %d TPIU, %d STM, %d other",
		    s.count, s.cores, s.ctis, s.pmus, s.etms, s.funnels, s.replicators, s.etf,
		    s.etr, s.etb, s.tpiu, s.stm, s.other);
	trace = s.funnels || s.replicators || s.etf || s.etr || s.etb || s.tpiu || s.stm;
	shell_print(sh, "%s", trace ? "full CoreSight: trace infrastructure present"
				    : s.etms ? "ETMs without trace sinks/links on this ROM table"
					     : "DAP-Lite style: debug only, no trace path");
	return 0;
}

static int cmd_dap_scan(const struct shell *sh, size_t argc, char **argv)
{
	uintptr_t start = ROUND_DOWN((uintptr_t)strtoull(argv[1], NULL, 0), 0x1000);
	uintptr_t end = (uintptr_t)strtoull(argv[2], NULL, 0);
	uintptr_t step = argc > 3 ? (uintptr_t)strtoull(argv[3], NULL, 0) : 0x1000;
	uint32_t found = 0, faults0 = probe_faults;
	uintptr_t err_start = 0;
	bool in_err = false;

	if (step < 0x1000 || (step & 0xfff) != 0 || end <= start) {
		shell_error(sh, "step: multiple of 0x1000, end > start");
		return -EINVAL;
	}
	if (start < DBG_BASE + DBG_SIZE && end > DBG_BASE &&
	    dbg_reachable(sh, MAX(start, DBG_BASE)) != 0) {
		return -EFAULT;
	}

	for (uintptr_t base = start; base < end; base += SCAN_CHUNK) {
		struct win w;
		size_t len = MIN(SCAN_CHUNK, end - base);

		win_map(&w, base, len);
		for (uintptr_t pa = base; pa < base + len; pa += step) {
			uint32_t cidr;
			int ret = pid4(&w, pa + CS_CIDR0, &cidr);

			if (ret < 0) {
				if (!in_err) {
					err_start = pa;
					in_err = true;
				}
				continue;
			}
			if (in_err) {
				shell_print(sh, "  0x%08lx..0x%08lx: bus error",
					    (unsigned long)err_start, (unsigned long)pa - 1);
				in_err = false;
			}
			if (is_cidr(cidr)) {
				uint32_t p0 = 0;

				(void)pid4(&w, pa + CS_PIDR0, &p0);
				shell_print(sh, "0x%08lx: CoreSight class %x, part 0x%03x",
					    (unsigned long)pa, (cidr >> 12) & 0xf, p0 & 0xfff);
				found++;
			}
		}
		win_unmap(&w);
	}
	if (in_err) {
		shell_print(sh, "  0x%08lx..0x%08lx: bus error", (unsigned long)err_start,
			    (unsigned long)end - 1);
	}
	shell_print(sh, "%u components, %u faulting reads", found, probe_faults - faults0);
	return 0;
}

static const char *const dec_target[16] = {
	[0] = "DRAM", [2] = "PCIe",
};

static int cmd_dap_win(const struct shell *sh, size_t argc, char **argv)
{
	struct win w;
	uint32_t v[64];

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	win_map(&w, CPU_DEC_BASE, 0x100);
	for (int i = 0; i < 64; i++) {
		if (prd(&w, CPU_DEC_BASE + 4 * i, &v[i]) < 0) {
			v[i] = 0xdeadbeef;
		}
	}
	win_unmap(&w);

	for (int i = 0; i < 64; i += 4) {
		shell_print(sh, "0x%08lx: %08x %08x %08x %08x", CPU_DEC_BASE + 4 * i, v[i],
			    v[i + 1], v[i + 2], v[i + 3]);
	}
	for (int wn = 0; wn < 5; wn++) {
		uint32_t ctrl = v[wn * 4], size = v[wn * 4 + 1], base = v[wn * 4 + 2];
		uint8_t tgt = (ctrl >> 4) & 0xf;

		shell_print(sh, "win %d: %s target %u (%s) base 0x%08x size 0x%08x remap 0x%08x", wn,
			    (ctrl & 1) ? "on " : "off", tgt, dec_target[tgt] ? dec_target[tgt] : "?",
			    base << 16, (size + 1) << 16, v[wn * 4 + 3]);
	}
	shell_print(sh, "CCI base 0x%08x, ROM base 0x%08x", v[0xe0 / 4] << 20, v[0xf4 / 4]);
	return 0;
}

static int cmd_dap_rd(const struct shell *sh, size_t argc, char **argv)
{
	uintptr_t pa = (uintptr_t)strtoull(argv[1], NULL, 0);
	int n = argc > 2 ? strtol(argv[2], NULL, 0) : 1;
	struct win w;
	uint32_t v;

	n = CLAMP(n, 1, 256);
	if (dbg_reachable(sh, pa) != 0) {
		return -EFAULT;
	}
	win_map(&w, pa, 4 * n);
	for (int i = 0; i < n; i++) {
		if (prd(&w, pa + 4 * i, &v) < 0) {
			shell_print(sh, "0x%08lx: bus error", (unsigned long)(pa + 4 * i));
		} else {
			shell_print(sh, "0x%08lx: 0x%08x", (unsigned long)(pa + 4 * i), v);
		}
	}
	win_unmap(&w);
	return 0;
}

/*
 * NB peripheral clocks of the debug/trace infrastructure (Linux
 * armada-37xx-periph data_nb[]): with them gated, an APB read of the
 * debug window does not even abort, it hangs.
 */
#define NB_CLK_DIS 0xd0013014UL

static const struct {
	uint8_t idx;
	uint8_t gate;
	const char *name;
} dbg_clks[] = {
	{2, 6, "sec_at"}, {3, 7, "sec_dap"}, {4, 8, "tscem"}, {5, 10, "tscem_tmx"},
	{13, 22, "trace"},
};

static int cmd_dap_clk(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *nb = DEVICE_DT_GET(DT_NODELABEL(nb_periph_clk));
	bool on = argc > 1 && strcmp(argv[1], "on") == 0;
	struct win w;
	uint32_t dis;

	if (on) {
		for (int i = 0; i < ARRAY_SIZE(dbg_clks); i++) {
			int ret = clock_control_on(nb, (clock_control_subsys_t)(uintptr_t)dbg_clks[i].idx);

			if (ret < 0) {
				shell_error(sh, "%s: %d", dbg_clks[i].name, ret);
			}
		}
	}

	win_map(&w, NB_CLK_DIS, 4);
	(void)prd(&w, NB_CLK_DIS, &dis);
	win_unmap(&w);
	shell_print(sh, "NB CLK_DIS 0x%08x", dis);
	for (int i = 0; i < ARRAY_SIZE(dbg_clks); i++) {
		uint32_t hz = 0;

		(void)clock_control_get_rate(nb, (clock_control_subsys_t)(uintptr_t)dbg_clks[i].idx,
					     &hz);
		shell_print(sh, "  %-9s (bit %2u): %s, %u Hz", dbg_clks[i].name, dbg_clks[i].gate,
			    (dis & BIT(dbg_clks[i].gate)) ? "gated" : "on", hz);
	}
	return 0;
}

static int cmd_dap_smc(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1) {
		use_smc = strcmp(argv[1], "on") == 0;
	}
	shell_print(sh, "debug window 0x%08lx+0x%lx: %s", DBG_BASE, DBG_SIZE,
		    use_smc ? "through BL31 (secure)" : "direct (non-secure)");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(dap_cmds,
	SHELL_CMD_ARG(claim, NULL, "claim tags of this core: [set|clr <bits>]", cmd_dap_claim, 1,
		      2),
	SHELL_CMD_ARG(force, NULL, "allow reads of CPU window 0x80000000 (they hang): [on|off]",
		      cmd_dap_force, 1, 1),
	SHELL_CMD_ARG(smc, NULL, "debug window reads through TF-A: [on|off]", cmd_dap_smc, 1, 1),
	SHELL_CMD_ARG(clk, NULL, "debug/trace clocks: [on]", cmd_dap_clk, 1, 1),
	SHELL_CMD_ARG(info, NULL, "ROM table walk: [rom table address], default the SoC's debug APB", cmd_dap_info, 1, 1),
	SHELL_CMD_ARG(scan, NULL, "CoreSight IDs: <start> <end> [step]", cmd_dap_scan, 3, 1),
	SHELL_CMD_ARG(win, NULL, "CPU address decode windows", cmd_dap_win, 1, 0),
	SHELL_CMD_ARG(rd, NULL, "probing read: <addr> [words]", cmd_dap_rd, 2, 1),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(dap, &dap_cmds, "CoreSight / DAP from the running system", NULL);
