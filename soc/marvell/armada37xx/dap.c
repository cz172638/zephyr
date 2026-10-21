/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 'dap info': what the debug infrastructure looks like from the running
 * system. MDRAR_EL1 gives the physical address of the debug ROM table
 * when the debug APB is visible in the memory map; the ROM table walk
 * then lists the CoreSight components behind it. Cores, CTIs and PMUs
 * only: DAP-Lite style; funnels, TMCs (ETF/ETR/ETB), TPIU, STM: a full
 * CoreSight trace subsystem.
 *
 * Reading a component the bus does not decode ends in an external
 * abort, so 'dap rd' and the walk are for bring-up only.
 */

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#define CS_DEVARCH 0xfbc
#define CS_DEVID   0xfc8
#define CS_DEVTYPE 0xfcc
#define CS_PIDR4   0xfd0
#define CS_PIDR0   0xfe0
#define CS_CIDR0   0xff0

#define CID_CLASS_ROM 0x1
#define CID_CLASS_CS  0x9

#define ARM_JEP106 0x43b /* continuation 4, id 0x3b */

#define MAX_DEPTH 4
#define MAX_COMPONENTS 64

struct dap_summary {
	int cores, ctis, pmus, etms, funnels, replicators, etf, etr, etb, tpiu, stm, other;
	int count;
};

static uint64_t read_mdrar(void)
{
	uint64_t v;

	__asm__ volatile("mrs %0, mdrar_el1" : "=r"(v));
	return v;
}

static uint32_t read_authstatus(void)
{
	uint64_t v;

	__asm__ volatile("mrs %0, dbgauthstatus_el1" : "=r"(v));
	return (uint32_t)v;
}

static mm_reg_t map4k(uintptr_t pa)
{
	mm_reg_t va;

	device_map(&va, pa, 0x1000, K_MEM_CACHE_NONE);
	return va;
}

static uint32_t id4(mm_reg_t base, uint32_t off)
{
	uint32_t v = 0;

	for (int i = 0; i < 4; i++) {
		v |= (sys_read32(base + off + 4 * i) & 0xff) << (8 * i);
	}
	return v;
}

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
	case 0x9e2: return "SoC-600 APB-AP";
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
	/* by DEVTYPE major: 1 trace sink, 2 link, 3 source */
	switch (devtype & 0xf) {
	case 2:
		s->funnels++;
		return;
	default:
		s->other++;
	}
}

static void walk(const struct shell *sh, uintptr_t pa, int depth, struct dap_summary *s)
{
	mm_reg_t base = map4k(pa);
	uint32_t cidr = id4(base, CS_CIDR0);
	uint64_t pidr = id4(base, CS_PIDR0) | ((uint64_t)id4(base, CS_PIDR4) << 32);
	uint8_t class = (cidr >> 12) & 0xf;
	uint16_t part = pidr & 0xfff;
	uint16_t des = ((pidr >> 12) & 0x7f) | (((pidr >> 32) & 0xf) << 8);
	uint8_t rev = (pidr >> 20) & 0xf;
	uint32_t devarch = 0, devid = 0, devtype = 0;
	const char *name = NULL;

	if (++s->count > MAX_COMPONENTS) {
		return;
	}

	if ((cidr & 0xffff0fff) != 0xb105000d) {
		shell_print(sh, "%*s0x%08lx: no component (CIDR 0x%08x)", depth * 2, "",
			    (unsigned long)pa, cidr);
		return;
	}

	if (class == CID_CLASS_CS) {
		devarch = sys_read32(base + CS_DEVARCH);
		devid = sys_read32(base + CS_DEVID);
		devtype = sys_read32(base + CS_DEVTYPE);
	}
	if (des == ARM_JEP106) {
		name = arm_part_name(part, devid);
	}

	shell_print(sh, "%*s0x%08lx: class %x designer 0x%03x part 0x%03x rev %u%s%s"
		    " devarch 0x%08x devtype 0x%02x", depth * 2, "", (unsigned long)pa, class,
		    des, part, rev, name ? " " : "", name ? name : "", devarch, devtype & 0xff);

	/* class 1 ROM table, or class 9 with the ROM table DEVARCH */
	if ((class == CID_CLASS_ROM ||
	     (class == CID_CLASS_CS && (devarch & 0xffff) == 0x0af7)) && depth < MAX_DEPTH) {
		for (int i = 0; i < 960; i++) {
			uint32_t e = sys_read32(base + 4 * i);

			if (e == 0) {
				break;
			}
			if ((e & BIT(0)) == 0) {
				continue;
			}
			walk(sh, pa + (int32_t)(e & 0xfffff000), depth + 1, s);
		}
		return;
	}

	account(s, des == ARM_JEP106 ? part : 0, devid, devtype);
}

static int cmd_dap_info(const struct shell *sh, size_t argc, char **argv)
{
	uint64_t mdrar = read_mdrar();
	uint32_t auth = read_authstatus();
	uintptr_t rom = argc > 1 ? (uintptr_t)strtoull(argv[1], NULL, 0)
				 : (uintptr_t)(mdrar & GENMASK64(47, 12));
	struct dap_summary s = {0};
	bool trace;

	shell_print(sh, "MDRAR_EL1 0x%016llx (%s)", mdrar,
		    (mdrar & 0x3) == 0x3 ? "ROM table address valid" : "no memory mapped ROM table");
	shell_print(sh, "DBGAUTHSTATUS 0x%08x: NS invasive %s, NS non-invasive %s, "
		    "S invasive %s, S non-invasive %s", auth,
		    (auth & 0x3) == 0x3 ? "on" : "off", ((auth >> 2) & 0x3) == 0x3 ? "on" : "off",
		    ((auth >> 4) & 0x3) == 0x3 ? "on" : "off",
		    ((auth >> 6) & 0x3) == 0x3 ? "on" : "off");

	if (argc < 2 && (mdrar & 0x3) != 0x3) {
		shell_print(sh, "give the ROM table address: dap info <addr>");
		return 0;
	}

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

static int cmd_dap_rd(const struct shell *sh, size_t argc, char **argv)
{
	uintptr_t pa = (uintptr_t)strtoull(argv[1], NULL, 0);
	int n = argc > 2 ? strtol(argv[2], NULL, 0) : 1;
	mm_reg_t base = map4k(pa & ~0xfffUL);

	for (int i = 0; i < n && (pa & 0xfff) + 4 * i < 0x1000; i++) {
		shell_print(sh, "0x%08lx: 0x%08x", (unsigned long)(pa + 4 * i),
			    sys_read32(base + (pa & 0xfff) + 4 * i));
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(dap_cmds,
	SHELL_CMD_ARG(info, NULL, "ROM table walk: [rom table address]", cmd_dap_info, 1, 1),
	SHELL_CMD_ARG(rd, NULL, "read debug APB: <addr> [words]", cmd_dap_rd, 2, 1),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(dap, &dap_cmds, "CoreSight / DAP from the running system", NULL);
