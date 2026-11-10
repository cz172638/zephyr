/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * dbg comphy: Armada 3720 COMPHY lane registers, for finding the DFT
 * functions (loopback, PRBS) Linux and TF-A do not use.
 *
 * Lanes (Linux phy-mvebu-a3700-comphy.c): 0 USB3/GbE1 at 0xd005c000,
 * 1 PCIe/GbE0 at 0xd001f000 (16-bit registers at reg << 1), 2 SATA/USB3
 * indirect through 0xd00e0178/0xd00e017c with reg + 0x200.
 *
 * Known from Linux: 0x01 POWER_PLL_CTRL, 0x23 "DIG_LOOPBACK_EN" (only
 * the data width [11:10] and PLL_READY_TX bit 4 are used; U-Boot calls
 * the SATA copy vphy_loopback_reg0), 0x24 SYNC_PATTERN (Tx/Rx invert),
 * 0x26 ISOLATION_CTRL, 0x4f MISC_CTRL0, 0x52 SFT_RESET, 0x1c2 PIPE test
 * mode. Kirkwood's PCIe PHY (functional spec 6.19) has loopback in reg
 * 0x0 bits 0/1 and PRBS in 0x40/0x50/0x51; whether anything of that
 * maps here is what this module is for.
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/lab/mod.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/sys_io.h>

#include <soc.h> /* a3700_probe_read32: aborts of gated blocks */

#define NREGS 0x200

static const uintptr_t lane_phys[3] = {0xd005c000, 0xd001f000, 0xd00e0000};
static uint8_t *lane_va[3];
static uint16_t snap[3][NREGS];
static bool snapped[3];
static bool lane_ok[3];

static int map_lanes(void)
{
	for (int l = 0; l < 3; l++) {
		k_mem_map_phys_bare(&lane_va[l], lane_phys[l], 0x1000,
				    K_MEM_PERM_RW | K_MEM_CACHE_NONE);
		if (lane_va[l] == NULL) {
			return -ENOMEM;
		}
		{
			uint32_t v;
			/* lane 2 sits behind the AHCI block: clock gated = abort */
			lane_ok[l] = a3700_probe_read32((uintptr_t)lane_va[l] +
								(l == 2 ? 0x178 : 0), &v) == 0;
		}
	}
	return 0;
}

static void unmap_lanes(void)
{
	for (int l = 0; l < 3; l++) {
		if (lane_va[l] != NULL) {
			k_mem_unmap_phys_bare(lane_va[l], 0x1000);
			lane_va[l] = NULL;
		}
	}
}

static uint16_t rd(int lane, uint16_t reg)
{
	if (lane == 2) {
		sys_write32(reg + 0x200, (mem_addr_t)lane_va[2] + 0x178);
		return (uint16_t)sys_read32((mem_addr_t)lane_va[2] + 0x17c);
	}
	return sys_read16((mem_addr_t)lane_va[lane] + (reg << 1));
}

static void wr(int lane, uint16_t reg, uint16_t val)
{
	if (lane == 2) {
		sys_write32(reg + 0x200, (mem_addr_t)lane_va[2] + 0x178);
		sys_write32(val, (mem_addr_t)lane_va[2] + 0x17c);
		return;
	}
	sys_write16(val, (mem_addr_t)lane_va[lane] + (reg << 1));
}

static int num(const struct shell *sh, const char *s, unsigned long *v, unsigned long max)
{
	char *end;

	*v = strtoul(s, &end, 0);
	if (*end != '\0' || *v > max) {
		shell_error(sh, "bad value '%s' (max 0x%lx)", s, max);
		return -EINVAL;
	}
	return 0;
}

static int lane_arg(const struct shell *sh, const char *s, int *lane)
{
	unsigned long v;

	if (num(sh, s, &v, 2) != 0) {
		return -EINVAL;
	}
	if (!lane_ok[v]) {
		shell_error(sh, "lane %lu: no access (block clock gated / powered down?)", v);
		return -EIO;
	}
	*lane = (int)v;
	return 0;
}

/* dump <lane> [from [to]] */
static int cmd_dump(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long from = 0, to = NREGS - 1;
	int lane;

	if (lane_arg(sh, argv[1], &lane) != 0 ||
	    (argc > 2 && num(sh, argv[2], &from, NREGS - 1) != 0) ||
	    (argc > 3 && num(sh, argv[3], &to, NREGS - 1) != 0)) {
		return -EINVAL;
	}
	for (unsigned long r = from & ~7UL; r <= to; r += 8) {
		shell_fprintf(sh, SHELL_NORMAL, "%03lx:", r);
		for (int i = 0; i < 8; i++) {
			shell_fprintf(sh, SHELL_NORMAL, " %04x", rd(lane, r + i));
		}
		shell_fprintf(sh, SHELL_NORMAL, "\n");
	}
	return 0;
}

static int cmd_snap(const struct shell *sh, size_t argc, char **argv)
{
	int lane;

	if (lane_arg(sh, argv[1], &lane) != 0) {
		return -EINVAL;
	}
	for (int r = 0; r < NREGS; r++) {
		snap[lane][r] = rd(lane, r);
	}
	snapped[lane] = true;
	shell_print(sh, "lane %d: %d registers saved", lane, NREGS);
	return 0;
}

/* diff <lane>: registers that differ from the snapshot */
static int cmd_diff(const struct shell *sh, size_t argc, char **argv)
{
	int lane, n = 0;

	if (lane_arg(sh, argv[1], &lane) != 0) {
		return -EINVAL;
	}
	if (!snapped[lane]) {
		shell_error(sh, "no snapshot of lane %d (snap)", lane);
		return -ENOENT;
	}
	for (int r = 0; r < NREGS; r++) {
		uint16_t v = rd(lane, r);

		if (v != snap[lane][r]) {
			shell_print(sh, "%03x: %04x -> %04x  changed %04x", r, snap[lane][r], v,
				    v ^ snap[lane][r]);
			n++;
		}
	}
	shell_print(sh, "%d registers differ", n);
	return 0;
}

/* watch <lane> [samples] [us]: bits that move on their own (status) */
static int cmd_watch(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long samples = 1000, us = 10;
	static uint16_t first[NREGS], moved[NREGS];
	int lane, n = 0;

	if (lane_arg(sh, argv[1], &lane) != 0 ||
	    (argc > 2 && num(sh, argv[2], &samples, 1000000) != 0) ||
	    (argc > 3 && num(sh, argv[3], &us, 1000000) != 0)) {
		return -EINVAL;
	}
	for (int r = 0; r < NREGS; r++) {
		first[r] = rd(lane, r);
		moved[r] = 0;
	}
	for (unsigned long s = 0; s < samples; s++) {
		for (int r = 0; r < NREGS; r++) {
			moved[r] |= rd(lane, r) ^ first[r];
		}
		k_busy_wait(us);
	}
	for (int r = 0; r < NREGS; r++) {
		if (moved[r] != 0) {
			shell_print(sh, "%03x: %04x  moving bits %04x", r, first[r], moved[r]);
			n++;
		}
	}
	shell_print(sh, "%d registers with moving bits in %lu samples", n, samples);
	return 0;
}

static int cmd_rd(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long reg;
	int lane;

	if (lane_arg(sh, argv[1], &lane) != 0 || num(sh, argv[2], &reg, NREGS - 1) != 0) {
		return -EINVAL;
	}
	shell_print(sh, "lane %d %03lx: %04x", lane, reg, rd(lane, reg));
	return 0;
}

/* wr <lane> <reg> <val> [mask] */
static int cmd_wr(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long reg, val, mask = 0xffff;
	uint16_t old, new;
	int lane;

	if (lane_arg(sh, argv[1], &lane) != 0 || num(sh, argv[2], &reg, NREGS - 1) != 0 ||
	    num(sh, argv[3], &val, 0xffff) != 0 ||
	    (argc > 4 && num(sh, argv[4], &mask, 0xffff) != 0)) {
		return -EINVAL;
	}
	old = rd(lane, reg);
	new = (old & ~mask) | (val & mask);
	wr(lane, reg, new);
	shell_print(sh, "lane %d %03lx: %04x -> %04x, reads %04x", lane, reg, old, new,
		    rd(lane, reg));
	return 0;
}

/*
 * kw <lane> lb shallow|deep|off | prbs on|off|reset|count: the Kirkwood
 * PCIe PHY recipe (functional spec 6.19.2/6.19.3) applied to this
 * COMPHY. A hypothesis: run snap first, diff after, restore by "off"
 * or a reset.
 */
static int cmd_kw(const struct shell *sh, size_t argc, char **argv)
{
	int lane;

	if (lane_arg(sh, argv[1], &lane) != 0) {
		return -EINVAL;
	}
	if (strcmp(argv[2], "lb") == 0 && argc > 3) {
		uint16_t v = rd(lane, 0x00) & ~0x3;

		if (strcmp(argv[3], "shallow") == 0) {
			v |= BIT(0);
		} else if (strcmp(argv[3], "deep") == 0) {
			v |= BIT(1);
		} else if (strcmp(argv[3], "off") != 0) {
			return -EINVAL;
		}
		wr(lane, 0x00, v);
		if (v & 0x3) {
			/* Kirkwood step 2: 0x44[9:8] = 3, 0x81[11:10] = 2 */
			wr(lane, 0x44, (rd(lane, 0x44) & ~(0x3 << 8)) | (0x3 << 8));
			wr(lane, 0x81, (rd(lane, 0x81) & ~(0x3 << 10)) | (0x2 << 10));
		}
		shell_print(sh, "lane %d: 00=%04x 44=%04x 81=%04x", lane, rd(lane, 0x00),
			    rd(lane, 0x44), rd(lane, 0x81));
		return 0;
	}
	if (strcmp(argv[2], "prbs") == 0 && argc > 3) {
		uint16_t v = rd(lane, 0x40);

		if (strcmp(argv[3], "on") == 0) {
			wr(lane, 0x40, v | BIT(2));
		} else if (strcmp(argv[3], "off") == 0) {
			wr(lane, 0x40, v & ~BIT(2));
		} else if (strcmp(argv[3], "reset") == 0) {
			wr(lane, 0x40, v | BIT(3));
			wr(lane, 0x40, v & ~BIT(3));
		} else if (strcmp(argv[3], "count") != 0) {
			return -EINVAL;
		}
		shell_print(sh, "lane %d: 40=%04x errors 50=%04x 51=%04x", lane, rd(lane, 0x40),
			    rd(lane, 0x50), rd(lane, 0x51));
		return 0;
	}
	shell_error(sh, "kw <lane> lb shallow|deep|off | prbs on|off|reset|count");
	return -EINVAL;
}

static int cmd_comphy(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	for (int l = 0; l < 3; l++) {
		if (!lane_ok[l]) {
			shell_print(sh, "lane %d: no access", l);
			continue;
		}
		shell_print(sh, "lane %d: 01 pwr/pll %04x  23 dig_lb %04x  24 sync %04x  26 iso "
			    "%04x  4f misc0 %04x  52 rst %04x", l, rd(l, 0x01), rd(l, 0x23),
			    rd(l, 0x24), rd(l, 0x26), rd(l, 0x4f), rd(l, 0x52));
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(comphy_cmds,
	SHELL_CMD_ARG(dump, NULL, "<lane> [from [to]]  Registers 0x000-0x1ff", cmd_dump, 2, 2),
	SHELL_CMD_ARG(snap, NULL, "<lane>  Save all registers", cmd_snap, 2, 0),
	SHELL_CMD_ARG(diff, NULL, "<lane>  Registers changed since snap", cmd_diff, 2, 0),
	SHELL_CMD_ARG(watch, NULL, "<lane> [samples] [us]  Bits that change on their own",
		      cmd_watch, 2, 2),
	SHELL_CMD_ARG(rd, NULL, "<lane> <reg>", cmd_rd, 3, 0),
	SHELL_CMD_ARG(wr, NULL, "<lane> <reg> <val> [mask]  (can break SATA/USB3/PCIe!)", cmd_wr,
		      4, 1),
	SHELL_CMD_ARG(kw, NULL,
		      "<lane> lb shallow|deep|off | prbs on|off|reset|count  Kirkwood PHY recipe "
		      "(hypothesis)", cmd_kw, 4, 0),
	SHELL_SUBCMD_SET_END);

static int comphy_init(void)
{
	return map_lanes();
}

LAB_DBG_CMD((dbg), "dbg", comphy, &comphy_cmds,
	    "A3720 COMPHY lanes: 0 USB3/GbE1, 1 PCIe, 2 SATA/USB3", cmd_comphy, 1, 0, comphy_init,
	    unmap_lanes);
