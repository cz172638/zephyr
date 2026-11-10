/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * pcie debug loopback: hunting the Aardvark's loopback controls.
 *
 * The LTSSM has the states (CFG_REG 0x6000 [29:24], names from Linux):
 * 0x21-0x23 loopback master entry/active/exit, 0x24-0x26 slave. How to
 * make the core enter master loopback is undocumented; candidates are
 * the unnamed bits of CTRL0..2 (0x4800-0x4808) and the LMI block.
 * "try" sets one bit, traces the local LTSSM and, with "peer", asks the
 * other board (lab agent, same module loaded there) where its LTSSM is:
 * 0x24/0x25 on the peer = it was told to loop back.
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

#define ADVK_PHYS 0xd0070000UL
#define ADVK_SIZE 0x20000
#define CFG_REG   0x6000
#define TRACE_MAX 64

static uint8_t *advk;

static const char *const ltssm_name[0x2b] = {
	[0x00] = "detect-quiet", [0x01] = "detect-active", [0x02] = "polling-active",
	[0x03] = "polling-compliance", [0x04] = "polling-config", [0x05] = "config-lw-start",
	[0x06] = "config-lw-accept", [0x07] = "config-ln-accept", [0x08] = "config-ln-wait",
	[0x09] = "config-complete", [0x0a] = "config-idle", [0x0b] = "recovery-rcvr-lock",
	[0x0c] = "recovery-speed", [0x0d] = "recovery-rcvr-cfg", [0x0e] = "recovery-idle",
	[0x10] = "L0", [0x11] = "rx-L0s-entry", [0x12] = "rx-L0s-idle", [0x13] = "rx-L0s-fts",
	[0x14] = "tx-L0s-entry", [0x15] = "tx-L0s-idle", [0x16] = "tx-L0s-fts",
	[0x17] = "L1-entry", [0x18] = "L1-idle", [0x19] = "L2-idle", [0x1a] = "L2-wake",
	[0x20] = "disabled", [0x21] = "LOOPBACK-entry-master", [0x22] = "LOOPBACK-active-master",
	[0x23] = "LOOPBACK-exit-master", [0x24] = "LOOPBACK-entry-slave",
	[0x25] = "LOOPBACK-active-slave", [0x26] = "LOOPBACK-exit-slave", [0x27] = "hot-reset",
	[0x28] = "recovery-eq0", [0x29] = "recovery-eq1", [0x2a] = "recovery-eq2",
};

static uint32_t rd(uint32_t off)
{
	return sys_read32((mem_addr_t)advk + off);
}

static void wr(uint32_t off, uint32_t v)
{
	sys_write32(v, (mem_addr_t)advk + off);
}

static uint8_t ltssm(void)
{
	return (rd(CFG_REG) >> 24) & 0x3f;
}

static const char *name(uint8_t s)
{
	return s < ARRAY_SIZE(ltssm_name) && ltssm_name[s] ? ltssm_name[s] : "?";
}

/* states seen within ms, busy polling */
static int trace(const struct shell *sh, uint32_t ms, bool print, bool *loop_seen)
{
	uint8_t seen[TRACE_MAX];
	uint32_t at[TRACE_MAX];
	int n = 0;
	int64_t t0 = k_uptime_get();
	uint64_t c0 = k_cycle_get_64();
	uint8_t last = 0xff;

	while (k_uptime_get() - t0 < ms) {
		uint8_t s = ltssm();

		if (s != last && n < TRACE_MAX) {
			seen[n] = s;
			at[n] = (uint32_t)(k_cyc_to_us_floor64(k_cycle_get_64() - c0));
			n++;
			last = s;
		}
		k_busy_wait(2);
	}
	for (int i = 0; i < n; i++) {
		if (seen[i] >= 0x21 && seen[i] <= 0x26 && loop_seen != NULL) {
			*loop_seen = true;
		}
		if (print) {
			shell_print(sh, "%8u us  0x%02x %s", at[i], seen[i], name(seen[i]));
		}
	}
	return n;
}

static int num(const struct shell *sh, const char *s, unsigned long *v)
{
	char *end;

	*v = strtoul(s, &end, 0);
	if (*end != '\0') {
		shell_error(sh, "bad number '%s'", s);
		return -EINVAL;
	}
	return 0;
}

/* ltssm [ms]: now, or transitions during ms */
static int cmd_ltssm(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long ms = 0;

	if (argc > 1 && num(sh, argv[1], &ms) != 0) {
		return -EINVAL;
	}
	if (ms == 0) {
		uint8_t s = ltssm();

		shell_print(sh, "ltssm 0x%02x %s (%s)", s, name(s),
			    advk_mode_get() == ADVK_MODE_RC ? "rc" : "ep");
		return 0;
	}
	trace(sh, ms, true, NULL);
	return 0;
}

static uint8_t peer_ltssm(void)
{
	char out[96];
	char *p;

	if (lab_peer_exec("pcie debug loopback ltssm", out, sizeof(out), K_SECONDS(2)) != 0) {
		return 0xff;
	}
	p = strstr(out, "ltssm 0x");
	return p ? (uint8_t)strtoul(p + 6, NULL, 16) : 0xfe;
}

static bool wait_l0(uint32_t ms)
{
	int64_t t0 = k_uptime_get();

	while (k_uptime_get() - t0 < ms) {
		if (ltssm() == 0x10) {
			return true;
		}
		k_msleep(1);
	}
	return false;
}

/* one bit: set, watch, ask the peer, clear, wait for L0 again */
static int try_bit(const struct shell *sh, uint32_t off, int bit, uint32_t ms, bool peer,
		   bool verbose)
{
	uint32_t old = rd(off);
	bool loop = false;
	uint8_t p = 0xff;
	int n;

	wr(off, old ^ BIT(bit));
	n = trace(sh, ms, verbose, &loop);
	if (peer) {
		p = peer_ltssm();
	}
	wr(off, old);
	shell_print(sh, "%05x bit %2d %s: %d states%s%s%s%s", off, bit,
		    (old & BIT(bit)) ? "cleared" : "set", n, loop ? ", LOCAL LOOPBACK" : "",
		    peer ? ", peer " : "", peer ? name(p) : "",
		    (peer && p >= 0x21 && p <= 0x26) ? " <== PEER LOOPS BACK" : "");
	if (!wait_l0(1000)) {
		shell_warn(sh, "link not back in L0 (0x%02x %s); retrain / hot reset", ltssm(),
			   name(ltssm()));
		return -EIO;
	}
	return 0;
}

/* try <off> <bit> [ms] [peer] */
static int cmd_try(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long off, bit, ms = 200;
	bool peer = strcmp(argv[argc - 1], "peer") == 0;

	if (peer) {
		argc--;
	}
	if (num(sh, argv[1], &off) != 0 || num(sh, argv[2], &bit) != 0 || bit > 31 ||
	    off >= ADVK_SIZE || (off & 3) || (argc > 3 && num(sh, argv[3], &ms) != 0)) {
		return -EINVAL;
	}
	return try_bit(sh, off, (int)bit, ms, peer, true);
}

/* scan <off> <mask> [ms] [peer]: try every bit of mask, stop when the link stays down */
static int cmd_scan(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long off, mask, ms = 100;
	bool peer = strcmp(argv[argc - 1], "peer") == 0;

	if (peer) {
		argc--;
	}
	if (num(sh, argv[1], &off) != 0 || num(sh, argv[2], &mask) != 0 || off >= ADVK_SIZE ||
	    (off & 3) || (argc > 3 && num(sh, argv[3], &ms) != 0)) {
		return -EINVAL;
	}
	if (ltssm() != 0x10) {
		shell_error(sh, "link not in L0 (0x%02x %s): bring it up first", ltssm(),
			    name(ltssm()));
		return -ENOTCONN;
	}
	for (int b = 0; b < 32; b++) {
		if ((mask & BIT(b)) && try_bit(sh, off, b, ms, peer, false) != 0) {
			return -EIO;
		}
	}
	return 0;
}

/* hold <off> <bit> 0|1: leave a bit as given (manual experiments) */
static int cmd_hold(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long off, bit, val;
	uint32_t old;

	if (num(sh, argv[1], &off) != 0 || num(sh, argv[2], &bit) != 0 ||
	    num(sh, argv[3], &val) != 0 || bit > 31 || off >= ADVK_SIZE || (off & 3)) {
		return -EINVAL;
	}
	old = rd(off);
	wr(off, val ? (old | BIT(bit)) : (old & ~BIT(bit)));
	shell_print(sh, "%05lx: %08x -> %08x", off, old, rd(off));
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(lb_cmds,
	SHELL_CMD_ARG(ltssm, NULL, "[ms]  LTSSM now, or its transitions during ms", cmd_ltssm, 1,
		      1),
	SHELL_CMD_ARG(try, NULL, "<off> <bit> [ms] [peer]  Flip one bit, trace, restore",
		      cmd_try, 3, 2),
	SHELL_CMD_ARG(scan, NULL, "<off> <mask> [ms] [peer]  try for every bit of mask", cmd_scan,
		      3, 2),
	SHELL_CMD_ARG(hold, NULL, "<off> <bit> 0|1  Set a bit and leave it", cmd_hold, 4, 0),
	SHELL_SUBCMD_SET_END);

static int lb_init(void)
{
	k_mem_map_phys_bare(&advk, ADVK_PHYS, ADVK_SIZE, K_MEM_PERM_RW | K_MEM_CACHE_NONE);
	return advk != NULL ? 0 : -ENOMEM;
}

static void lb_fini(void)
{
	k_mem_unmap_phys_bare(advk, ADVK_SIZE);
	advk = NULL;
}

LAB_DBG_CMD((pcie_debug), "pcie", loopback, &lb_cmds,
	    "LTSSM trace and loopback control hunt (try/scan bits, ask the peer)", cmd_ltssm, 1,
	    1, lb_init, lb_fini);
