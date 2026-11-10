/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * dbg netlb: MAC loopback of the Armada 3720 NETA ports, a guess from
 * the same GMAC in mvpp2 (GMAC_CTRL_1 +0x4: bit 5 GMII loopback, bit 6
 * PCS loopback). NETA: GMAC block at +0x2c00, so CTRL_1 = 0x2c04; MIB
 * counters at +0x3000 (clear on read).
 *
 * On ESPRESSObin NETA0 is the only port (RGMII to the switch): with the
 * loopback on, the board is off the network, lab agent and ssh too.
 * Hence "pulse": on for a given time, counters before and after, off.
 * Traffic for the pulse: anything the stack sends meanwhile (ARP, the
 * gPTP / LLDP chatter), or run "net ping" just before.
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/lab/mod.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/sys_io.h>

#include <soc.h>

static const uintptr_t neta_phys[2] = {0xd0030000, 0xd0040000};
static uint8_t *neta[2];
static bool neta_ok[2];
static struct k_work_delayable off_work;
static int off_port = -1;

#define GMAC_CTRL(n)   (0x2c00 + 4 * (n))
#define GMII_LB        BIT(5)
#define PCS_LB         BIT(6)
#define MIB_RX_OCT_LO  0x3000
#define MIB_RX_GOOD    0x3010
#define MIB_TX_OCT_LO  0x3038
#define MIB_TX_GOOD    0x3040

static uint32_t rd(int p, uint32_t off)
{
	return sys_read32((mem_addr_t)neta[p] + off);
}

static void wr(int p, uint32_t off, uint32_t v)
{
	sys_write32(v, (mem_addr_t)neta[p] + off);
}

static int port_arg(const struct shell *sh, const char *s, int *p)
{
	if (strcmp(s, "0") != 0 && strcmp(s, "1") != 0) {
		shell_error(sh, "port 0 or 1");
		return -EINVAL;
	}
	*p = s[0] - '0';
	if (!neta_ok[*p]) {
		shell_error(sh, "neta%d: no access (clock gated?)", *p);
		return -EIO;
	}
	return 0;
}

static int mode_arg(const struct shell *sh, const char *s, uint32_t *bits)
{
	if (strcmp(s, "gmii") == 0) {
		*bits = GMII_LB;
	} else if (strcmp(s, "pcs") == 0) {
		*bits = PCS_LB;
	} else {
		shell_error(sh, "gmii or pcs");
		return -EINVAL;
	}
	return 0;
}

static void lb_off(struct k_work *w)
{
	ARG_UNUSED(w);
	if (off_port >= 0 && neta_ok[off_port]) {
		wr(off_port, GMAC_CTRL(1), rd(off_port, GMAC_CTRL(1)) & ~(GMII_LB | PCS_LB));
		off_port = -1;
	}
}

static int cmd_show(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	for (int p = 0; p < 2; p++) {
		if (!neta_ok[p]) {
			shell_print(sh, "neta%d: no access", p);
			continue;
		}
		uint32_t c1 = rd(p, GMAC_CTRL(1));

		shell_print(sh, "neta%d: ctrl0 %08x ctrl1 %08x ctrl2 %08x ctrl4 %08x  loopback%s%s%s",
			    p, rd(p, GMAC_CTRL(0)), c1, rd(p, GMAC_CTRL(2)), rd(p, 0x2c90),
			    (c1 & GMII_LB) ? " gmii" : "", (c1 & PCS_LB) ? " pcs" : "",
			    (c1 & (GMII_LB | PCS_LB)) ? "" : " off");
	}
	return 0;
}

/* mib <port>: good frames/octets since the last read (counters clear on read) */
static int cmd_mib(const struct shell *sh, size_t argc, char **argv)
{
	int p;

	if (port_arg(sh, argv[1], &p) != 0) {
		return -EINVAL;
	}
	shell_print(sh, "neta%d: rx %u frames %u B, tx %u frames %u B", p, rd(p, MIB_RX_GOOD),
		    rd(p, MIB_RX_OCT_LO), rd(p, MIB_TX_GOOD), rd(p, MIB_TX_OCT_LO));
	return 0;
}

/* pulse <port> gmii|pcs <ms>: loopback for ms, counters during it */
static int cmd_pulse(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long ms = strtoul(argv[3], NULL, 0);
	uint32_t bits, c1, rxf, txf;
	int p;

	if (port_arg(sh, argv[1], &p) != 0 || mode_arg(sh, argv[2], &bits) != 0 || ms == 0 ||
	    ms > 60000) {
		return -EINVAL;
	}
	(void)rd(p, MIB_RX_GOOD);
	(void)rd(p, MIB_TX_GOOD);
	c1 = rd(p, GMAC_CTRL(1));
	wr(p, GMAC_CTRL(1), c1 | bits);
	k_msleep(ms);
	wr(p, GMAC_CTRL(1), c1);
	rxf = rd(p, MIB_RX_GOOD);
	txf = rd(p, MIB_TX_GOOD);
	shell_print(sh, "neta%d %s loopback %lu ms: tx %u frames, rx %u frames%s", p, argv[2], ms,
		    txf, rxf,
		    txf && rxf >= txf ? "  <== looped back?" : (txf ? "" : "  (nothing sent)"));
	return 0;
}

/* on <port> gmii|pcs [ms]: leave it on, off by itself after ms (default 10 s) */
static int cmd_on(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long ms = argc > 3 ? strtoul(argv[3], NULL, 0) : 10000;
	uint32_t bits;
	int p;

	if (port_arg(sh, argv[1], &p) != 0 || mode_arg(sh, argv[2], &bits) != 0) {
		return -EINVAL;
	}
	wr(p, GMAC_CTRL(1), rd(p, GMAC_CTRL(1)) | bits);
	off_port = p;
	k_work_reschedule(&off_work, K_MSEC(ms));
	shell_print(sh, "neta%d %s loopback on, off again in %lu ms", p, argv[2], ms);
	return 0;
}

static int cmd_off(const struct shell *sh, size_t argc, char **argv)
{
	int p;

	if (port_arg(sh, argv[1], &p) != 0) {
		return -EINVAL;
	}
	off_port = p;
	lb_off(NULL);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(netlb_cmds,
	SHELL_CMD_ARG(mib, NULL, "<port>  Good frames/octets since the last read", cmd_mib, 2, 0),
	SHELL_CMD_ARG(pulse, NULL, "<port> gmii|pcs <ms>  Loopback for ms, counters", cmd_pulse,
		      4, 0),
	SHELL_CMD_ARG(on, NULL, "<port> gmii|pcs [ms]  On, automatically off after ms (10 s)",
		      cmd_on, 3, 1),
	SHELL_CMD_ARG(off, NULL, "<port>", cmd_off, 2, 0),
	SHELL_SUBCMD_SET_END);

static int netlb_init(void)
{
	for (int p = 0; p < 2; p++) {
		k_mem_map_phys_bare(&neta[p], neta_phys[p], 0x4000,
				    K_MEM_PERM_RW | K_MEM_CACHE_NONE);
		if (neta[p] == NULL) {
			return -ENOMEM;
		}
		{
			uint32_t v;

			/* an unused, clock gated port aborts */
			neta_ok[p] = a3700_probe_read32((uintptr_t)neta[p] + GMAC_CTRL(0), &v) == 0;
		}
	}
	k_work_init_delayable(&off_work, lb_off);
	return 0;
}

static void netlb_fini(void)
{
	struct k_work_sync sync;

	k_work_cancel_delayable_sync(&off_work, &sync);
	lb_off(NULL);
	for (int p = 0; p < 2; p++) {
		k_mem_unmap_phys_bare(neta[p], 0x4000);
		neta[p] = NULL;
	}
}

LAB_DBG_CMD((dbg), "dbg", netlb, &netlb_cmds,
	    "NETA MAC loopback (GMII/PCS, guessed from mvpp2) and MIB counters", cmd_show, 1, 0,
	    netlb_init, netlb_fini);
