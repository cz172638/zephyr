/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * "mv88e6xxx" shell: raw access to the registers behind the Global 2 AVB
 * command (PTP, AVB, Qav and Qvb blocks), to explore what is not
 * documented, and a summary of the egress shapers.
 *
 *   mv88e6xxx avb <block> <port> <addr> [<value>]
 *       block: ptp, avb, qav, qvb or 0..7
 *       port:  switch port, "g" (block global) or "tai"
 *   mv88e6xxx qav [<port>]
 *   mv88e6xxx reg <addr> <reg> [<value>]   any internal SMI register
 *   mv88e6xxx tcam <entry> [<addr>]
 *       read the three pages of a TCAM entry with the 88E6390 (Peridot)
 *       layout, at SMI address 0x1f unless given: the 88E6341 has a
 *       TCAM, its registers are not documented
 */

#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "phy_marvell_mv88e6xxx.h"

#define SW_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(marvell_mv88e6085)

static const struct device *const sw = DEVICE_DT_GET(SW_NODE);

static int parse_block(const char *arg)
{
	static const char *const names[] = {"ptp", "avb", "qav", "qvb"};
	char *end;
	long v;

	for (int i = 0; i < ARRAY_SIZE(names); i++) {
		if (strcmp(arg, names[i]) == 0) {
			return i;
		}
	}
	v = strtol(arg, &end, 0);

	return (*end == '\0' && v >= 0 && v <= 7) ? v : -EINVAL;
}

static int parse_port(const char *arg, int *port)
{
	char *end;

	if (strcmp(arg, "g") == 0) {
		*port = MV88E6XXX_AVB_PORT_GLOBAL;
		return 0;
	}
	if (strcmp(arg, "tai") == 0) {
		*port = MV88E6XXX_AVB_PORT_TAI;
		return 0;
	}
	*port = strtol(arg, &end, 0);

	return (*end == '\0' && *port >= 0 && *port < 0x1f) ? 0 : -EINVAL;
}

static int cmd_avb(const struct shell *sh, size_t argc, char **argv)
{
	int block = parse_block(argv[1]);
	int port, ret;
	uint8_t addr = strtoul(argv[3], NULL, 0) & 0x1f;
	uint16_t v[4];

	if (!device_is_ready(sw)) {
		shell_error(sh, "switch not ready");
		return -ENODEV;
	}
	if (block < 0 || parse_port(argv[2], &port) < 0) {
		shell_error(sh, "bad block or port");
		return -EINVAL;
	}

	if (argc == 5) {
		ret = mv88e6xxx_avb_block_write(sw, block, port, addr, strtoul(argv[4], NULL, 0));
		if (ret < 0) {
			shell_error(sh, "write: %d", ret);
			return ret;
		}
	}

	/* Four words from addr (read-increment) */
	ret = mv88e6xxx_avb_block_read(sw, block, port, addr, v, ARRAY_SIZE(v));
	if (ret < 0) {
		shell_error(sh, "read: %d", ret);
		return ret;
	}
	shell_print(sh, "block %d port %s addr 0x%02x: %04x %04x %04x %04x", block, argv[2], addr,
		    v[0], v[1], v[2], v[3]);

	return 0;
}

static void qav_port(const struct shell *sh, int port)
{
	uint16_t q[8], cfg, ctl2;

	if (mv88e6xxx_avb_block_read(sw, MV88E6XXX_AVB_BLOCK_QAV, port, 0, &q[0], 4) < 0 ||
	    mv88e6xxx_avb_block_read(sw, MV88E6XXX_AVB_BLOCK_QAV, port, 4, &q[4], 4) < 0 ||
	    mv88e6xxx_avb_block_read(sw, MV88E6XXX_AVB_BLOCK_AVB, port, 0, &cfg, 1) < 0 ||
	    mv88e6xxx_reg_rmw(sw, MV88E6XXX_ADDR_PORT(port), 0x0a, 0, 0, &ctl2) < 0) {
		shell_error(sh, "port %d: read failed", port);
		return;
	}
	shell_print(sh, "%4d  %04x/%04x %04x/%04x %04x/%04x %04x/%04x  %04x  %u", port, q[0],
		    q[1], q[2], q[3], q[4], q[5], q[6], q[7], cfg, (ctl2 >> 12) & 3);
}

static int cmd_qav(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t g[4], avb, pri;
	int port;

	if (!device_is_ready(sw)) {
		shell_error(sh, "switch not ready");
		return -ENODEV;
	}
	if (mv88e6xxx_avb_block_read(sw, MV88E6XXX_AVB_BLOCK_QAV, MV88E6XXX_AVB_PORT_GLOBAL, 0, g,
				     4) < 0 ||
	    mv88e6xxx_avb_block_read(sw, MV88E6XXX_AVB_BLOCK_AVB, MV88E6XXX_AVB_PORT_GLOBAL, 0,
				     &avb, 1) < 0 ||
	    mv88e6xxx_reg_rmw(sw, MV88E6XXX_ADDR_GLOBAL1, 0x18, 0, 0, &pri) < 0) {
		shell_error(sh, "read failed");
		return -EIO;
	}
	shell_print(sh, "Qav global %04x %04x %04x %04x, AVB global %04x, IEEE-PRI %04x", g[0],
		    g[1], g[2], g[3], avb, pri);
	shell_print(sh, "port  rate/hilimit q0..q3                    avb   sched");

	if (argc == 2) {
		if (parse_port(argv[1], &port) < 0) {
			return -EINVAL;
		}
		qav_port(sh, port);
		return 0;
	}
	for (port = 0; port < 6; port++) {
		qav_port(sh, port);
	}

	return 0;
}

static int cmd_reg(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t addr = strtoul(argv[1], NULL, 0) & 0x1f;
	uint8_t reg = strtoul(argv[2], NULL, 0) & 0x1f;
	uint16_t v;
	int ret;

	if (!device_is_ready(sw)) {
		return -ENODEV;
	}
	if (argc == 4) {
		ret = mv88e6xxx_reg_rmw(sw, addr, reg, 0xffff, strtoul(argv[3], NULL, 0), NULL);
		if (ret < 0) {
			shell_error(sh, "write: %d", ret);
			return ret;
		}
	}
	ret = mv88e6xxx_reg_rmw(sw, addr, reg, 0, 0, &v);
	if (ret < 0) {
		shell_error(sh, "read: %d", ret);
		return ret;
	}
	shell_print(sh, "0x%02x/0x%02x = 0x%04x", addr, reg, v);

	return 0;
}

/* Linux tcam.c: operation register 0, busy bit 15, read = 5 << 12,
 * page in bits 11:10, entry in bits 7:0
 */
#define TCAM_OP_BUSY BIT(15)
#define TCAM_OP_READ 0x5000

static int tcam_wait(uint8_t addr, uint16_t *op)
{
	for (int i = 0; i < 100; i++) {
		int ret = mv88e6xxx_reg_rmw(sw, addr, 0, 0, 0, op);

		if (ret < 0) {
			return ret;
		}
		if ((*op & TCAM_OP_BUSY) == 0) {
			return 0;
		}
		k_busy_wait(10);
	}

	return -ETIMEDOUT;
}

static int cmd_tcam(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t entry = strtoul(argv[1], NULL, 0);
	uint8_t addr = argc > 2 ? strtoul(argv[2], NULL, 0) & 0x1f : 0x1f;
	uint16_t op, v[32];
	int ret;

	if (!device_is_ready(sw)) {
		return -ENODEV;
	}

	ret = tcam_wait(addr, &op);
	shell_print(sh, "TCAM at 0x%02x: op 0x%04x%s", addr, op,
		    ret == -ETIMEDOUT ? " (busy, not a TCAM?)" : "");
	if (ret < 0) {
		return ret;
	}

	for (int page = 0; page < 3; page++) {
		ret = mv88e6xxx_reg_rmw(sw, addr, 0, 0xffff,
					TCAM_OP_BUSY | TCAM_OP_READ | (page << 10) | entry, NULL);
		if (ret == 0) {
			ret = tcam_wait(addr, &op);
		}
		for (int r = 0; ret == 0 && r < ARRAY_SIZE(v); r++) {
			ret = mv88e6xxx_reg_rmw(sw, addr, r, 0, 0, &v[r]);
		}
		if (ret < 0) {
			shell_error(sh, "page %d: %d", page, ret);
			return ret;
		}
		shell_print(sh, "entry %u page %d:", entry, page);
		for (int r = 0; r < ARRAY_SIZE(v); r += 8) {
			shell_print(sh, "  %02x: %04x %04x %04x %04x %04x %04x %04x %04x", r, v[r],
				    v[r + 1], v[r + 2], v[r + 3], v[r + 4], v[r + 5], v[r + 6],
				    v[r + 7]);
		}
	}

	/* Port priority override: TCAM mode in bits 1:0 on the 88E6390 */
	for (uint8_t port = 0; port < 6; port++) {
		if (mv88e6xxx_reg_rmw(sw, MV88E6XXX_ADDR_PORT(port), 0x0d, 0, 0, &v[port]) < 0) {
			v[port] = 0xffff;
		}
	}
	shell_print(sh, "port 0x0d (pri override): %04x %04x %04x %04x %04x %04x", v[0], v[1],
		    v[2], v[3], v[4], v[5]);

	return 0;
}

/* clang-format off */
SHELL_STATIC_SUBCMD_SET_CREATE(mv88e6xxx_cmds,
	SHELL_CMD_ARG(avb, NULL, "<ptp|avb|qav|qvb|0-7> <port|g|tai> <addr> [<value>]: "
		      "AVB block registers (4 words)", cmd_avb, 4, 1),
	SHELL_CMD_ARG(qav, NULL, "[<port>]: shaper registers, AVB port config, scheduling",
		      cmd_qav, 1, 1),
	SHELL_CMD_ARG(reg, NULL, "<smi addr> <reg> [<value>]: switch register", cmd_reg, 3, 1),
	SHELL_CMD_ARG(tcam, NULL, "<entry> [<smi addr>]: TCAM entry pages (88E6390 layout)",
		      cmd_tcam, 2, 1),
	SHELL_SUBCMD_SET_END);
/* clang-format on */

SHELL_CMD_REGISTER(mv88e6xxx, &mv88e6xxx_cmds, "Marvell 88E6xxx switch AVB/Qav registers", NULL);
