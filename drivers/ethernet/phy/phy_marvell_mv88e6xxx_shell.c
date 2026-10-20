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
 */

#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
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

/* clang-format off */
SHELL_STATIC_SUBCMD_SET_CREATE(mv88e6xxx_cmds,
	SHELL_CMD_ARG(avb, NULL, "<ptp|avb|qav|qvb|0-7> <port|g|tai> <addr> [<value>]: "
		      "AVB block registers (4 words)", cmd_avb, 4, 1),
	SHELL_CMD_ARG(qav, NULL, "[<port>]: shaper registers, AVB port config, scheduling",
		      cmd_qav, 1, 1),
	SHELL_SUBCMD_SET_END);
/* clang-format on */

SHELL_CMD_REGISTER(mv88e6xxx, &mv88e6xxx_cmds, "Marvell 88E6xxx switch AVB/Qav registers", NULL);
