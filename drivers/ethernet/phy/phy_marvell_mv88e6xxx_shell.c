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
 *   mv88e6xxx tcam dump <entry> [<addr>]
 *       read the three pages of a TCAM entry with the 88E6390 (Peridot)
 *       layout, at SMI address 0x1f unless given
 *   mv88e6xxx tcam add <id> <prio> <ports> <action> [<match> ...]
 *       ports:  all, or switch port numbers / interface names, "1,lan0"
 *       action: permit | drop | trap | redirect <ports> | mirror <ports>
 *       match:  ethertype <n> | vid <n> | smac <mac> | dmac <mac> |
 *               proto <n|icmp|tcp|udp> | src <ip>[/len] | dst <ip>[/len] |
 *               sport <n> | dport <n> | byte <offset> <value> [<mask>]
 *   mv88e6xxx tcam del <id> | list | flush
 */

#include <stdio.h>
#include <stdlib.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "phy_marvell_mv88e6xxx.h"

#if defined(CONFIG_PHY_MARVELL_MV88E6XXX_TCAM)
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/ethernet.h>
#if defined(CONFIG_NET_DSA)
#include <zephyr/net/dsa_core.h>
#endif
#include "phy_marvell_mv88e6xxx_tcam.h"
#endif

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

#if defined(CONFIG_PHY_MARVELL_MV88E6XXX_TCAM)
/* "1,2,lan0" or "all" to a port vector */
static int parse_ports(const struct shell *sh, const char *arg, uint16_t *ports)
{
	char buf[64];
	char *tok, *save;

	*ports = 0;
	if (strcmp(arg, "all") == 0) {
		*ports = GENMASK(10, 0) & ~BIT(mv88e6xxx_cpu_port(sw));
		return 0;
	}

	strncpy(buf, arg, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	for (tok = strtok_r(buf, ",", &save); tok != NULL; tok = strtok_r(NULL, ",", &save)) {
		char *end;
		long p = strtol(tok, &end, 0);

		if (*end == '\0' && p >= 0 && p <= 10) {
			*ports |= BIT(p);
			continue;
		}
#if defined(CONFIG_NET_DSA) && defined(CONFIG_NET_INTERFACE_NAME)
		int idx = net_if_get_by_name(tok);
		struct net_if *iface = idx > 0 ? net_if_get_by_index(idx) : NULL;

		if (iface != NULL && net_if_l2(iface) == &NET_L2_GET_NAME(ETHERNET)) {
			struct ethernet_context *ctx = net_if_l2_data(iface);

			if (ctx->dsa_port == DSA_USER_PORT) {
				const struct dsa_port_config *cfg = net_if_get_device(iface)->config;

				*ports |= BIT(cfg->port_idx);
				continue;
			}
		}
#endif
		shell_error(sh, "unknown port %s", tok);
		return -EINVAL;
	}

	return 0;
}

static int parse_mac(const char *arg, uint8_t mac[6])
{
	unsigned int b[6];

	if (sscanf(arg, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
		return -EINVAL;
	}
	for (int i = 0; i < 6; i++) {
		mac[i] = b[i];
	}

	return 0;
}

static int parse_prefix(const char *arg, uint8_t addr[4], uint8_t mask[4])
{
	char buf[NET_IPV4_ADDR_LEN + 4];
	char *slash;
	struct net_in_addr in;
	int len = 32;

	strncpy(buf, arg, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';
	slash = strchr(buf, '/');
	if (slash != NULL) {
		*slash = '\0';
		len = atoi(slash + 1);
		if (len < 0 || len > 32) {
			return -EINVAL;
		}
	}
	if (net_addr_pton(NET_AF_INET, buf, &in) < 0) {
		return -EINVAL;
	}
	memcpy(addr, in.s4_addr, 4);
	sys_put_be32(len == 0 ? 0 : GENMASK(31, 32 - len), mask);

	return 0;
}

/* Matches on IP fields need IPv4 without options */
static void match_ipv4(struct mv88e6xxx_tcam_rule *r)
{
	static const uint8_t et[2] = {0x08, 0x00};
	static const uint8_t ihl = 0x45, ihl_mask = 0xff;

	mv88e6xxx_tcam_match(r, MV88E6XXX_TCAM_OFF_ETHERTYPE, et, NULL, 2);
	mv88e6xxx_tcam_match(r, MV88E6XXX_TCAM_OFF_IP, &ihl, &ihl_mask, 1);
}

static int parse_action(const struct shell *sh, struct mv88e6xxx_tcam_rule *r, char **argv,
			int argc, int *i)
{
	static const struct {
		const char *name;
		enum mv88e6xxx_tcam_action action;
		bool ports;
	} actions[] = {
		{"permit", MV88E6XXX_TCAM_PERMIT, false},
		{"drop", MV88E6XXX_TCAM_DROP, false},
		{"trap", MV88E6XXX_TCAM_TRAP, false},
		{"redirect", MV88E6XXX_TCAM_REDIRECT, true},
		{"mirror", MV88E6XXX_TCAM_MIRROR, true},
	};

	for (int a = 0; a < ARRAY_SIZE(actions); a++) {
		if (strcmp(argv[*i], actions[a].name) != 0) {
			continue;
		}
		r->action = actions[a].action;
		(*i)++;
		if (actions[a].ports) {
			if (*i >= argc || parse_ports(sh, argv[*i], &r->action_ports) < 0) {
				shell_error(sh, "%s needs ports", actions[a].name);
				return -EINVAL;
			}
			(*i)++;
		}
		return 0;
	}

	shell_error(sh, "unknown action %s", argv[*i]);
	return -EINVAL;
}

static int parse_match(const struct shell *sh, struct mv88e6xxx_tcam_rule *r, char **argv,
		       int argc, int *i)
{
	const char *key = argv[*i];
	const char *val = *i + 1 < argc ? argv[*i + 1] : NULL;
	uint8_t d[6], m[6];

	if (val == NULL) {
		shell_error(sh, "%s needs a value", key);
		return -EINVAL;
	}
	*i += 2;

	if (strcmp(key, "ethertype") == 0) {
		sys_put_be16(strtoul(val, NULL, 0), d);
		mv88e6xxx_tcam_match(r, MV88E6XXX_TCAM_OFF_ETHERTYPE, d, NULL, 2);
	} else if (strcmp(key, "vid") == 0) {
		static const uint8_t vm[2] = {0x0f, 0xff};

		sys_put_be16(strtoul(val, NULL, 0), d);
		mv88e6xxx_tcam_match(r, MV88E6XXX_TCAM_OFF_TAG + 2, d, vm, 2);
	} else if (strcmp(key, "smac") == 0 || strcmp(key, "dmac") == 0) {
		if (parse_mac(val, d) < 0) {
			goto bad;
		}
		mv88e6xxx_tcam_match(r, key[0] == 's' ? MV88E6XXX_TCAM_OFF_SA : MV88E6XXX_TCAM_OFF_DA,
				     d, NULL, 6);
	} else if (strcmp(key, "proto") == 0) {
		d[0] = strcmp(val, "icmp") == 0  ? NET_IPPROTO_ICMP
		       : strcmp(val, "tcp") == 0 ? NET_IPPROTO_TCP
		       : strcmp(val, "udp") == 0 ? NET_IPPROTO_UDP
						 : strtoul(val, NULL, 0);
		match_ipv4(r);
		mv88e6xxx_tcam_match(r, MV88E6XXX_TCAM_OFF_IP_PROTO, d, NULL, 1);
	} else if (strcmp(key, "src") == 0 || strcmp(key, "dst") == 0) {
		if (parse_prefix(val, d, m) < 0) {
			goto bad;
		}
		match_ipv4(r);
		mv88e6xxx_tcam_match(r, key[0] == 's' ? MV88E6XXX_TCAM_OFF_IP_SRC
						      : MV88E6XXX_TCAM_OFF_IP_DST,
				     d, m, 4);
	} else if (strcmp(key, "sport") == 0 || strcmp(key, "dport") == 0) {
		sys_put_be16(strtoul(val, NULL, 0), d);
		match_ipv4(r);
		mv88e6xxx_tcam_match(r, key[0] == 's' ? MV88E6XXX_TCAM_OFF_L4_SPORT
						      : MV88E6XXX_TCAM_OFF_L4_DPORT,
				     d, NULL, 2);
	} else if (strcmp(key, "byte") == 0) {
		int off = strtol(val, NULL, 0);

		if (*i >= argc || off < 0 || off >= MV88E6XXX_TCAM_KEY_SIZE) {
			goto bad;
		}
		d[0] = strtoul(argv[*i], NULL, 0);
		(*i)++;
		m[0] = 0xff;
		if (*i < argc && (argv[*i][0] >= '0' && argv[*i][0] <= '9')) {
			m[0] = strtoul(argv[*i], NULL, 0);
			(*i)++;
		}
		mv88e6xxx_tcam_match(r, off, d, m, 1);
	} else {
		shell_error(sh, "unknown match %s", key);
		return -EINVAL;
	}

	return 0;
bad:
	shell_error(sh, "bad %s %s", key, val);
	return -EINVAL;
}

static int cmd_tcam_add(const struct shell *sh, size_t argc, char **argv)
{
	struct mv88e6xxx_tcam_rule r = {0};
	int i = 4, ret;

	if (!device_is_ready(sw)) {
		return -ENODEV;
	}

	r.id = strtoul(argv[1], NULL, 0);
	r.prio = strtoul(argv[2], NULL, 0);
	if (parse_ports(sh, argv[3], &r.ports) < 0 || r.ports == 0) {
		return -EINVAL;
	}
	if (parse_action(sh, &r, argv, argc, &i) < 0) {
		return -EINVAL;
	}
	while (i < (int)argc) {
		if (parse_match(sh, &r, argv, argc, &i) < 0) {
			return -EINVAL;
		}
	}

	ret = mv88e6xxx_tcam_add(sw, &r);
	if (ret < 0) {
		shell_error(sh, "add: %d", ret);
	}

	return ret;
}

static int cmd_tcam_del(const struct shell *sh, size_t argc, char **argv)
{
	int ret = mv88e6xxx_tcam_del(sw, strtoul(argv[1], NULL, 0));

	if (ret < 0) {
		shell_error(sh, "del: %d", ret);
	}

	return ret;
}

static int cmd_tcam_flush(const struct shell *sh, size_t argc, char **argv)
{
	return mv88e6xxx_tcam_flush(sw);
}

static void list_cb(const struct mv88e6xxx_tcam_rule *r, int hw_idx, void *user_data)
{
	static const char *const names[] = {"permit", "drop", "trap", "redirect", "mirror"};
	const struct shell *sh = user_data;
	char key[3 * MV88E6XXX_TCAM_KEY_SIZE + 1];
	int n = 0;

	for (int i = 0; i < MV88E6XXX_TCAM_KEY_SIZE; i++) {
		if (r->mask[i] != 0) {
			n += snprintf(&key[n], sizeof(key) - n, " %d=%02x/%02x", i, r->data[i],
				      r->mask[i]);
			if (n >= sizeof(key)) {
				break;
			}
		}
	}
	key[MIN(n, (int)sizeof(key) - 1)] = '\0';

	shell_print(sh, "%3d  id %u prio %u ports 0x%03x %s 0x%03x:%s", hw_idx, r->id, r->prio,
		    r->ports, names[r->action], r->action_ports, n ? key : " any");
}

static int cmd_tcam_list(const struct shell *sh, size_t argc, char **argv)
{
	mv88e6xxx_tcam_foreach(sw, list_cb, (void *)sh);

	return 0;
}
#endif /* CONFIG_PHY_MARVELL_MV88E6XXX_TCAM */

/* clang-format off */
SHELL_STATIC_SUBCMD_SET_CREATE(tcam_cmds,
	SHELL_CMD_ARG(dump, NULL, "<entry> [<smi addr>]: raw pages of an entry", cmd_tcam, 2, 1),
#if defined(CONFIG_PHY_MARVELL_MV88E6XXX_TCAM)
	SHELL_CMD_ARG(add, NULL,
		      "<id> <prio> <ports> <permit|drop|trap|redirect <p>|mirror <p>> [match...]: "
		      "ethertype, vid, smac, dmac, proto, src, dst, sport, dport, byte",
		      cmd_tcam_add, 5, 30),
	SHELL_CMD_ARG(del, NULL, "<id>", cmd_tcam_del, 2, 0),
	SHELL_CMD_ARG(list, NULL, "rules in hardware order", cmd_tcam_list, 1, 0),
	SHELL_CMD_ARG(flush, NULL, "remove all rules", cmd_tcam_flush, 1, 0),
#endif
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(mv88e6xxx_cmds,
	SHELL_CMD_ARG(avb, NULL, "<ptp|avb|qav|qvb|0-7> <port|g|tai> <addr> [<value>]: "
		      "AVB block registers (4 words)", cmd_avb, 4, 1),
	SHELL_CMD_ARG(qav, NULL, "[<port>]: shaper registers, AVB port config, scheduling",
		      cmd_qav, 1, 1),
	SHELL_CMD_ARG(reg, NULL, "<smi addr> <reg> [<value>]: switch register", cmd_reg, 3, 1),
	SHELL_CMD(tcam, &tcam_cmds, "TCAM (ingress ACL)", NULL),
	SHELL_SUBCMD_SET_END);
/* clang-format on */

SHELL_CMD_REGISTER(mv88e6xxx, &mv88e6xxx_cmds, "Marvell 88E6xxx switch AVB/Qav registers", NULL);
