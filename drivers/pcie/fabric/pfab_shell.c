/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * pcie fabric ...: the local node (endpoint function or the root
 * complex's node) and, on a root complex, the manager.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/pcie/fabric.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>
#ifdef CONFIG_PCIE_FABRIC_MGR
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pcie/pcie.h>
#endif

static struct pfab_node *local(const struct shell *sh)
{
	struct pfab_node *n;

	for (int i = 0; (n = pfab_node_get(i)) != NULL; i++) {
		if (n->running) {
			return n;
		}
	}
	n = pfab_node_get(0);
	if (n == NULL) {
		shell_error(sh, "no fabric node here (endpoint function not bound, manager not up)");
	}
	return n;
}

static int num(const struct shell *sh, const char *s, unsigned long long *v)
{
	char *e;

	*v = strtoull(s, &e, 0);
	if (*e != '\0') {
		shell_error(sh, "bad number: %s", s);
		return -EINVAL;
	}
	return 0;
}

static uint16_t role_parse(const char *s)
{
	for (uint16_t r = 0; r < 5; r++) {
		if (strcmp(s, pfab_role_name(r)) == 0) {
			return r;
		}
	}
	return (uint16_t)strtoul(s, NULL, 0);
}

/* pcie fabric node */
static int cmd_node(const struct shell *sh, size_t argc, char **argv)
{
	struct pfab_node *n;
	const struct pfab_stats *s;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	for (int i = 0; (n = pfab_node_get(i)) != NULL; i++) {
		s = &n->st;
		shell_print(sh, "%s: %s, node %u (%s), epoch %08x, state %s, poll %u us", n->name,
			    n->running ? "running" : "stopped", n->id, pfab_role_name(n->role),
			    n->epoch, pfab_state_name(pfab_rd(n->local, PFAB_INFO_STATE)),
			    n->poll_us);
		shell_print(sh, "  region 0x%x: %u nodes x %u slots x %u B, RMA 0x%x at +0x%x, "
			    "DSN %016llx", n->g.size, n->g.max_nodes, n->g.nslots, n->g.slot_size,
			    n->g.rma_size, n->g.rma_off,
			    (unsigned long long)pfab_rd64(n->local, PFAB_INFO_DSN));
		if ((int32_t)pfab_rd(n->local, PFAB_INFO_ERROR) != 0) {
			shell_print(sh, "  last card refused: %d",
				    (int32_t)pfab_rd(n->local, PFAB_INFO_ERROR));
		}
		for (int k = 0; k < ARRAY_SIZE(n->peer); k++) {
			const struct pfab_peer *p = &n->peer[k];

			if (!p->valid) {
				continue;
			}
			shell_print(sh, "  peer %u %-8s bus 0x%llx (0x%x), DSN %016llx, sent %u, "
				    "credit %u, taken %u", p->id, pfab_role_name(p->role),
				    (unsigned long long)p->bus, p->g.size,
				    (unsigned long long)p->dsn, p->sent,
				    pfab_rd(n->local, n->g.cred_off + 4U * p->id), n->rx_count[k]);
		}
		shell_print(sh, "  tx %u (ring full %u), rx %u (stale %u, dropped %u), echo %u, "
			    "puts %u, cards %u", s->tx, s->tx_full, s->rx, s->rx_stale,
			    s->rx_dropped, s->echo, s->puts, s->cards);
	}
	if (pfab_node_get(0) == NULL) {
		shell_print(sh, "no fabric node here");
	}
	return 0;
}

/* pcie fabric ping <node> [count] [len] */
static int cmd_ping(const struct shell *sh, size_t argc, char **argv)
{
	struct pfab_node *n = local(sh);
	unsigned long long dst, count = 10, len = 16;
	struct pfab_ping_result r;
	int ret;

	if (n == NULL || num(sh, argv[1], &dst) != 0 ||
	    (argc > 2 && num(sh, argv[2], &count) != 0) ||
	    (argc > 3 && num(sh, argv[3], &len) != 0)) {
		return -EINVAL;
	}
	ret = pfab_ping(n, (uint16_t)dst, (uint32_t)count, (size_t)len, K_MSEC(1000), &r);
	if (ret != 0 && r.sent == 0) {
		shell_error(sh, "send to node %llu: %d", dst, ret);
		return ret;
	}
	shell_print(sh, "node %llu: %u sent, %u back, %llu B", dst, r.sent, r.received, len);
	if (r.received != 0) {
		shell_print(sh, "round trip min %llu.%03llu us, avg %llu.%03llu us, "
			    "max %llu.%03llu us", r.min_ns / 1000, r.min_ns % 1000,
			    (r.sum_ns / r.received) / 1000, (r.sum_ns / r.received) % 1000,
			    r.max_ns / 1000, r.max_ns % 1000);
	}
	return 0;
}

/* pcie fabric send <node> <text...> */
static int cmd_send(const struct shell *sh, size_t argc, char **argv)
{
	struct pfab_node *n = local(sh);
	unsigned long long dst;
	char buf[PFAB_PAYLOAD_MAX];
	size_t len = 0;
	int ret;

	if (n == NULL || num(sh, argv[1], &dst) != 0) {
		return -EINVAL;
	}
	for (size_t i = 2; i < argc && len < sizeof(buf); i++) {
		int w = snprintf(&buf[len], sizeof(buf) - len, "%s%s", i > 2 ? " " : "", argv[i]);

		len = MIN(len + w, sizeof(buf));
	}
	ret = pfab_send(n, (uint16_t)dst, PFAB_TYPE_USER, buf, len);
	if (ret != 0) {
		shell_error(sh, "send: %d", ret);
	}
	return ret;
}

/* pcie fabric recv [ms] */
static int cmd_recv(const struct shell *sh, size_t argc, char **argv)
{
	struct pfab_node *n = local(sh);
	unsigned long long ms = 0;
	struct pfab_msg m;
	int got = 0;

	if (n == NULL || (argc > 1 && num(sh, argv[1], &ms) != 0)) {
		return -EINVAL;
	}
	while (pfab_recv(n, &m, got == 0 ? K_MSEC(ms) : K_NO_WAIT) == 0) {
		bool text = true;

		for (int i = 0; i < m.len; i++) {
			text &= m.data[i] >= 0x20 && m.data[i] < 0x7f;
		}
		if (text) {
			shell_print(sh, "from %u type 0x%04x: \"%.*s\"", m.src, m.type, m.len,
				    (const char *)m.data);
		} else {
			shell_print(sh, "from %u type 0x%04x, %u B:", m.src, m.type, m.len);
			shell_hexdump(sh, m.data, m.len);
		}
		got++;
	}
	if (got == 0) {
		shell_print(sh, "nothing");
	}
	return 0;
}

/* pcie fabric put <node> <off> <word>... */
static int cmd_put(const struct shell *sh, size_t argc, char **argv)
{
	struct pfab_node *n = local(sh);
	unsigned long long dst, off, v;
	uint32_t w[16];
	size_t k = 0;
	int ret;

	if (n == NULL || num(sh, argv[1], &dst) != 0 || num(sh, argv[2], &off) != 0) {
		return -EINVAL;
	}
	for (size_t i = 3; i < argc && k < ARRAY_SIZE(w); i++) {
		if (num(sh, argv[i], &v) != 0) {
			return -EINVAL;
		}
		w[k++] = (uint32_t)v;
	}
	ret = pfab_put(n, (uint16_t)dst, (uint32_t)off, w, k * 4);
	if (ret != 0) {
		shell_error(sh, "put: %d", ret);
	}
	return ret;
}

/* pcie fabric rma [off] [len]: our window */
static int cmd_rma(const struct shell *sh, size_t argc, char **argv)
{
	struct pfab_node *n = local(sh);
	unsigned long long off = 0, len = 64;
	volatile uint8_t *w;
	size_t sz;

	if (n == NULL || (argc > 1 && num(sh, argv[1], &off) != 0) ||
	    (argc > 2 && num(sh, argv[2], &len) != 0)) {
		return -EINVAL;
	}
	w = pfab_rma_local(n, &sz);
	off &= ~3ULL;
	if (off >= sz) {
		shell_error(sh, "window is 0x%zx bytes", sz);
		return -ERANGE;
	}
	len = MIN(len, sz - off);
	for (size_t o = 0; o < len; o += 16) {
		char line[48];
		int p = 0;

		for (size_t i = o; i < o + 16 && i < len; i += 4) {
			p += snprintf(&line[p], sizeof(line) - p, " %08x",
				      pfab_rd(w, (uint32_t)(off + i)));
		}
		shell_print(sh, "%06llx:%s", off + o, line);
	}
	return 0;
}

/* pcie fabric poll [us] */
static int cmd_poll(const struct shell *sh, size_t argc, char **argv)
{
	struct pfab_node *n;
	unsigned long long us;

	if (argc > 1) {
		if (num(sh, argv[1], &us) != 0) {
			return -EINVAL;
		}
		for (int i = 0; (n = pfab_node_get(i)) != NULL; i++) {
			n->poll_us = (uint32_t)us;
		}
	}
	n = pfab_node_get(0);
	shell_print(sh, "service poll %u us%s", n != NULL ? n->poll_us : 0,
		    n != NULL && n->poll_us == 0 ? " (busy)" : "");
	return 0;
}

SHELL_SUBCMD_ADD((pcie_fabric), node, NULL, "Local node: ID, role, peers, counters", cmd_node,
		 1, 0);
SHELL_SUBCMD_ADD((pcie_fabric), ping, NULL, "<node> [count] [len] Round trip (echo)", cmd_ping,
		 2, 2);
SHELL_SUBCMD_ADD((pcie_fabric), send, NULL, "<node> <text...> Message", cmd_send, 3, 14);
SHELL_SUBCMD_ADD((pcie_fabric), recv, NULL, "[ms] Messages received", cmd_recv, 1, 1);
SHELL_SUBCMD_ADD((pcie_fabric), put, NULL, "<node> <off> <word>... Write into a peer's RMA window",
		 cmd_put, 4, 15);
SHELL_SUBCMD_ADD((pcie_fabric), rma, NULL, "[off] [len] Our RMA window", cmd_rma, 1, 2);
SHELL_SUBCMD_ADD((pcie_fabric), poll, NULL, "[us] Service thread poll interval (0 busy)",
		 cmd_poll, 1, 1);

#ifdef CONFIG_PCIE_FABRIC_MGR
#define MGR_CTRL DEVICE_DT_GET(DT_CHOSEN(zephyr_pcie_controller))

/* pcie fabric ls: what the manager found */
static int cmd_ls(const struct shell *sh, size_t argc, char **argv)
{
	const struct pfab_mgr_ent *e;
	struct pfab_node *rc = pfab_mgr_node();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "epoch %08x", pfab_mgr_epoch());
	if (rc != NULL) {
		shell_print(sh, "  node 0 manager  root complex     %s", rc->running ? "running"
										 : "stopped");
	}
	for (int i = 0; (e = pfab_mgr_get(i)) != NULL; i++) {
		shell_print(sh, "  node %u %-8s %02x:%02x.%x %-8s DSN %016llx BAR 0x%llx+0x%x "
			    "roles 0x%x", e->id, pfab_role_name(e->role), PCIE_BDF_TO_BUS(e->bdf),
			    PCIE_BDF_TO_DEV(e->bdf), PCIE_BDF_TO_FUNC(e->bdf),
			    pfab_state_name(e->state), (unsigned long long)e->dsn,
			    (unsigned long long)e->bus, e->g.size, e->roles);
		shell_print(sh, "         %s, fw %s", e->board, e->fw);
	}
	if (pfab_mgr_get(0) == NULL) {
		shell_print(sh, "  no endpoints (pcie fabric scan)");
	}
	return 0;
}

static int cmd_scan(const struct shell *sh, size_t argc, char **argv)
{
	int n = pfab_mgr_scan(MGR_CTRL, PCIE_BDF(0, 0, 0));

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "%d fabric endpoint(s)", n);
	return n < 0 ? n : 0;
}

static int cmd_up(const struct shell *sh, size_t argc, char **argv)
{
	int n;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	(void)pfab_mgr_scan(MGR_CTRL, PCIE_BDF(0, 0, 0));
	n = pfab_mgr_up(MGR_CTRL, PCIE_BDF(0, 0, 0), K_MSEC(2000));
	shell_print(sh, "epoch %08x: %d endpoint(s) running", pfab_mgr_epoch(), n);
	return n < 0 ? n : 0;
}

static int cmd_down(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	pfab_mgr_down();
	shell_print(sh, "cards withdrawn");
	return 0;
}

/* pcie fabric role <dsn> <role> */
static int cmd_role(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long long dsn;
	uint16_t role;
	int ret;

	ARG_UNUSED(argc);
	if (num(sh, argv[1], &dsn) != 0) {
		return -EINVAL;
	}
	role = role_parse(argv[2]);
	ret = pfab_mgr_role_set(dsn, role);
	if (ret == 0) {
		shell_print(sh, "%016llx: %s at the next pcie fabric up", dsn,
			    pfab_role_name(role));
	}
	return ret;
}

SHELL_SUBCMD_ADD((pcie_fabric), ls, NULL, "Manager: nodes, roles, state", cmd_ls, 1, 0);
SHELL_SUBCMD_ADD((pcie_fabric), scan, NULL, "Manager: find the fabric endpoints", cmd_scan, 1,
		 0);
SHELL_SUBCMD_ADD((pcie_fabric), up, NULL, "Manager: hand out the cards (new epoch)", cmd_up, 1,
		 0);
SHELL_SUBCMD_ADD((pcie_fabric), down, NULL, "Manager: withdraw the cards", cmd_down, 1, 0);
SHELL_SUBCMD_ADD((pcie_fabric), role, NULL,
		 "<dsn> <none|manager|compute|storage|io|n> Role for the next up", cmd_role, 3, 0);
#endif
