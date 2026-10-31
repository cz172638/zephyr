/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Saved configuration, as on network gear: changes apply at once and
 * are kept only after "config save", so a rule that locks one out goes
 * away with a reboot.
 *
 *   config save | show | erase
 *   router lan <addr>/<len> | pool <addr> | dns <addr>|auto | show
 *
 * Kept in settings under netshell/cfg/: the router LAN, the TCAM rules
 * (one key per rule) and the Qav state of every interface with Qav.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/ethernet_mgmt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/dhcpv4_server.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

#include "config.h"

#if defined(CONFIG_PHY_MARVELL_MV88E6XXX_TCAM)
#include "phy_marvell_mv88e6xxx_tcam.h"

#define SW_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(marvell_mv88e6085)
static const struct device *const sw = DEVICE_DT_GET(SW_NODE);
#endif

LOG_MODULE_REGISTER(netshell_cfg, LOG_LEVEL_INF);

#define KEY_LAN    "netshell/cfg/lan"
#define KEY_TCAM_N "netshell/cfg/tcam/n"
#define KEY_TCAM   "netshell/cfg/tcam/%d"
#define KEY_QAV    "netshell/cfg/qav/%s"

#define QAV_MAX_QUEUES 8

struct qav_saved {
	uint8_t queues;
	uint8_t enabled;
	uint32_t idle_slope[QAV_MAX_QUEUES];
};

struct netshell_lan netshell_lan;

struct read_ctx {
	void *buf;
	size_t len;
	bool found;
};

static int read_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
		   void *param)
{
	struct read_ctx *r = param;

	ARG_UNUSED(key);

	if (len == r->len && read_cb(cb_arg, r->buf, len) == len) {
		r->found = true;
	}
	return 0;
}

static bool load(const char *key, void *buf, size_t len)
{
	struct read_ctx r = {.buf = buf, .len = len};

	(void)settings_load_subtree_direct(key, read_cb, &r);
	return r.found;
}

void netshell_config_load_lan(void)
{
#if defined(CONFIG_NETSHELL_ROUTER)
	struct netshell_lan saved;

	(void)net_addr_pton(NET_AF_INET, CONFIG_NETSHELL_LAN_ADDR, &netshell_lan.addr);
	(void)net_addr_pton(NET_AF_INET, CONFIG_NETSHELL_LAN_NETMASK, &netshell_lan.mask);
	(void)net_addr_pton(NET_AF_INET, CONFIG_NETSHELL_DHCP_POOL, &netshell_lan.pool);
	netshell_lan.dns.s_addr = 0;

	if (load(KEY_LAN, &saved, sizeof(saved))) {
		netshell_lan = saved;
		LOG_INF("saved LAN configuration");
	}
#endif
}

/* Qav of one interface, through Ethernet management as "mvneta qav" */
static int qav_get(struct net_if *iface, struct qav_saved *q)
{
	struct ethernet_req_params p = {0};

	if (net_if_l2(iface) != &NET_L2_GET_NAME(ETHERNET) ||
	    (net_eth_get_hw_capabilities(iface) & ETHERNET_QAV) == 0 ||
	    net_mgmt(NET_REQUEST_ETHERNET_GET_PRIORITY_QUEUES_NUM, iface, &p, sizeof(p)) < 0) {
		return -ENOTSUP;
	}

	memset(q, 0, sizeof(*q));
	q->queues = MIN(p.priority_queues_num, QAV_MAX_QUEUES);
	for (int i = 0; i < q->queues; i++) {
		p.qav_param.queue_id = i;
		p.qav_param.type = ETHERNET_QAV_PARAM_TYPE_STATUS;
		if (net_mgmt(NET_REQUEST_ETHERNET_GET_QAV_PARAM, iface, &p, sizeof(p)) == 0 &&
		    p.qav_param.enabled) {
			q->enabled |= BIT(i);
		}
		p.qav_param.type = ETHERNET_QAV_PARAM_TYPE_IDLE_SLOPE;
		if (net_mgmt(NET_REQUEST_ETHERNET_GET_QAV_PARAM, iface, &p, sizeof(p)) == 0) {
			q->idle_slope[i] = p.qav_param.idle_slope;
		}
	}

	return 0;
}

static void qav_set(struct net_if *iface, const struct qav_saved *q)
{
	struct ethernet_req_params p = {0};

	for (int i = 0; i < q->queues; i++) {
		p.qav_param.queue_id = i;
		if (q->idle_slope[i] != 0) {
			p.qav_param.type = ETHERNET_QAV_PARAM_TYPE_IDLE_SLOPE;
			p.qav_param.idle_slope = q->idle_slope[i];
			(void)net_mgmt(NET_REQUEST_ETHERNET_SET_QAV_PARAM, iface, &p, sizeof(p));
		}
		p.qav_param.type = ETHERNET_QAV_PARAM_TYPE_STATUS;
		p.qav_param.enabled = (q->enabled & BIT(i)) != 0;
		(void)net_mgmt(NET_REQUEST_ETHERNET_SET_QAV_PARAM, iface, &p, sizeof(p));
	}
}

static void qav_apply_cb(struct net_if *iface, void *user_data)
{
	char name[CONFIG_NET_INTERFACE_NAME_LEN + 1];
	char key[sizeof(KEY_QAV) + CONFIG_NET_INTERFACE_NAME_LEN];
	struct qav_saved q;

	ARG_UNUSED(user_data);

	if (net_if_get_name(iface, name, sizeof(name)) <= 0) {
		return;
	}
	snprintf(key, sizeof(key), KEY_QAV, name);
	if (load(key, &q, sizeof(q)) && q.queues <= QAV_MAX_QUEUES) {
		qav_set(iface, &q);
		LOG_INF("%s: saved Qav (queues 0x%02x)", name, q.enabled);
	}
}

void netshell_config_apply(void)
{
#if defined(CONFIG_PHY_MARVELL_MV88E6XXX_TCAM)
	struct mv88e6xxx_tcam_rule r;
	char key[sizeof(KEY_TCAM) + 12];
	int n = 0, added = 0;

	if (device_is_ready(sw) && load(KEY_TCAM_N, &n, sizeof(n))) {
		for (int i = 0; i < n; i++) {
			snprintf(key, sizeof(key), KEY_TCAM, i);
			if (load(key, &r, sizeof(r)) && mv88e6xxx_tcam_add(sw, &r) == 0) {
				added++;
			}
		}
		LOG_INF("%d of %d saved TCAM rules", added, n);
	}
#endif
	net_if_foreach(qav_apply_cb, NULL);
}

/* "config save" */
struct save_ctx {
	const struct shell *sh;
	int n;
	int ret;
};

#if defined(CONFIG_PHY_MARVELL_MV88E6XXX_TCAM)
static void tcam_save_cb(const struct mv88e6xxx_tcam_rule *rule, int hw_idx, void *user_data)
{
	struct save_ctx *c = user_data;
	char key[sizeof(KEY_TCAM) + 12];

	ARG_UNUSED(hw_idx);

	snprintf(key, sizeof(key), KEY_TCAM, c->n);
	if (settings_save_one(key, rule, sizeof(*rule)) < 0) {
		c->ret = -EIO;
	}
	c->n++;
}
#endif

static void qav_save_cb(struct net_if *iface, void *user_data)
{
	struct save_ctx *c = user_data;
	char name[CONFIG_NET_INTERFACE_NAME_LEN + 1];
	char key[sizeof(KEY_QAV) + CONFIG_NET_INTERFACE_NAME_LEN];
	struct qav_saved q;
	bool any = false;

	if (net_if_get_name(iface, name, sizeof(name)) <= 0 || qav_get(iface, &q) < 0) {
		return;
	}
	for (int i = 0; i < q.queues; i++) {
		any |= q.idle_slope[i] != 0 || (q.enabled & BIT(i));
	}
	snprintf(key, sizeof(key), KEY_QAV, name);
	if (!any) {
		(void)settings_delete(key);
		return;
	}
	if (settings_save_one(key, &q, sizeof(q)) < 0) {
		c->ret = -EIO;
	}
	c->n++;
}

static void tcam_erase(int from)
{
	char key[sizeof(KEY_TCAM) + 12];
	int old = 0;

	(void)load(KEY_TCAM_N, &old, sizeof(old));
	for (int i = from; i < old; i++) {
		snprintf(key, sizeof(key), KEY_TCAM, i);
		(void)settings_delete(key);
	}
}

static int cmd_save(const struct shell *sh, size_t argc, char **argv)
{
	struct save_ctx c = {.sh = sh};

#if defined(CONFIG_NETSHELL_ROUTER)
	if (settings_save_one(KEY_LAN, &netshell_lan, sizeof(netshell_lan)) < 0) {
		c.ret = -EIO;
	}
#endif
#if defined(CONFIG_PHY_MARVELL_MV88E6XXX_TCAM)
	mv88e6xxx_tcam_foreach(sw, tcam_save_cb, &c);
	tcam_erase(c.n);
	if (settings_save_one(KEY_TCAM_N, &c.n, sizeof(c.n)) < 0) {
		c.ret = -EIO;
	}
	shell_print(sh, "TCAM rules: %d", c.n);
#endif
	c.n = 0;
	net_if_foreach(qav_save_cb, &c);
	shell_print(sh, "interfaces with Qav: %d", c.n);

	if (c.ret < 0) {
		shell_error(sh, "saving failed");
	}
	return c.ret;
}

static void qav_erase_cb(struct net_if *iface, void *user_data)
{
	char name[CONFIG_NET_INTERFACE_NAME_LEN + 1];
	char key[sizeof(KEY_QAV) + CONFIG_NET_INTERFACE_NAME_LEN];

	ARG_UNUSED(user_data);

	if (net_if_get_name(iface, name, sizeof(name)) > 0) {
		snprintf(key, sizeof(key), KEY_QAV, name);
		(void)settings_delete(key);
	}
}

static int cmd_erase(const struct shell *sh, size_t argc, char **argv)
{
	int zero = 0;

	(void)settings_delete(KEY_LAN);
	tcam_erase(0);
	(void)settings_save_one(KEY_TCAM_N, &zero, sizeof(zero));
	net_if_foreach(qav_erase_cb, NULL);
	shell_print(sh, "saved configuration erased, defaults after a reboot");

	return 0;
}

static int cmd_mac_uid(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	(void)netshell_mac_forget();
	shell_print(sh, "saved MAC addresses dropped, the next boot derives them from the "
			"SPI NOR unique ID (new addresses: new DHCP leases and .local name)");
	return 0;
}

static int cmd_show(const struct shell *sh, size_t argc, char **argv)
{
#if defined(CONFIG_NETSHELL_ROUTER)
	struct netshell_lan l;
	char a[NET_IPV4_ADDR_LEN], m[NET_IPV4_ADDR_LEN], p[NET_IPV4_ADDR_LEN], d[NET_IPV4_ADDR_LEN];

	if (load(KEY_LAN, &l, sizeof(l))) {
		shell_print(sh, "lan %s/%s pool %s dns %s",
			    net_addr_ntop(NET_AF_INET, &l.addr, a, sizeof(a)),
			    net_addr_ntop(NET_AF_INET, &l.mask, m, sizeof(m)),
			    net_addr_ntop(NET_AF_INET, &l.pool, p, sizeof(p)),
			    net_ipv4_is_addr_unspecified(&l.dns)
				    ? "auto"
				    : net_addr_ntop(NET_AF_INET, &l.dns, d, sizeof(d)));
	} else {
		shell_print(sh, "lan: defaults");
	}
#endif
#if defined(CONFIG_PHY_MARVELL_MV88E6XXX_TCAM)
	int n = 0;

	(void)load(KEY_TCAM_N, &n, sizeof(n));
	shell_print(sh, "TCAM rules: %d", n);
#endif
	return 0;
}

#if defined(CONFIG_NETSHELL_ROUTER)
static int parse_ip(const struct shell *sh, const char *arg, struct net_in_addr *addr)
{
	if (net_addr_pton(NET_AF_INET, arg, addr) < 0) {
		shell_error(sh, "bad address %s", arg);
		return -EINVAL;
	}
	return 0;
}

static int lan_reapply(const struct shell *sh)
{
	struct net_if *br = netshell_bridge();
	int ret;

	if (br == NULL) {
		shell_error(sh, "no LAN bridge");
		return -ENODEV;
	}
	ret = netshell_lan_apply(br);
	if (ret < 0) {
		shell_error(sh, "LAN: %d (pool outside the subnet or over the address?)", ret);
	}
	return ret;
}

static int cmd_router_lan(const struct shell *sh, size_t argc, char **argv)
{
	char buf[NET_IPV4_ADDR_LEN + 4];
	char *slash;
	struct net_in_addr addr;
	int len;

	strncpy(buf, argv[1], sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';
	slash = strchr(buf, '/');
	len = slash != NULL ? atoi(slash + 1) : 24;
	if (slash != NULL) {
		*slash = '\0';
	}
	if (len < 8 || len > 30 || parse_ip(sh, buf, &addr) < 0) {
		shell_error(sh, "use <addr>/<8..30>");
		return -EINVAL;
	}

	netshell_lan.addr = addr;
	netshell_lan.mask.s_addr = net_htonl(GENMASK(31, 32 - len));
	/* Pool .100 of the new subnet unless it is still inside */
	if ((netshell_lan.pool.s_addr & netshell_lan.mask.s_addr) !=
	    (addr.s_addr & netshell_lan.mask.s_addr)) {
		netshell_lan.pool.s_addr = (addr.s_addr & netshell_lan.mask.s_addr) |
					   (net_htonl(100) & ~netshell_lan.mask.s_addr);
	}

	return lan_reapply(sh);
}

static int cmd_router_pool(const struct shell *sh, size_t argc, char **argv)
{
	if (parse_ip(sh, argv[1], &netshell_lan.pool) < 0) {
		return -EINVAL;
	}
	return lan_reapply(sh);
}

static int cmd_router_dns(const struct shell *sh, size_t argc, char **argv)
{
	if (strcmp(argv[1], "auto") == 0) {
		netshell_lan.dns.s_addr = 0;
	} else if (parse_ip(sh, argv[1], &netshell_lan.dns) < 0) {
		return -EINVAL;
	}
	net_dhcpv4_server_set_dns_address(net_ipv4_is_addr_unspecified(&netshell_lan.dns)
						  ? &netshell_wan_dns
						  : &netshell_lan.dns);
	return 0;
}

static int cmd_router_show(const struct shell *sh, size_t argc, char **argv)
{
	char a[NET_IPV4_ADDR_LEN], m[NET_IPV4_ADDR_LEN], p[NET_IPV4_ADDR_LEN], d[NET_IPV4_ADDR_LEN];

	shell_print(sh, "lan %s/%s pool %s (%d addresses) dns %s%s",
		    net_addr_ntop(NET_AF_INET, &netshell_lan.addr, a, sizeof(a)),
		    net_addr_ntop(NET_AF_INET, &netshell_lan.mask, m, sizeof(m)),
		    net_addr_ntop(NET_AF_INET, &netshell_lan.pool, p, sizeof(p)),
		    CONFIG_NET_DHCPV4_SERVER_ADDR_COUNT,
		    net_ipv4_is_addr_unspecified(&netshell_lan.dns)
			    ? net_addr_ntop(NET_AF_INET, &netshell_wan_dns, d, sizeof(d))
			    : net_addr_ntop(NET_AF_INET, &netshell_lan.dns, d, sizeof(d)),
		    net_ipv4_is_addr_unspecified(&netshell_lan.dns) ? " (from wan)" : "");
	return 0;
}

/* clang-format off */
SHELL_STATIC_SUBCMD_SET_CREATE(router_cmds,
	SHELL_CMD_ARG(lan, NULL, "<addr>/<len>: LAN address of the bridge", cmd_router_lan, 2, 0),
	SHELL_CMD_ARG(pool, NULL, "<addr>: first DHCP address", cmd_router_pool, 2, 0),
	SHELL_CMD_ARG(dns, NULL, "<addr>|auto: DNS server for the LAN", cmd_router_dns, 2, 0),
	SHELL_CMD_ARG(show, NULL, "current LAN settings", cmd_router_show, 1, 0),
	SHELL_SUBCMD_SET_END);
/* clang-format on */

SHELL_CMD_REGISTER(router, &router_cmds, "Router LAN (keep with \"config save\")", NULL);
#endif /* CONFIG_NETSHELL_ROUTER */

/* clang-format off */
SHELL_STATIC_SUBCMD_SET_CREATE(config_cmds,
	SHELL_CMD_ARG(save, NULL, "keep LAN, TCAM rules and Qav over reboots", cmd_save, 1, 0),
	SHELL_CMD_ARG(show, NULL, "what is saved", cmd_show, 1, 0),
	SHELL_CMD_ARG(erase, NULL, "back to the defaults on the next boot", cmd_erase, 1, 0),
	SHELL_CMD_ARG(mac-uid, NULL, "MAC addresses from the SPI NOR unique ID from the next boot",
		      cmd_mac_uid, 1, 0),
	SHELL_SUBCMD_SET_END);
/* clang-format on */

SHELL_CMD_REGISTER(config, &config_cmds, "Saved configuration", NULL);
