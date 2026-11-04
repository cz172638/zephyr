/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Headless ESPRESSObin: get an address with DHCP and start an SSH server
 * with the Zephyr shell, so the board can be used and updated
 * ("fwupdate tcp") without a serial console.
 *
 * The RSA host key is generated on the first boot and kept in settings
 * (NVS in the SPI NOR), so the fingerprint stays the same across reboots
 * and firmware updates. A random (locally administered) MAC address is
 * kept the same way, so DHCP leases and the mDNS name stay stable.
 *
 * With DSA (espressobin-dsa snippet) every switch port is its own
 * interface: each one gets a stable MAC address and DHCP.
 *
 * As a router (router.conf, ports named by espressobin-dsa-v5/-v7) the
 * wan port gets DHCP, the lan ports form a bridge, switched in hardware,
 * with a static address and a DHCP server, and IPv4 is forwarded between
 * the two (no NAT: the upstream router needs a route to the LAN).
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/flash/spi_nor_ex.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/dhcpv4_server.h>
#include <zephyr/net/ethernet_bridge.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/ethernet_mgmt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/ssh/keygen.h>
#include <zephyr/net/ssh/server.h>
#include <zephyr/settings/settings.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

#include "config.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(netshell, LOG_LEVEL_INF);

#if DT_HAS_CHOSEN(zephyr_settings_partition)
#define SETTINGS_AREA DT_PARTITION_ID(DT_CHOSEN(zephyr_settings_partition))
#else
#define SETTINGS_AREA PARTITION_ID(storage_partition)
#endif

#define HOST_KEY_IDX 0
#define AUTH_KEY_IDX 1

#if defined(NETSHELL_HAVE_AUTHORIZED_KEY)
static const char authorized_key[] = {
#include "authorized_key.inc"
	0,
};
#endif

struct setting_read {
	void *buf;
	size_t len;
	bool found;
};

static int setting_read_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
			   void *param)
{
	struct setting_read *r = param;

	ARG_UNUSED(key);

	if (len == r->len && read_cb(cb_arg, r->buf, len) == len) {
		r->found = true;
	}
	return 0;
}

/*
 * MAC address from the SPI NOR factory unique ID (command 0x4B): the same
 * on every boot and after a settings erase, different per interface (n).
 * FNV-1a 64 over the ID and n; locally administered, unicast.
 */
static bool uid_mac(int n, uint8_t mac[NET_ETH_ADDR_LEN])
{
#if defined(CONFIG_NETSHELL_MAC_SPI_NOR_UID)
	static uint8_t uid[SPI_NOR_UID_LEN];
	static int uid_state; /* 0 not read, 1 valid, -1 none */
	uint64_t h = 0xcbf29ce484222325ULL;

	if (uid_state == 0) {
		const struct device *nor = DEVICE_DT_GET_ANY(jedec_spi_nor);
		bool blank = true;

		uid_state = -1;
		if (nor != NULL && device_is_ready(nor) &&
		    flash_ex_op(nor, FLASH_EX_OP_SPI_NOR_READ_UID, 0, uid) == 0) {
			for (int i = 0; i < SPI_NOR_UID_LEN; i++) {
				blank &= uid[i] == 0x00 || uid[i] == 0xff;
			}
			uid_state = blank ? -1 : 1;
		}
		if (uid_state < 0) {
			LOG_WRN("no SPI NOR unique ID, random MAC addresses");
		}
	}
	if (uid_state < 0) {
		return false;
	}

	for (int i = 0; i < SPI_NOR_UID_LEN + 1; i++) {
		h ^= i < SPI_NOR_UID_LEN ? uid[i] : (uint8_t)n;
		h *= 0x100000001b3ULL;
	}
	for (int i = 0; i < NET_ETH_ADDR_LEN; i++) {
		mac[i] = h >> (8 * i);
	}
	mac[0] = (mac[0] & ~0x01) | 0x02;
	return true;
#else
	ARG_UNUSED(n);
	ARG_UNUSED(mac);
	return false;
#endif
}

/*
 * Stable MAC address: one saved in settings (older images, or set by
 * hand) wins, else the one from the SPI NOR unique ID, else the random
 * one is saved and kept. A real (universal) address is left alone.
 * Interface 1 keeps the key of the single interface setup.
 */
static void stable_mac(struct net_if *iface)
{
	struct net_linkaddr *ll = net_if_get_link_addr(iface);
	struct ethernet_req_params params = {0};
	uint8_t mac[NET_ETH_ADDR_LEN];
	struct setting_read r = {.buf = mac, .len = sizeof(mac)};
	int idx = net_if_get_by_iface(iface);
	const char *how = "saved";
	char key[24];

	if (ll->len != NET_ETH_ADDR_LEN || (ll->addr[0] & 0x02) == 0) {
		return;
	}

	if (idx == 1) {
		snprintf(key, sizeof(key), "netshell/mac");
	} else {
		snprintf(key, sizeof(key), "netshell/mac%d", idx);
	}

	(void)settings_load_subtree_direct(key, setting_read_cb, &r);
	if (!r.found) {
		if (!uid_mac(idx, mac)) {
			(void)settings_save_one(key, ll->addr, NET_ETH_ADDR_LEN);
			LOG_INF("iface %d: saved MAC %02x:%02x:%02x:%02x:%02x:%02x", idx,
				ll->addr[0], ll->addr[1], ll->addr[2], ll->addr[3], ll->addr[4],
				ll->addr[5]);
			return;
		}
		how = "SPI NOR UID";
	}
	if (memcmp(mac, ll->addr, sizeof(mac)) == 0) {
		return;
	}

	memcpy(params.mac_address.addr, mac, sizeof(mac));
	(void)net_if_down(iface);
	if (net_mgmt(NET_REQUEST_ETHERNET_SET_MAC_ADDRESS, iface, &params, sizeof(params)) != 0) {
		LOG_WRN("could not set the MAC address");
	}
	(void)net_if_up(iface);
	LOG_INF("iface %d: MAC %02x:%02x:%02x:%02x:%02x:%02x (%s)", idx, mac[0], mac[1], mac[2],
		mac[3], mac[4], mac[5], how);
}

/* Drop the saved MAC addresses: the next boot takes them from the UID */
int netshell_mac_forget(void)
{
	char key[24];

	(void)settings_delete("netshell/mac");
	(void)settings_delete("netshell/brmac");
	for (int i = 2; i <= 16; i++) {
		snprintf(key, sizeof(key), "netshell/mac%d", i);
		(void)settings_delete(key);
	}
	return 0;
}

#if defined(CONFIG_NETSHELL_ROUTER)
static struct net_if *lan_bridge;

struct net_if *netshell_bridge(void)
{
	return lan_bridge;
}

static void find_bridge(struct eth_bridge_iface_context *br, void *user_data)
{
	struct net_if **found = user_data;

	if (*found == NULL) {
		*found = br->iface;
	}
}

/* Bridge MAC address, kept like the port ones; set while the bridge is
 * down and before the ports join (the switch gets it as a static entry)
 */
static void bridge_mac(struct net_if *br)
{
	struct net_linkaddr *ll = net_if_get_link_addr(br);
	uint8_t mac[NET_ETH_ADDR_LEN];
	struct setting_read r = {.buf = mac, .len = sizeof(mac)};

	(void)settings_load_subtree_direct("netshell/brmac", setting_read_cb, &r);
	if (!r.found && !uid_mac(0, mac)) {
		(void)settings_save_one("netshell/brmac", ll->addr, NET_ETH_ADDR_LEN);
		return;
	}
	(void)net_if_set_link_addr(br, mac, sizeof(mac), NET_LINK_ETHERNET);
}

/* Hand out the DNS server the wan lease names (there is no forwarder) */
static uint8_t wan_dns[4];
static struct net_dhcpv4_option_callback wan_dns_cb;

static void wan_dns_handler(struct net_dhcpv4_option_callback *cb, size_t length,
			    enum net_dhcpv4_msg_type msg_type, struct net_if *iface)
{
	char name[CONFIG_NET_INTERFACE_NAME_LEN + 1];
	struct net_in_addr dns;
	char str[NET_IPV4_ADDR_LEN];

	if (length < sizeof(wan_dns) || msg_type != NET_DHCPV4_MSG_TYPE_ACK ||
	    net_if_get_name(iface, name, sizeof(name)) < 0 ||
	    strcmp(name, CONFIG_NETSHELL_WAN_IFACE) != 0) {
		return;
	}

	memcpy(dns.s4_addr, wan_dns, sizeof(wan_dns));
	netshell_wan_dns = dns;
	if (!net_ipv4_is_addr_unspecified(&netshell_lan.dns)) {
		/* A fixed one is configured */
		return;
	}
	net_dhcpv4_server_set_dns_address(&dns);
	LOG_INF("LAN DNS server %s (from %s)",
		net_addr_ntop(NET_AF_INET, &dns, str, sizeof(str)), name);
}

/* The DHCP client only sets the interface gateway, forwarding needs a
 * default router: the router option of the wan lease
 */
static uint8_t wan_gw[4];
static struct net_dhcpv4_option_callback wan_gw_cb;
static struct net_if_router *wan_router;

static void wan_gw_handler(struct net_dhcpv4_option_callback *cb, size_t length,
			   enum net_dhcpv4_msg_type msg_type, struct net_if *iface)
{
	char name[CONFIG_NET_INTERFACE_NAME_LEN + 1];
	struct net_in_addr gw;
	char str[NET_IPV4_ADDR_LEN];

	if (length < sizeof(wan_gw) || msg_type != NET_DHCPV4_MSG_TYPE_ACK ||
	    net_if_get_name(iface, name, sizeof(name)) < 0 ||
	    strcmp(name, CONFIG_NETSHELL_WAN_IFACE) != 0) {
		return;
	}

	memcpy(gw.s4_addr, wan_gw, sizeof(wan_gw));
	if (wan_router != NULL) {
		if (net_ipv4_addr_cmp(net_if_router_ipv4(wan_router), &gw)) {
			return;
		}
		(void)net_if_ipv4_router_rm(wan_router);
	}
	wan_router = net_if_ipv4_router_add(iface, &gw, true, 0);
	LOG_INF("default router %s on %s%s", net_addr_ntop(NET_AF_INET, &gw, str, sizeof(str)),
		name, wan_router == NULL ? " failed" : "");
}

struct net_in_addr netshell_wan_dns;

/* (Re)configure the bridge from netshell_lan: one address, the DHCP
 * server handing out this address as router and the DNS server
 */
int netshell_lan_apply(struct net_if *br)
{
	struct net_if_ipv4 *ipv4;
	char name[CONFIG_NET_INTERFACE_NAME_LEN + 1];
	char a[NET_IPV4_ADDR_LEN], m[NET_IPV4_ADDR_LEN], p[NET_IPV4_ADDR_LEN];
	int ret;

	(void)net_dhcpv4_server_stop(br);

	if (net_if_config_ipv4_get(br, &ipv4) == 0) {
		ARRAY_FOR_EACH(ipv4->unicast, i) {
			if (ipv4->unicast[i].ipv4.is_used &&
			    !net_ipv4_addr_cmp(&ipv4->unicast[i].ipv4.address.in_addr,
					       &netshell_lan.addr)) {
				(void)net_if_ipv4_addr_rm(
					br, &ipv4->unicast[i].ipv4.address.in_addr);
			}
		}
	}

	if (net_if_ipv4_addr_lookup_by_iface(br, &netshell_lan.addr) == NULL &&
	    net_if_ipv4_addr_add(br, &netshell_lan.addr, NET_ADDR_MANUAL, 0) == NULL) {
		return -ENOMEM;
	}
	net_if_ipv4_set_netmask_by_addr(br, &netshell_lan.addr, &netshell_lan.mask);
	/* Handed out as the router option */
	net_if_ipv4_set_gw(br, &netshell_lan.addr);

	net_dhcpv4_server_set_dns_address(net_ipv4_is_addr_unspecified(&netshell_lan.dns)
						  ? &netshell_wan_dns
						  : &netshell_lan.dns);

	ret = net_if_up(br);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("bridge up: %d (no lan port?)", ret);
		return ret;
	}

	ret = net_dhcpv4_server_start(br, &netshell_lan.pool);
	(void)net_if_get_name(br, name, sizeof(name));
	LOG_INF("%s: %s/%s, DHCP from %s: %d", name,
		net_addr_ntop(NET_AF_INET, &netshell_lan.addr, a, sizeof(a)),
		net_addr_ntop(NET_AF_INET, &netshell_lan.mask, m, sizeof(m)),
		net_addr_ntop(NET_AF_INET, &netshell_lan.pool, p, sizeof(p)), ret);

	return ret;
}
#endif /* CONFIG_NETSHELL_ROUTER */

/* Interfaces that carry IP: all but a DSA conduit and the DSA CPU port.
 * The conduit (eth0, the default interface) still keeps its MAC address:
 * the unique hostname is derived from it. As a router the lan ports join
 * the bridge instead.
 */
static void start_iface(struct net_if *iface, void *user_data)
{
	ARG_UNUSED(user_data);

	if (net_if_l2(iface) != &NET_L2_GET_NAME(ETHERNET)) {
		return;
	}
#if defined(CONFIG_NET_DSA)
	enum ethernet_hw_caps caps = net_eth_get_hw_capabilities(iface);

	if (caps & ETHERNET_DSA_CONDUIT_PORT) {
		stable_mac(iface);
		return;
	}
	if ((caps & ETHERNET_DSA_USER_PORT) == 0) {
		return;
	}
#endif
	stable_mac(iface);

#if defined(CONFIG_NETSHELL_ROUTER)
	char name[CONFIG_NET_INTERFACE_NAME_LEN + 1];

	if (net_if_get_name(iface, name, sizeof(name)) > 0 && lan_bridge != NULL &&
	    strncmp(name, CONFIG_NETSHELL_LAN_PREFIX, strlen(CONFIG_NETSHELL_LAN_PREFIX)) == 0) {
		int ret = eth_bridge_iface_add(lan_bridge, iface);

		LOG_INF("%s: bridged (%d)", name, ret);
		return;
	}
	if (strcmp(name, CONFIG_NETSHELL_WAN_IFACE) != 0) {
		LOG_WRN("%s: neither wan nor lan, using DHCP", name);
	}
#endif
	net_dhcpv4_start(iface);
}

static int run(const char *cmd)
{
	int ret = shell_execute_cmd(shell_backend_uart_get_ptr(), cmd);

	if (ret != 0) {
		LOG_WRN("\"%s\": %d", cmd, ret);
	}
	return ret;
}

/*
 * The settings partition is the sector where the stock U-Boot kept its
 * environment: anything but erased flash or NVS makes the mount fail.
 * Erase it once and try again.
 */
static int settings_init(void)
{
	const struct flash_area *fa;
	int ret = settings_subsys_init();

	if (ret == 0) {
		return 0;
	}
	if (ret == -EDOM || ret == -EINVAL || ret == -ENOENT) {
		/* configuration problem, erasing would not help */
		LOG_ERR("settings init failed (%d)", ret);
		return ret;
	}
	LOG_WRN("settings init failed (%d), erasing the settings partition", ret);

	ret = flash_area_open(SETTINGS_AREA, &fa);
	if (ret == 0) {
		ret = flash_area_erase(fa, 0, fa->fa_size);
		flash_area_close(fa);
	}
	if (ret != 0) {
		LOG_ERR("erase failed: %d", ret);
		return ret;
	}
	return settings_subsys_init();
}

static int host_key(void)
{
	char cmd[64];

	if (run("net ssh_key load 0 priv id_rsa") == 0) {
		LOG_INF("SSH host key loaded");
		return 0;
	}

	LOG_INF("generating a %d bit RSA host key, this takes a while",
		CONFIG_NETSHELL_HOST_KEY_BITS);
	snprintf(cmd, sizeof(cmd), "net ssh_key gen %d rsa %d", HOST_KEY_IDX,
		 CONFIG_NETSHELL_HOST_KEY_BITS);
	if (run(cmd) != 0) {
		return -EIO;
	}
	if (run("net ssh_key save 0 priv id_rsa") != 0) {
		/* Still usable for this boot, just a new fingerprint next time */
		LOG_WRN("host key not saved");
	}
	return 0;
}

/*
 * SSH banner: board and serial number (the SPI NOR factory unique ID, the
 * one MAC addresses and the PCIe DSN are derived from), then the MAC of the
 * default interface.
 */
static char banner[160];

static void banner_set(void)
{
	int n = snprintf(banner, sizeof(banner), "Board %s", CONFIG_BOARD_TARGET);
	struct net_if *iface = net_if_get_default();

#if DT_HAS_COMPAT_STATUS_OKAY(jedec_spi_nor) && defined(CONFIG_FLASH_EX_OP_ENABLED)
	const struct device *nor = DEVICE_DT_GET_ANY(jedec_spi_nor);
	uint8_t uid[SPI_NOR_UID_LEN];

	if (device_is_ready(nor) && flash_ex_op(nor, FLASH_EX_OP_SPI_NOR_READ_UID, 0, uid) == 0) {
		n += snprintf(banner + n, sizeof(banner) - n, ", serial ");
		for (int i = 0; i < SPI_NOR_UID_LEN && n < (int)sizeof(banner) - 3; i++) {
			n += snprintf(banner + n, sizeof(banner) - n, "%02x", uid[i]);
		}
	}
#endif
	if (iface != NULL && net_if_get_link_addr(iface)->len == NET_ETH_ADDR_LEN &&
	    n < (int)sizeof(banner)) {
		const uint8_t *m = net_if_get_link_addr(iface)->addr;

		(void)snprintf(banner + n, sizeof(banner) - n,
			       ", MAC %02x:%02x:%02x:%02x:%02x:%02x", m[0], m[1], m[2], m[3], m[4],
			       m[5]);
	}
	ssh_server_set_banner_text(banner);
	LOG_INF("%s", banner);
}

int main(void)
{
	char cmd[128];
	int n;

	/* Let the shell instance come up before using it */
	while (!shell_ready(shell_backend_uart_get_ptr())) {
		k_msleep(10);
	}

	if (settings_init() != 0) {
		LOG_ERR("settings unavailable, host key and MAC will not persist");
	}

#if defined(CONFIG_NETSHELL_ROUTER)
	net_dhcpv4_init_option_callback(&wan_dns_cb, wan_dns_handler, 6 /* DNS server */, wan_dns,
					sizeof(wan_dns));
	(void)net_dhcpv4_add_option_callback(&wan_dns_cb);
	net_dhcpv4_init_option_callback(&wan_gw_cb, wan_gw_handler, 3 /* router */, wan_gw,
					sizeof(wan_gw));
	(void)net_dhcpv4_add_option_callback(&wan_gw_cb);

	net_eth_bridge_foreach(find_bridge, &lan_bridge);
	if (lan_bridge != NULL) {
		bridge_mac(lan_bridge);
	}
#endif

	net_if_foreach(start_iface, NULL);

#if defined(CONFIG_NETSHELL_ROUTER)
	netshell_config_load_lan();
	if (lan_bridge == NULL || netshell_lan_apply(lan_bridge) < 0) {
		LOG_ERR("LAN not started");
	}
#endif
	netshell_config_apply();

	if (host_key() != 0) {
		LOG_ERR("no SSH host key");
		return 0;
	}

	banner_set();
	n = snprintf(cmd, sizeof(cmd), "net sshd start -i 0 -k %d", HOST_KEY_IDX);

#if defined(NETSHELL_HAVE_AUTHORIZED_KEY)
	if (ssh_keygen_import(AUTH_KEY_IDX, false, SSH_HOST_KEY_FORMAT_PEM, authorized_key,
			      sizeof(authorized_key)) == 0) {
		n += snprintf(cmd + n, sizeof(cmd) - n, " -a %d", AUTH_KEY_IDX);
	} else {
		LOG_ERR("authorized key not accepted (PEM/PKCS8 expected)");
	}
#endif
	if (CONFIG_NETSHELL_USERNAME[0] != '\0') {
		n += snprintf(cmd + n, sizeof(cmd) - n, " -u %s", CONFIG_NETSHELL_USERNAME);
	}
	if (CONFIG_NETSHELL_PASSWORD[0] != '\0') {
		n += snprintf(cmd + n, sizeof(cmd) - n, " -p %s", CONFIG_NETSHELL_PASSWORD);
	}

	if (run(cmd) == 0) {
		LOG_INF("SSH server listening on port %d", CONFIG_SSH_PORT);
	}

	return 0;
}
