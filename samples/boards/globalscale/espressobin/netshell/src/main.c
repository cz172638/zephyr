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
#include <zephyr/net/net_if.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/dhcpv4_server.h>
#include <zephyr/net/ethernet_bridge.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/ethernet_mgmt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/ssh/keygen.h>
#include <zephyr/settings/settings.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

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

/* Keep a random MAC address over reboots; a real (universal) one wins.
 * Interface 1 keeps the key of the single interface setup.
 */
static void stable_mac(struct net_if *iface)
{
	struct net_linkaddr *ll = net_if_get_link_addr(iface);
	struct ethernet_req_params params = {0};
	uint8_t mac[NET_ETH_ADDR_LEN];
	struct setting_read r = {.buf = mac, .len = sizeof(mac)};
	int idx = net_if_get_by_iface(iface);
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
		(void)settings_save_one(key, ll->addr, NET_ETH_ADDR_LEN);
		LOG_INF("iface %d: saved MAC %02x:%02x:%02x:%02x:%02x:%02x", idx, ll->addr[0],
			ll->addr[1], ll->addr[2], ll->addr[3], ll->addr[4], ll->addr[5]);
		return;
	}
	if (memcmp(mac, ll->addr, sizeof(mac)) == 0) {
		return;
	}

	memcpy(params.mac_address.addr, mac, sizeof(mac));
	(void)net_if_down(iface);
	if (net_mgmt(NET_REQUEST_ETHERNET_SET_MAC_ADDRESS, iface, &params, sizeof(params)) != 0) {
		LOG_WRN("could not restore the saved MAC address");
	}
	(void)net_if_up(iface);
	LOG_INF("iface %d: MAC %02x:%02x:%02x:%02x:%02x:%02x (saved)", idx, mac[0], mac[1], mac[2],
		mac[3], mac[4], mac[5]);
}

#if defined(CONFIG_NETSHELL_ROUTER)
static struct net_if *lan_bridge;

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
	if (!r.found) {
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
	net_dhcpv4_server_set_dns_address(&dns);
	LOG_INF("LAN DNS server %s (from %s)",
		net_addr_ntop(NET_AF_INET, &dns, str, sizeof(str)), name);
}

static int lan_start(struct net_if *br)
{
	struct net_in_addr addr, mask, pool;
	char name[CONFIG_NET_INTERFACE_NAME_LEN + 1];
	int ret;

	if (net_addr_pton(NET_AF_INET, CONFIG_NETSHELL_LAN_ADDR, &addr) < 0 ||
	    net_addr_pton(NET_AF_INET, CONFIG_NETSHELL_LAN_NETMASK, &mask) < 0 ||
	    net_addr_pton(NET_AF_INET, CONFIG_NETSHELL_DHCP_POOL, &pool) < 0) {
		LOG_ERR("bad LAN address configuration");
		return -EINVAL;
	}

	if (net_if_ipv4_addr_add(br, &addr, NET_ADDR_MANUAL, 0) == NULL) {
		return -ENOMEM;
	}
	net_if_ipv4_set_netmask_by_addr(br, &addr, &mask);
	/* Handed out as the router option */
	net_if_ipv4_set_gw(br, &addr);

	ret = net_if_up(br);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("bridge up: %d (no lan port?)", ret);
		return ret;
	}

	ret = net_dhcpv4_server_start(br, &pool);
	(void)net_if_get_name(br, name, sizeof(name));
	LOG_INF("%s: %s/%s, DHCP from %s: %d", name, CONFIG_NETSHELL_LAN_ADDR,
		CONFIG_NETSHELL_LAN_NETMASK, CONFIG_NETSHELL_DHCP_POOL, ret);

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

	net_eth_bridge_foreach(find_bridge, &lan_bridge);
	if (lan_bridge != NULL) {
		bridge_mac(lan_bridge);
	}
#endif

	net_if_foreach(start_iface, NULL);

#if defined(CONFIG_NETSHELL_ROUTER)
	if (lan_bridge == NULL || lan_start(lan_bridge) < 0) {
		LOG_ERR("LAN not started");
	}
#endif

	if (host_key() != 0) {
		LOG_ERR("no SSH host key");
		return 0;
	}

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
