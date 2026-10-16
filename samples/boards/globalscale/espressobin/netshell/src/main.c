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
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/ethernet_mgmt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/ssh/keygen.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(netshell, LOG_LEVEL_INF);

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

/* Keep a random MAC address over reboots; a real (universal) one wins */
static void stable_mac(struct net_if *iface)
{
	struct net_linkaddr *ll = net_if_get_link_addr(iface);
	struct ethernet_req_params params = {0};
	uint8_t mac[NET_ETH_ADDR_LEN];
	struct setting_read r = {.buf = mac, .len = sizeof(mac)};

	if (ll->len != NET_ETH_ADDR_LEN || (ll->addr[0] & 0x02) == 0) {
		return;
	}

	(void)settings_load_subtree_direct("netshell/mac", setting_read_cb, &r);
	if (!r.found) {
		(void)settings_save_one("netshell/mac", ll->addr, NET_ETH_ADDR_LEN);
		LOG_INF("saved MAC %02x:%02x:%02x:%02x:%02x:%02x", ll->addr[0], ll->addr[1],
			ll->addr[2], ll->addr[3], ll->addr[4], ll->addr[5]);
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
	LOG_INF("MAC %02x:%02x:%02x:%02x:%02x:%02x (saved)", mac[0], mac[1], mac[2], mac[3], mac[4],
		mac[5]);
}

static int run(const char *cmd)
{
	int ret = shell_execute_cmd(shell_backend_uart_get_ptr(), cmd);

	if (ret != 0) {
		LOG_WRN("\"%s\": %d", cmd, ret);
	}
	return ret;
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
	return run("net ssh_key save 0 priv id_rsa");
}

int main(void)
{
	struct net_if *iface = net_if_get_default();
	char cmd[128];
	int n;

	/* Let the shell instance come up before using it */
	while (!shell_ready(shell_backend_uart_get_ptr())) {
		k_msleep(10);
	}

	if (settings_subsys_init() != 0) {
		LOG_ERR("settings unavailable, host key will not persist");
	}

	if (net_if_l2(iface) == &NET_L2_GET_NAME(ETHERNET)) {
		stable_mac(iface);
	}
	net_dhcpv4_start(iface);

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
