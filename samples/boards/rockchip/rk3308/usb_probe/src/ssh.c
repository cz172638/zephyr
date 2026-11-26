/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * SSH server with the shell backend, on all addresses: the host key built
 * into the image (USB_PROBE_SSH_HOST_KEY) or a new one per boot.
 */

#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/ssh/keygen.h>
#include <zephyr/net/ssh/server.h>
#include <zephyr/shell/shell_ssh.h>
#include <zephyr/sys/printk.h>

#include "probe.h"

#define HOST_KEY 0

#if defined(USB_PROBE_SSH_HOST_KEY_INC)
static const uint8_t host_key_der[] = {
#include "ssh_host_key.der.inc"
};
#endif

static int host_key_setup(void)
{
	int ret;

#if defined(USB_PROBE_SSH_HOST_KEY_INC)
	ret = ssh_keygen_import(HOST_KEY, true, SSH_HOST_KEY_FORMAT_DER, host_key_der,
				sizeof(host_key_der));
	if (ret != 0) {
		printk("ssh: host key import: %d\n", ret);
	}
#else
	int64_t t0;

	printk("ssh: generating a %d bit RSA host key\n", CONFIG_USB_PROBE_SSH_HOST_KEY_BITS);
	t0 = k_uptime_get();
	ret = ssh_keygen(HOST_KEY, SSH_HOST_KEY_TYPE_RSA, CONFIG_USB_PROBE_SSH_HOST_KEY_BITS);
	if (ret != 0) {
		printk("ssh: host key generation: %d\n", ret);
	} else {
		printk("ssh: host key ready after %lld ms\n", k_uptime_get() - t0);
	}
#endif
	return ret;
}

void probe_ssh_start(void)
{
	struct net_sockaddr_in addr = {
		.sin_family = NET_AF_INET,
		.sin_port = net_htons(CONFIG_SSH_PORT),
	};
	const char *user = CONFIG_USB_PROBE_SSH_USERNAME;
	int ret;

	if (CONFIG_USB_PROBE_SSH_PASSWORD[0] == '\0') {
		printk("ssh: no password configured, server not started\n");
		return;
	}
	if (host_key_setup() != 0) {
		return;
	}
	ssh_server_set_banner_text("RK3308 lab probe (" CONFIG_BOARD_TARGET ")");
	ret = ssh_server_start(ssh_server_instance(0), (struct net_sockaddr *)&addr, HOST_KEY,
			       user[0] != '\0' ? user : NULL, CONFIG_USB_PROBE_SSH_PASSWORD, NULL,
			       0, shell_sshd_event_callback, shell_sshd_transport_event_callback,
			       NULL);
	if (ret != 0) {
		printk("ssh: server start: %d\n", ret);
		return;
	}
	printk("ssh: listening on port %d\n", CONFIG_SSH_PORT);
}
