/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef NETSHELL_CONFIG_H_
#define NETSHELL_CONFIG_H_

#include <zephyr/net/net_if.h>

/* LAN of the router; dns unspecified: the DNS server of the wan lease */
struct netshell_lan {
	struct net_in_addr addr;
	struct net_in_addr mask;
	struct net_in_addr pool;
	struct net_in_addr dns;
};

extern struct netshell_lan netshell_lan;

/* DNS server of the last wan lease */
extern struct net_in_addr netshell_wan_dns;

/* Saved configuration ("config save"): LAN, TCAM rules and Qav. The LAN
 * part is read before the LAN starts, the rest applied once the
 * interfaces exist.
 */
void netshell_config_load_lan(void);
void netshell_config_apply(void);

/* Bring the bridge to netshell_lan (address, DHCP server, DNS) */
int netshell_lan_apply(struct net_if *br);

/* The LAN bridge, NULL without one */
struct net_if *netshell_bridge(void);

#endif /* NETSHELL_CONFIG_H_ */
