/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * "mvneta" shell: per-queue state and Qav setup through net_mgmt, i.e. the
 * same path an application or an AVB stack would use.
 *
 *   mvneta queues <iface>
 *   mvneta qav <iface> <queue> <idle slope bit/s | off>
 *   mvneta regs <iface>              MAC state, queue status, MIB counters
 *   mvneta sw <iface> [<addr> <reg>] switch registers through the PHY API
 */

#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/phy.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/ethernet_mgmt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/shell/shell.h>

static struct net_if *get_iface(const struct shell *sh, const char *arg)
{
	struct net_if *iface = net_if_get_by_index(strtol(arg, NULL, 10));

	if (iface == NULL || net_if_l2(iface) != &NET_L2_GET_NAME(ETHERNET)) {
		shell_error(sh, "no Ethernet interface %s", arg);
		return NULL;
	}
	return iface;
}

static int qav_get(struct net_if *iface, int queue, enum ethernet_qav_param_type type,
		   struct ethernet_req_params *p)
{
	memset(p, 0, sizeof(*p));
	p->qav_param.queue_id = queue;
	p->qav_param.type = type;
	return net_mgmt(NET_REQUEST_ETHERNET_GET_QAV_PARAM, iface, p, sizeof(*p));
}

static int cmd_queues(const struct shell *sh, size_t argc, char **argv)
{
	struct ethernet_req_params p = {0};
	struct net_if *iface = get_iface(sh, argv[1]);
	int n;

	if (iface == NULL) {
		return -ENOENT;
	}

	if (net_mgmt(NET_REQUEST_ETHERNET_GET_PRIORITY_QUEUES_NUM, iface, &p, sizeof(p)) < 0) {
		shell_error(sh, "priority queues not supported");
		return -ENOTSUP;
	}
	n = p.priority_queues_num;

	shell_print(sh, "queue  qav  idle slope [b/s]  oper [b/s]");
	for (int q = 0; q < n; q++) {
		struct ethernet_req_params st, is, op;

		if (qav_get(iface, q, ETHERNET_QAV_PARAM_TYPE_STATUS, &st) < 0 ||
		    qav_get(iface, q, ETHERNET_QAV_PARAM_TYPE_IDLE_SLOPE, &is) < 0 ||
		    qav_get(iface, q, ETHERNET_QAV_PARAM_TYPE_OPER_IDLE_SLOPE, &op) < 0) {
			shell_print(sh, "%5d  n/a", q);
			continue;
		}
		shell_print(sh, "%5d  %-3s  %16u  %10u", q, st.qav_param.enabled ? "on" : "off",
			    is.qav_param.idle_slope, op.qav_param.oper_idle_slope);
	}

	return 0;
}

static int cmd_qav(const struct shell *sh, size_t argc, char **argv)
{
	struct ethernet_req_params p = {0};
	struct net_if *iface = get_iface(sh, argv[1]);
	int queue = strtol(argv[2], NULL, 10);
	bool off = strcmp(argv[3], "off") == 0;
	int ret;

	if (iface == NULL) {
		return -ENOENT;
	}

	p.qav_param.queue_id = queue;
	if (!off) {
		p.qav_param.type = ETHERNET_QAV_PARAM_TYPE_IDLE_SLOPE;
		p.qav_param.idle_slope = strtoul(argv[3], NULL, 10);
		ret = net_mgmt(NET_REQUEST_ETHERNET_SET_QAV_PARAM, iface, &p, sizeof(p));
		if (ret < 0) {
			shell_error(sh, "idle slope: %d", ret);
			return ret;
		}
	}

	p.qav_param.type = ETHERNET_QAV_PARAM_TYPE_STATUS;
	p.qav_param.enabled = !off;
	ret = net_mgmt(NET_REQUEST_ETHERNET_SET_QAV_PARAM, iface, &p, sizeof(p));
	if (ret < 0) {
		shell_error(sh, "status: %d", ret);
	}

	return ret;
}

static uint32_t rd(const struct device *dev, uint32_t reg)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + reg);
}

static const struct {
	uint16_t off;
	const char *name;
} mib[] = {
	{0x3000, "good_octets_rx"}, {0x3010, "good_frames_rx"}, {0x3014, "bad_frames_rx"},
	{0x3018, "bcast_rx"},       {0x301c, "mcast_rx"},       {0x3060, "undersize_rx"},
	{0x3068, "oversize_rx"},    {0x3070, "mac_rx_error"},   {0x3074, "bad_crc"},
	{0x2484, "rx_discard"},     {0x2488, "rx_overrun"},     {0x3038, "good_octets_tx"},
	{0x3040, "good_frames_tx"}, {0x3048, "mcast_tx"},       {0x304c, "bcast_tx"},
};

static int cmd_regs(const struct shell *sh, size_t argc, char **argv)
{
	struct net_if *iface = get_iface(sh, argv[1]);
	const struct device *dev;

	if (iface == NULL) {
		return -ENOENT;
	}
	dev = net_if_get_device(iface);

	shell_print(sh, "GMAC_CTRL0 %08x CTRL2 %08x AN_CFG %08x STATUS %08x", rd(dev, 0x2c00),
		    rd(dev, 0x2c08), rd(dev, 0x2c0c), rd(dev, 0x2c10));
	shell_print(sh, "PORT_STATUS %08x RXQ_CMD %08x TXQ_CMD %08x", rd(dev, 0x2444),
		    rd(dev, 0x2680), rd(dev, 0x2448));
	shell_print(sh, "INTR cause %08x mask %08x misc %08x enable %08x", rd(dev, 0x25a0),
		    rd(dev, 0x25a4), rd(dev, 0x25b0), rd(dev, 0x25b8));
	shell_print(sh, "TXQ_CMD1 %08x REFILL_CLK %08x TX_MTU %08x TX_TOKEN %08x", rd(dev, 0x3e00),
		    rd(dev, 0x3e08), rd(dev, 0x3e0c), rd(dev, 0x3e14));
	shell_print(sh, "LPI ctrl0 %08x ctrl1 %08x ctrl2 %08x status %08x", rd(dev, 0x2cc0),
		    rd(dev, 0x2cc4), rd(dev, 0x2cc8), rd(dev, 0x2ccc));
	for (int q = 0; q < 8; q++) {
		shell_print(
			sh, "q%d: RXQ %08x TXQ %08x base %08x tok %08x/%08x refill %08x size %08x",
			q, rd(dev, 0x14e0 + 4 * q), rd(dev, 0x3c40 + 4 * q),
			rd(dev, 0x3c00 + 4 * q), rd(dev, 0x2700 + 16 * q), rd(dev, 0x2704 + 16 * q),
			rd(dev, 0x3e20 + 4 * q), rd(dev, 0x3e40 + 4 * q));
	}
	{
		/* First TX descriptors of queue 0 (identity mapped, non-cacheable) */
		volatile uint32_t *d = (volatile uint32_t *)(uintptr_t)rd(dev, 0x3c00);

		for (int i = 0; i < 3 && d != NULL; i++, d += 8) {
			shell_print(sh, "txd%d: cmd %08x size %04x buf %08x", i, d[0], d[1] >> 16,
				    d[2]);
		}
	}
	shell_print(sh, "MIB (clear on read):");
	for (size_t i = 0; i < ARRAY_SIZE(mib); i++) {
		shell_print(sh, "  %-16s %u", mib[i].name, rd(dev, mib[i].off));
	}
	return 0;
}

static int sw_rd(const struct device *phy, uint8_t addr, uint8_t reg, uint32_t *val)
{
	return phy_read(phy, (addr << 8) | reg, val);
}

static int cmd_sw(const struct shell *sh, size_t argc, char **argv)
{
	struct net_if *iface = get_iface(sh, argv[1]);
	const struct device *phy;
	uint32_t v;

	if (iface == NULL) {
		return -ENOENT;
	}
	phy = net_eth_get_phy(iface);
	if (phy == NULL) {
		shell_error(sh, "no PHY");
		return -ENODEV;
	}

	if (argc == 4) {
		uint8_t addr = strtoul(argv[2], NULL, 0);
		uint8_t reg = strtoul(argv[3], NULL, 0);
		int ret = sw_rd(phy, addr, reg, &v);

		if (ret < 0) {
			shell_error(sh, "read: %d", ret);
			return ret;
		}
		shell_print(sh, "0x%02x/0x%02x = 0x%04x", addr, reg, v);
		return 0;
	}

	/* Per-port summary: status, MAC control, control 0, base VLAN */
	shell_print(sh, "port  sts   link spd dup  macctl ctl0  vlan");
	for (uint8_t p = 0; p < 6; p++) {
		uint32_t sts, mac, ctl, vlan;
		static const char *const spd[] = {"10", "100", "1000", "x"};

		if (sw_rd(phy, 0x10 + p, 0, &sts) || sw_rd(phy, 0x10 + p, 1, &mac) ||
		    sw_rd(phy, 0x10 + p, 4, &ctl) || sw_rd(phy, 0x10 + p, 6, &vlan)) {
			shell_error(sh, "port %u: read failed", p);
			continue;
		}
		shell_print(sh, "%4u  %04x  %-4s %-4s %-4s %04x   %04x  %04x", p, sts,
			    (sts & BIT(11)) ? "up" : "down", spd[(sts >> 8) & 3],
			    (sts & BIT(10)) ? "full" : "half", mac, ctl, vlan);
	}
	return 0;
}

/* clang-format off */
SHELL_STATIC_SUBCMD_SET_CREATE(mvneta_cmds,
	SHELL_CMD_ARG(queues, NULL, "<iface>: TX queues and their Qav state", cmd_queues, 2, 0),
	SHELL_CMD_ARG(qav, NULL, "<iface> <queue> <idle slope b/s|off>", cmd_qav, 4, 0),
	SHELL_CMD_ARG(regs, NULL, "<iface>: MAC registers, queues, MIB counters", cmd_regs, 2, 0),
	SHELL_CMD_ARG(sw, NULL, "<iface> [<addr> <reg>]: switch ports or one register", cmd_sw,
		      2, 2),
	SHELL_SUBCMD_SET_END);
/* clang-format on */

SHELL_CMD_REGISTER(mvneta, &mvneta_cmds, "Marvell NETA queues, shaping and debug", NULL);
