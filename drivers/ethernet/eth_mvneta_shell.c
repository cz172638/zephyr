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
 */

#include <stdlib.h>
#include <zephyr/net/net_if.h>
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

SHELL_STATIC_SUBCMD_SET_CREATE(mvneta_cmds,
			       SHELL_CMD_ARG(queues, NULL, "<iface>: TX queues and their Qav state",
					     cmd_queues, 2, 0),
			       SHELL_CMD_ARG(qav, NULL, "<iface> <queue> <idle slope b/s|off>",
					     cmd_qav, 4, 0),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(mvneta, &mvneta_cmds, "Marvell NETA queues and shaping", NULL);
