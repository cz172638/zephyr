/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * CMSIS-DAP over TCP, as OpenOCD's cmsis-dap tcp backend frames it: each
 * request and response is an 8-byte little-endian header (signature "DAP",
 * payload length, packet type, reserved) followed by the payload. One
 * client at a time.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/dap/dap_link.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/byteorder.h>

#include "cmsis_dap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dap_tcp, CONFIG_DAP_LOG_LEVEL);

#define HDR_SIZE      8U
#define HDR_SIGNATURE 0x00504144U
#define PKT_REQUEST   0x01U
#define PKT_RESPONSE  0x02U

#define PKT_SIZE CONFIG_DAP_BACKEND_TCP_PACKET_SIZE

static uint8_t req_buf[HDR_SIZE + PKT_SIZE];
static uint8_t rsp_buf[HDR_SIZE + PKT_SIZE];

static struct dap_link_context *tcp_ctx;

static K_THREAD_STACK_DEFINE(dap_tcp_stack, CONFIG_DAP_BACKEND_TCP_STACK_SIZE);
static struct k_thread dap_tcp_thread;

static int recv_all(int fd, uint8_t *buf, size_t len)
{
	while (len > 0U) {
		ssize_t n = zsock_recv(fd, buf, len, 0);

		if (n <= 0) {
			return n == 0 ? -ECONNRESET : -errno;
		}
		buf += n;
		len -= (size_t)n;
	}
	return 0;
}

static int send_all(int fd, const uint8_t *buf, size_t len)
{
	while (len > 0U) {
		ssize_t n = zsock_send(fd, buf, len, 0);

		if (n < 0) {
			return -errno;
		}
		buf += n;
		len -= (size_t)n;
	}
	return 0;
}

static void serve(int fd)
{
	uint32_t len, rsp_len;

	while (true) {
		if (recv_all(fd, req_buf, HDR_SIZE) != 0) {
			return;
		}
		len = sys_get_le16(&req_buf[4]);
		if (sys_get_le32(&req_buf[0]) != HDR_SIGNATURE || req_buf[6] != PKT_REQUEST ||
		    len == 0U || len > PKT_SIZE) {
			LOG_ERR("bad header");
			return;
		}
		if (recv_all(fd, &req_buf[HDR_SIZE], len) != 0) {
			return;
		}
		/* commands longer than the request read zeros, not stale data */
		memset(&req_buf[HDR_SIZE + len], 0, PKT_SIZE - len);

		rsp_len = dap_link_execute_cmd(tcp_ctx, &req_buf[HDR_SIZE], &rsp_buf[HDR_SIZE]);
		rsp_len = MIN(rsp_len, PKT_SIZE);
		sys_put_le32(HDR_SIGNATURE, &rsp_buf[0]);
		sys_put_le16((uint16_t)rsp_len, &rsp_buf[4]);
		rsp_buf[6] = PKT_RESPONSE;
		rsp_buf[7] = 0U;
		if (send_all(fd, rsp_buf, HDR_SIZE + rsp_len) != 0) {
			return;
		}
	}
}

static void dap_tcp_main(void *p1, void *p2, void *p3)
{
	struct net_sockaddr_in addr = {
		.sin_family = NET_AF_INET,
		.sin_port = net_htons(CONFIG_DAP_BACKEND_TCP_PORT),
	};
	int one = 1;
	int srv, fd;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	srv = zsock_socket(NET_AF_INET, NET_SOCK_STREAM, NET_IPPROTO_TCP);
	if (srv < 0) {
		LOG_ERR("socket: %d", errno);
		return;
	}
	(void)zsock_setsockopt(srv, ZSOCK_SOL_SOCKET, ZSOCK_SO_REUSEADDR, &one, sizeof(one));
	if (zsock_bind(srv, (struct net_sockaddr *)&addr, sizeof(addr)) < 0 ||
	    zsock_listen(srv, 1) < 0) {
		LOG_ERR("bind/listen: %d", errno);
		zsock_close(srv);
		return;
	}
	LOG_INF("CMSIS-DAP on TCP port %d", CONFIG_DAP_BACKEND_TCP_PORT);

	while (true) {
		fd = zsock_accept(srv, NULL, NULL);
		if (fd < 0) {
			k_msleep(100);
			continue;
		}
		/* one small request, one small response: no Nagle delay */
		(void)zsock_setsockopt(fd, NET_IPPROTO_TCP, ZSOCK_TCP_NODELAY, &one, sizeof(one));
		LOG_INF("client connected");
		serve(fd);
		zsock_close(fd);
		LOG_INF("client disconnected");
	}
}

int dap_link_backend_tcp_init(struct dap_link_context *const dap_link_ctx)
{
	if (tcp_ctx != NULL) {
		return -EALREADY;
	}
	tcp_ctx = dap_link_ctx;
	dap_link_set_pkt_size(dap_link_ctx, PKT_SIZE);
	k_thread_create(&dap_tcp_thread, dap_tcp_stack, K_THREAD_STACK_SIZEOF(dap_tcp_stack),
			dap_tcp_main, NULL, NULL, NULL, CONFIG_DAP_BACKEND_TCP_PRIORITY, 0,
			K_NO_WAIT);
	k_thread_name_set(&dap_tcp_thread, "dap_tcp");
	return 0;
}
