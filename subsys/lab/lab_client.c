/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Lab agent, client side: requests to the peer's agent with retries.
 * A retry reuses the request id, so the peer answers from its cache and
 * a command never runs twice.
 */

#undef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L /* strtok_r */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/lab/lab.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/util.h>

#include "lab_internal.h"

#define MAGIC     "LAB1"
#define DGRAM_MAX 1400
#define CHUNK     1200 /* server OUT payload size */
#define PUT_CHUNK 1024
#define RETRIES   3

static K_MUTEX_DEFINE(client_lock);
static uint8_t crx[DGRAM_MAX + 1];
static uint8_t ctx_buf[DGRAM_MAX + 1];
static uint32_t next_id;
static int64_t peer_offset;

typedef int (*reply_cb_t)(void *ctx, const char *op, char *args, const uint8_t *pl, size_t plen);

static int peer_addr(const char *addr, struct sockaddr_in *sa)
{
	if (addr == NULL || *addr == '\0') {
		addr = lab.peer;
	}
	if (*addr == '\0') {
		return -ENOTCONN;
	}
	memset(sa, 0, sizeof(*sa));
	sa->sin_family = AF_INET;
	sa->sin_port = htons(CONFIG_LAB_PORT);
	return zsock_inet_pton(AF_INET, addr, &sa->sin_addr) == 1 ? 0 : -EINVAL;
}

/*
 * Send one request and feed the replies with its id to cb until cb says
 * done (1) or fails (<0); retransmit after a silent timeout slice.
 */
static int xfer(const char *addr, const char *op, const char *args, const void *pl, size_t plen,
		uint32_t timeout_ms, reply_cb_t cb, void *ctx)
{
	struct sockaddr_in sa;
	uint32_t id;
	size_t h;
	int s, ret;

	if (lab.token[0] == '\0') {
		return -EACCES;
	}
	ret = peer_addr(addr, &sa);
	if (ret != 0) {
		return ret;
	}
	if (next_id == 0) {
		next_id = sys_rand32_get() | 1;
	}
	id = next_id++;
	h = snprintf((char *)ctx_buf, sizeof(ctx_buf), MAGIC " %s %s %u%s%s\n", lab.token, op, id,
		     args ? " " : "", args ? args : "");
	if (h + plen > DGRAM_MAX) {
		return -EMSGSIZE;
	}
	if (plen > 0) {
		memcpy(&ctx_buf[h], pl, plen);
	}

	s = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (s < 0) {
		return -errno;
	}
	ret = -ETIMEDOUT;
	for (int attempt = 0; attempt < RETRIES && ret == -ETIMEDOUT; attempt++) {
		int64_t end = k_uptime_get() + timeout_ms / RETRIES + 1;

		if (zsock_sendto(s, ctx_buf, h + plen, 0, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
			ret = -errno;
			break;
		}
		for (;;) {
			struct zsock_pollfd pfd = {.fd = s, .events = ZSOCK_POLLIN};
			int64_t left = end - k_uptime_get();
			char *save, *rop, *rid, *nl, *rargs;
			const uint8_t *rpl = NULL;
			size_t rplen = 0;
			ssize_t n;

			if (left <= 0 || zsock_poll(&pfd, 1, (int)left) <= 0) {
				break;
			}
			n = zsock_recv(s, crx, DGRAM_MAX, 0);
			if (n <= 0) {
				continue;
			}
			crx[n] = '\0';
			nl = memchr(crx, '\n', n);
			if (nl != NULL) {
				*nl = '\0';
				rpl = (uint8_t *)nl + 1;
				rplen = n - (size_t)(rpl - crx);
			}
			if (strtok_r((char *)crx, " ", &save) == NULL ||
			    strcmp((char *)crx, MAGIC) != 0) {
				continue;
			}
			rop = strtok_r(NULL, " ", &save);
			rid = strtok_r(NULL, " ", &save);
			if (rop == NULL || rid == NULL || strtoul(rid, NULL, 0) != id) {
				continue; /* a late answer to an older request */
			}
			rargs = save != NULL ? save : "";
			ret = cb(ctx, rop, rargs, rpl, rplen);
			if (ret != 0) {
				break;
			}
			end = k_uptime_get() + timeout_ms / RETRIES + 1;
			ret = -ETIMEDOUT;
		}
	}
	zsock_close(s);
	return ret < 0 ? ret : 0;
}

/* ------------------------------------------------------------------------ */

struct text_ctx {
	char *out;
	size_t len;
};

static int hello_cb(void *p, const char *op, char *args, const uint8_t *pl, size_t plen)
{
	struct text_ctx *c = p;

	ARG_UNUSED(pl);
	ARG_UNUSED(plen);
	if (strcmp(op, "HELLO") != 0) {
		return 0;
	}
	snprintf(c->out, c->len, "%s", args);
	return 1;
}

int lab_hello(const char *addr, char *out, size_t len, uint32_t timeout_ms)
{
	struct text_ctx c = {out, len};
	int ret;

	k_mutex_lock(&client_lock, K_FOREVER);
	ret = xfer(addr, "HELLO", NULL, NULL, 0, timeout_ms, hello_cb, &c);
	k_mutex_unlock(&client_lock);
	return ret;
}

struct exec_ctx {
	char *out;
	size_t len;
	uint32_t got; /* bitmap of chunks seen, first 32 */
	int rc;
	size_t total;
};

static int exec_cb(void *p, const char *op, char *args, const uint8_t *pl, size_t plen)
{
	struct exec_ctx *c = p;

	if (strcmp(op, "OUT") == 0) {
		uint32_t seq = strtoul(args, NULL, 0);
		size_t off = (size_t)seq * CHUNK;

		if (c->out != NULL && off < c->len) {
			memcpy(&c->out[off], pl, MIN(plen, c->len - off));
		}
		if (seq < 32) {
			c->got |= BIT(seq);
		}
		return 0;
	}
	if (strcmp(op, "END") == 0) {
		char *save;
		char *rc = strtok_r(args, " ", &save);
		char *chunks = strtok_r(NULL, " ", &save);
		char *total = strtok_r(NULL, " ", &save);
		uint32_t n = chunks ? strtoul(chunks, NULL, 0) : 0;
		uint32_t want = n >= 32 ? UINT32_MAX : BIT_MASK(n);

		c->rc = rc ? strtol(rc, NULL, 0) : -EIO;
		c->total = total ? strtoul(total, NULL, 0) : 0;
		if ((c->got & want) != want) {
			/* lost a chunk: time out and ask again (same id) */
			c->got = 0;
			return 0;
		}
		return 1;
	}
	return 0;
}

int lab_peer_exec(const char *cmd, char *out, size_t len, k_timeout_t timeout)
{
	struct exec_ctx c = {.out = out, .len = len};
	uint32_t ms = K_TIMEOUT_EQ(timeout, K_FOREVER) ? 60000 : k_ticks_to_ms_ceil32(timeout.ticks);
	int ret;

	if (out != NULL && len > 0) {
		memset(out, 0, len);
	}
	k_mutex_lock(&client_lock, K_FOREVER);
	/* the command may run long; the first slice must cover it */
	ret = xfer(NULL, "EXEC", cmd, NULL, 0, ms * RETRIES, exec_cb, &c);
	k_mutex_unlock(&client_lock);
	if (out != NULL && len > 0) {
		out[MIN(c.total, len - 1)] = '\0';
	}
	return ret < 0 ? ret : c.rc;
}

struct time_ctx {
	uint64_t t1, t2, t3, t4;
};

static int time_cb(void *p, const char *op, char *args, const uint8_t *pl, size_t plen)
{
	struct time_ctx *c = p;
	char *save;
	char *a1, *a2, *a3;

	ARG_UNUSED(pl);
	ARG_UNUSED(plen);
	c->t4 = lab_now_ns();
	if (strcmp(op, "TIME") != 0) {
		return 0;
	}
	a1 = strtok_r(args, " ", &save);
	a2 = strtok_r(NULL, " ", &save);
	a3 = strtok_r(NULL, " ", &save);
	if (a1 == NULL || a2 == NULL || a3 == NULL || strtoull(a1, NULL, 0) != c->t1) {
		return 0;
	}
	c->t2 = strtoull(a2, NULL, 0);
	c->t3 = strtoull(a3, NULL, 0);
	return 1;
}

int lab_peer_time(int n, int64_t *offset_ns, uint64_t *rtt_ns)
{
	uint64_t best_rtt = UINT64_MAX;
	int64_t best_off = 0;
	int ok = 0, ret = 0;

	k_mutex_lock(&client_lock, K_FOREVER);
	for (int i = 0; i < MAX(n, 1); i++) {
		struct time_ctx c;
		char a[24];
		uint64_t rtt;

		c.t1 = lab_now_ns();
		snprintf(a, sizeof(a), "%llu", (unsigned long long)c.t1);
		ret = xfer(NULL, "TIME", a, NULL, 0, 300, time_cb, &c);
		if (ret != 0) {
			continue;
		}
		rtt = (c.t4 - c.t1) - (c.t3 - c.t2);
		if (rtt < best_rtt) {
			best_rtt = rtt;
			best_off = ((int64_t)(c.t2 - c.t1) + (int64_t)(c.t3 - c.t4)) / 2;
		}
		ok++;
	}
	k_mutex_unlock(&client_lock);
	if (ok == 0) {
		return ret != 0 ? ret : -ETIMEDOUT;
	}
	peer_offset = best_off;
	if (offset_ns != NULL) {
		*offset_ns = best_off;
	}
	if (rtt_ns != NULL) {
		*rtt_ns = best_rtt;
	}
	return ok;
}

int64_t lab_peer_offset_ns(void)
{
	return peer_offset;
}

static int ack_cb(void *p, const char *op, char *args, const uint8_t *pl, size_t plen)
{
	int *res = p;
	char *save;

	ARG_UNUSED(pl);
	ARG_UNUSED(plen);
	if (strcmp(op, "ACK") != 0) {
		return 0;
	}
	(void)strtok_r(args, " ", &save); /* offset */
	*res = save != NULL ? strtol(save, NULL, 0) : -EIO;
	return 1;
}

int lab_peer_put(const char *name, const void *data, size_t len)
{
	const uint8_t *d = data;
	int ret = 0;

	k_mutex_lock(&client_lock, K_FOREVER);
	for (size_t off = 0; off < len || (len == 0 && off == 0); off += PUT_CHUNK) {
		size_t n = MIN(PUT_CHUNK, len - off);
		char a[64];
		int res = -EIO;

		snprintf(a, sizeof(a), "%s %zu %zu", name, off, len);
		ret = xfer(NULL, "PUT", a, &d[off], n, 1500, ack_cb, &res);
		if (ret == 0) {
			ret = res;
		}
		if (ret != 0 || len == 0) {
			break;
		}
	}
	k_mutex_unlock(&client_lock);
	return ret;
}
