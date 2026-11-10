/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Lab agent: a UDP service that runs shell commands for the peer board
 * (on the dummy shell backend, shared with ssh exec under one lock),
 * answers time probes and takes files; and the client side that sends
 * those requests. See zephyr/lab/lab.h for the protocol.
 *
 * Configuration in settings: lab/token (requests without it are
 * dropped; no token = the server stays off), lab/peer (IPv4 address of
 * the other board; requests from elsewhere are dropped).
 */

#undef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L /* strtok_r */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/lab/lab.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_dummy.h>
#include <zephyr/sys/util.h>
#include <zephyr/version.h>
#ifdef CONFIG_PCIE_AARDVARK_MODE
#include <zephyr/drivers/pcie/aardvark.h>
#endif

#include "lab_internal.h"

LOG_MODULE_REGISTER(lab, CONFIG_LAB_LOG_LEVEL);

#define MAGIC     "LAB1"
#define DGRAM_MAX 1400
#define CHUNK     1200

#ifdef BUILD_VERSION
#define FW_STRING STRINGIFY(BUILD_VERSION)
#else
#define FW_STRING KERNEL_VERSION_STRING
#endif

struct lab_state lab;

static char out_buf[CONFIG_SHELL_BACKEND_DUMMY_BUF_SIZE];
static uint8_t rx[DGRAM_MAX + 1];
static uint8_t tx[DGRAM_MAX + 1];

/* last EXEC: repeated requests get the cached reply */
static struct {
	uint32_t id;
	int rc;
	size_t len;
	bool valid;
} last_exec;

/* RAM slots for PUT */
static uint8_t file_pool[CONFIG_LAB_FILE_POOL_SIZE] __aligned(64);
static struct lab_file files[CONFIG_LAB_FILES];

uint64_t lab_now_ns(void)
{
	return k_cyc_to_ns_floor64(k_cycle_get_64());
}

const char *lab_role(void)
{
#ifdef CONFIG_PCIE_AARDVARK_MODE
	return advk_mode_get() == ADVK_MODE_RC ? "rc" : "ep";
#else
	return "node";
#endif
}

void lab_identity(char *buf, size_t len)
{
	struct net_if *iface = net_if_get_default();
	struct net_linkaddr *ll = iface != NULL ? net_if_get_link_addr(iface) : NULL;

	if (ll == NULL || ll->len < 6) {
		snprintf(buf, len, "?");
		return;
	}
	snprintf(buf, len, "%02x%02x%02x%02x%02x%02x", ll->addr[0], ll->addr[1], ll->addr[2],
		 ll->addr[3], ll->addr[4], ll->addr[5]);
}

/* ------------------------------------------------------------------------ */
/* Settings                                                                  */

static int lab_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	char tmp[LAB_TOKEN_MAX + 1] = {0};
	ssize_t n;

	if (len > LAB_TOKEN_MAX) {
		return -EINVAL;
	}
	n = read_cb(cb_arg, tmp, len);
	if (n < 0) {
		return (int)n;
	}
	tmp[n] = '\0';
	if (strcmp(name, "token") == 0) {
		strcpy(lab.token, tmp);
	} else if (strcmp(name, "peer") == 0) {
		strcpy(lab.peer, tmp);
	}
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(lab, "lab", NULL, lab_set, NULL, NULL);

int lab_save(const char *key, const char *val)
{
	char path[16];

	snprintf(path, sizeof(path), "lab/%s", key);
	return settings_save_one(path, val, strlen(val));
}

/* ------------------------------------------------------------------------ */
/* Local execution on the dummy backend                                      */

int lab_local_exec(const char *cmd, char *out, size_t len)
{
	const struct shell *sh = shell_backend_dummy_get_ptr();
	const char *o;
	size_t n;
	int ret;

	shell_backend_dummy_lock();
	shell_backend_dummy_clear_output(sh);
	ret = shell_execute_cmd(sh, cmd);
	o = shell_backend_dummy_get_output(sh, &n);
	while (n > 0 && (*o == '\r' || *o == '\n')) {
		o++;
		n--;
	}
	if (out != NULL && len > 0) {
		n = MIN(n, len - 1);
		memcpy(out, o, n);
		out[n] = '\0';
	}
	shell_backend_dummy_unlock();
	return ret;
}

/* ------------------------------------------------------------------------ */
/* Files (PUT)                                                               */

static struct lab_file *file_find(const char *name, bool create, size_t total)
{
	size_t used = 0;
	struct lab_file *free_slot = NULL;

	for (int i = 0; i < ARRAY_SIZE(files); i++) {
		if (files[i].used && strcmp(files[i].name, name) == 0) {
			return &files[i];
		}
	}
	if (!create) {
		return NULL;
	}
	for (int i = 0; i < ARRAY_SIZE(files); i++) {
		if (files[i].used) {
			used = MAX(used, files[i].off + ROUND_UP(files[i].cap, 64));
		} else if (free_slot == NULL) {
			free_slot = &files[i];
		}
	}
	if (free_slot == NULL || used + total > sizeof(file_pool)) {
		return NULL;
	}
	*free_slot = (struct lab_file){.used = true, .off = used, .cap = total};
	strncpy(free_slot->name, name, sizeof(free_slot->name) - 1);
	return free_slot;
}

const void *lab_file_get(const char *name, size_t *len)
{
	struct lab_file *f = file_find(name, false, 0);

	if (f == NULL || f->len != f->cap) {
		return NULL;
	}
	if (len != NULL) {
		*len = f->len;
	}
	return &file_pool[f->off];
}

struct lab_file *lab_file_slot(int i)
{
	return i < ARRAY_SIZE(files) && files[i].used ? &files[i] : NULL;
}

void lab_file_clear(void)
{
	memset(files, 0, sizeof(files));
}

static int put_ram(const char *name, uint32_t off, uint32_t total, const uint8_t *d, size_t n)
{
	struct lab_file *f = file_find(name, off == 0, total);

	if (f != NULL && off == 0 && f->cap != total) {
		/* the same name again, another size: start over */
		f->used = false;
		f = file_find(name, true, total);
	}
	if (f == NULL || off + n > f->cap) {
		return -ENOSPC;
	}
	memcpy(&file_pool[f->off + off], d, n);
	f->len = MAX(f->len, off + n);
	return 0;
}

static int put_fs(const char *path, uint32_t off, const uint8_t *d, size_t n)
{
#ifdef CONFIG_FILE_SYSTEM
	struct fs_file_t fp;
	int ret;

	fs_file_t_init(&fp);
	ret = fs_open(&fp, path, FS_O_CREATE | FS_O_WRITE);
	if (ret != 0) {
		return ret;
	}
	if (off == 0) {
		(void)fs_truncate(&fp, 0);
	}
	ret = fs_seek(&fp, off, FS_SEEK_SET);
	if (ret == 0) {
		ret = fs_write(&fp, d, n) == (ssize_t)n ? 0 : -EIO;
	}
	fs_close(&fp);
	return ret;
#else
	ARG_UNUSED(path);
	ARG_UNUSED(off);
	ARG_UNUSED(d);
	ARG_UNUSED(n);
	return -ENOTSUP;
#endif
}

/* ------------------------------------------------------------------------ */
/* Server                                                                    */

static int sock = -1;
static struct k_thread srv_thread;
static K_KERNEL_STACK_DEFINE(srv_stack, CONFIG_LAB_STACK_SIZE);

static void reply(const struct sockaddr *to, socklen_t tolen, const char *hdr,
		  const void *payload, size_t plen)
{
	size_t h = strlen(hdr);

	if (h + 1 + plen > sizeof(tx)) {
		plen = sizeof(tx) - h - 1;
	}
	memcpy(tx, hdr, h);
	tx[h] = '\n';
	if (plen > 0) {
		memcpy(&tx[h + 1], payload, plen);
	}
	(void)zsock_sendto(sock, tx, h + 1 + plen, 0, to, tolen);
}

static void send_exec_result(const struct sockaddr *to, socklen_t tolen, uint32_t id)
{
	char hdr[64];
	uint32_t seq = 0;

	for (size_t o = 0; o < last_exec.len; o += CHUNK, seq++) {
		snprintf(hdr, sizeof(hdr), MAGIC " OUT %u %u", id, seq);
		reply(to, tolen, hdr, &out_buf[o], MIN(CHUNK, last_exec.len - o));
		/* the peer's RX ring is not deep: pace the burst a bit */
		if ((seq & 7) == 7) {
			k_usleep(200);
		}
	}
	snprintf(hdr, sizeof(hdr), MAGIC " END %u %d %u %zu", id, last_exec.rc, seq,
		 last_exec.len);
	reply(to, tolen, hdr, NULL, 0);
}

static bool from_peer(const struct sockaddr_in *src)
{
	struct in_addr p;

	if (lab.peer[0] == '\0') {
		return true;
	}
	if (zsock_inet_pton(AF_INET, lab.peer, &p) != 1) {
		return false;
	}
	return p.s_addr == src->sin_addr.s_addr;
}

static void handle(const struct sockaddr *src, socklen_t slen, char *msg, size_t n)
{
	char *nl = memchr(msg, '\n', n);
	char *payload = NULL;
	size_t plen = 0;
	char *save, *tok, *op, *ids;
	char hdr[160];
	uint32_t id;

	if (nl != NULL) {
		*nl = '\0';
		payload = nl + 1;
		plen = n - (size_t)(payload - msg);
	} else {
		msg[n] = '\0';
	}
	if (strtok_r(msg, " ", &save) == NULL || strcmp(msg, MAGIC) != 0) {
		return;
	}
	tok = strtok_r(NULL, " ", &save);
	op = strtok_r(NULL, " ", &save);
	ids = strtok_r(NULL, " ", &save);
	if (tok == NULL || op == NULL || ids == NULL || strcmp(tok, lab.token) != 0 ||
	    !from_peer((const struct sockaddr_in *)src)) {
		lab.dropped++;
		return;
	}
	id = strtoul(ids, NULL, 0);
	lab.served++;
	lab.last_client = ((const struct sockaddr_in *)src)->sin_addr.s_addr;

	if (strcmp(op, "HELLO") == 0) {
		char ident[16];

		lab_identity(ident, sizeof(ident));
		snprintf(hdr, sizeof(hdr), MAGIC " HELLO %u role=%s id=%s fw=%s board=%s t=%llu", id,
			 lab_role(), ident, FW_STRING, CONFIG_BOARD,
			 (unsigned long long)lab_now_ns());
		reply(src, slen, hdr, NULL, 0);
	} else if (strcmp(op, "TIME") == 0) {
		uint64_t t2 = lab_now_ns();
		char *t1 = strtok_r(NULL, " ", &save);

		snprintf(hdr, sizeof(hdr), MAGIC " TIME %u %s %llu %llu", id, t1 ? t1 : "0",
			 (unsigned long long)t2, (unsigned long long)lab_now_ns());
		reply(src, slen, hdr, NULL, 0);
	} else if (strcmp(op, "EXEC") == 0) {
		char *cmd = save; /* the rest of the line */

		if (!(last_exec.valid && last_exec.id == id)) {
			if (cmd == NULL || *cmd == '\0') {
				return;
			}
			LOG_INF("exec %u: %s", id, cmd);
			last_exec.rc = lab_local_exec(cmd, out_buf, sizeof(out_buf));
			last_exec.len = strlen(out_buf);
			last_exec.id = id;
			last_exec.valid = true;
			lab.execs++;
		}
		send_exec_result(src, slen, id);
	} else if (strcmp(op, "PUT") == 0) {
		char *name = strtok_r(NULL, " ", &save);
		char *offs = strtok_r(NULL, " ", &save);
		char *tots = strtok_r(NULL, " ", &save);
		int ret;

		if (name == NULL || offs == NULL || tots == NULL || payload == NULL) {
			return;
		}
		if (name[0] == '/') {
			ret = put_fs(name, strtoul(offs, NULL, 0), (uint8_t *)payload, plen);
		} else {
			ret = put_ram(name, strtoul(offs, NULL, 0), strtoul(tots, NULL, 0),
				      (uint8_t *)payload, plen);
		}
		snprintf(hdr, sizeof(hdr), MAGIC " ACK %u %s %d", id, offs, ret);
		reply(src, slen, hdr, NULL, 0);
	}
}

static void srv_fn(void *a, void *b, void *c)
{
	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_port = htons(CONFIG_LAB_PORT),
		.sin_addr.s_addr = htonl(INADDR_ANY),
	};

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	/* let the network come up; settings_subsys_init() is idempotent */
	k_sleep(K_SECONDS(2));
	if (settings_subsys_init() == 0) {
		(void)settings_load_subtree("lab");
	}
	sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0 || zsock_bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
		LOG_ERR("lab: UDP port %d: %d", CONFIG_LAB_PORT, errno);
		return;
	}
	LOG_INF("lab agent on UDP %d (%s)%s", CONFIG_LAB_PORT, lab_role(),
		lab.token[0] ? "" : ", no token: requests dropped (lab token <secret>)");
	for (;;) {
		struct sockaddr_storage src;
		socklen_t slen = sizeof(src);
		ssize_t n = zsock_recvfrom(sock, rx, DGRAM_MAX, 0, (struct sockaddr *)&src, &slen);

		if (n <= 0 || src.ss_family != AF_INET || lab.token[0] == '\0') {
			continue;
		}
		handle((struct sockaddr *)&src, slen, (char *)rx, (size_t)n);
	}
}

static int lab_init(void)
{
	k_thread_create(&srv_thread, srv_stack, K_KERNEL_STACK_SIZEOF(srv_stack), srv_fn, NULL,
			NULL, NULL, CONFIG_LAB_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&srv_thread, "lab");
	return 0;
}

SYS_INIT(lab_init, APPLICATION, 99);
