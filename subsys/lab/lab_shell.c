/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * "lab" shell commands.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/lab/lab.h>
#include <zephyr/lab/mod.h>
#include <zephyr/net/socket.h>
#include <zephyr/shell/shell.h>

#include "lab_internal.h"

static char peer_out[CONFIG_SHELL_BACKEND_DUMMY_BUF_SIZE];

static void join(char *buf, size_t len, size_t argc, char **argv)
{
	size_t o = 0;

	buf[0] = '\0';
	for (size_t i = 0; i < argc && o < len; i++) {
		o += snprintf(&buf[o], len - o, "%s%s", i ? " " : "", argv[i]);
	}
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	char ident[16];
	char last[16] = "-";

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	lab_identity(ident, sizeof(ident));
	if (lab.last_client != 0) {
		struct in_addr a = {.s_addr = lab.last_client};

		zsock_inet_ntop(AF_INET, &a, last, sizeof(last));
	}
	shell_print(sh, "role %s, id %s, image %d, UDP %d", lab_role(), ident, CONFIG_LAB_IMAGE,
		    CONFIG_LAB_PORT);
	shell_print(sh, "token %s, peer %s", lab.token[0] ? "set" : "NONE (agent deaf)",
		    lab.peer[0] ? lab.peer : "any");
	shell_print(sh, "served %u, dropped %u, execs %u, last client %s", lab.served,
		    lab.dropped, lab.execs, last);
	shell_print(sh, "peer clock offset %lld ns", (long long)lab_peer_offset_ns());
	return 0;
}

static int cmd_token(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "token %s", lab.token[0] ? "set" : "not set");
		return 0;
	}
	if (strcmp(argv[1], "-") == 0) {
		lab.token[0] = '\0';
	} else if (strlen(argv[1]) > LAB_TOKEN_MAX || strchr(argv[1], ' ') != NULL) {
		shell_error(sh, "token: up to %d chars, no spaces", LAB_TOKEN_MAX);
		return -EINVAL;
	} else {
		strcpy(lab.token, argv[1]);
	}
	return lab_save("token", lab.token);
}

static int cmd_peer(const struct shell *sh, size_t argc, char **argv)
{
	struct in_addr a;

	if (argc < 2) {
		shell_print(sh, "peer %s", lab.peer[0] ? lab.peer : "any (not set)");
		return 0;
	}
	if (strcmp(argv[1], "-") == 0) {
		lab.peer[0] = '\0';
	} else if (zsock_inet_pton(AF_INET, argv[1], &a) != 1) {
		shell_error(sh, "peer: IPv4 address");
		return -EINVAL;
	} else {
		strncpy(lab.peer, argv[1], sizeof(lab.peer) - 1);
	}
	return lab_save("peer", lab.peer);
}

static int cmd_hello(const struct shell *sh, size_t argc, char **argv)
{
	int64_t t0 = k_uptime_get();
	int ret = lab_hello(argc > 1 ? argv[1] : NULL, peer_out, sizeof(peer_out), 1000);

	if (ret != 0) {
		shell_error(sh, "hello: %d", ret);
		return ret;
	}
	shell_print(sh, "%s (%lld ms)", peer_out, k_uptime_get() - t0);
	return 0;
}

static int cmd_exec(const struct shell *sh, size_t argc, char **argv)
{
	char cmd[256];
	int ret;

	join(cmd, sizeof(cmd), argc - 1, &argv[1]);
	ret = lab_peer_exec(cmd, peer_out, sizeof(peer_out), K_SECONDS(CONFIG_LAB_EXEC_TIMEOUT));
	shell_fprintf(sh, SHELL_NORMAL, "%s", peer_out);
	if (ret == -ETIMEDOUT || ret == -ENOTCONN || ret == -EACCES) {
		shell_error(sh, "lab exec: %d", ret);
	} else if (ret != 0) {
		shell_warn(sh, "[peer rc %d]", ret);
	}
	return ret;
}

static int cmd_time(const struct shell *sh, size_t argc, char **argv)
{
	int n = argc > 1 ? atoi(argv[1]) : 16;
	int64_t off;
	uint64_t rtt;
	int ret = lab_peer_time(n, &off, &rtt);

	if (ret < 0) {
		shell_error(sh, "time: %d", ret);
		return ret;
	}
	shell_print(sh, "offset %lld ns (peer = local + offset), best rtt %llu ns, %d/%d ok",
		    (long long)off, (unsigned long long)rtt, ret, n);
	return 0;
}

/* a RAM slot by name, or a file from '/' read into the heap */
static const void *source(const struct shell *sh, const char *name, size_t *len, void **heap)
{
#ifdef CONFIG_FILE_SYSTEM
	struct fs_dirent st;
	struct fs_file_t fp;
	void *buf;
#endif

	*heap = NULL;
	if (name[0] != '/') {
		const void *d = lab_file_get(name, len);

		if (d == NULL) {
			shell_error(sh, "%s: no such slot", name);
		}
		return d;
	}
#ifndef CONFIG_FILE_SYSTEM
	shell_error(sh, "%s: no file system in this build", name);
	return NULL;
#else
	if (fs_stat(name, &st) != 0 || st.type != FS_DIR_ENTRY_FILE) {
		shell_error(sh, "%s: not found", name);
		return NULL;
	}
	buf = k_malloc(MAX(st.size, 1));
	if (buf == NULL) {
		shell_error(sh, "%s: %zu bytes, no memory", name, st.size);
		return NULL;
	}
	fs_file_t_init(&fp);
	if (fs_open(&fp, name, FS_O_READ) != 0 || fs_read(&fp, buf, st.size) != (ssize_t)st.size) {
		shell_error(sh, "%s: read failed", name);
		fs_close(&fp);
		k_free(buf);
		return NULL;
	}
	fs_close(&fp);
	*heap = buf;
	*len = st.size;
	return buf;
#endif
}

static int cmd_put(const struct shell *sh, size_t argc, char **argv)
{
	const char *dst = argc > 2 ? argv[2] : argv[1];
	const void *d;
	void *heap;
	size_t len;
	int64_t t0;
	int ret;

	d = source(sh, argv[1], &len, &heap);
	if (d == NULL) {
		return -ENOENT;
	}
	if (dst[0] == '/' && argc < 3) {
		/* /SD:/x.llext -> slot "x.llext" on the peer */
		dst = strrchr(dst, '/') + 1;
	}
	t0 = k_uptime_get();
	ret = lab_peer_put(dst, d, len);
	k_free(heap);
	if (ret != 0) {
		shell_error(sh, "put: %d", ret);
		return ret;
	}
	shell_print(sh, "%s -> peer:%s, %zu bytes, %lld ms", argv[1], dst, len,
		    k_uptime_get() - t0);
	return 0;
}

static int cmd_files(const struct shell *sh, size_t argc, char **argv)
{
	struct lab_file *f;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	for (int i = 0; i < CONFIG_LAB_FILES; i++) {
		f = lab_file_slot(i);
		if (f != NULL) {
			shell_print(sh, "%-24s %8zu/%zu%s", f->name, f->len, f->cap,
				    f->len == f->cap ? "" : " (incomplete)");
		}
	}
	return 0;
}

static int cmd_rm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	lab_file_clear();
	return 0;
}

#ifdef CONFIG_LAB_EXT
static int cmd_ext(const struct shell *sh, size_t argc, char **argv)
{
	bool force = argc > 2 && strcmp(argv[1], "-f") == 0;
	const char *base;
	const void *d;
	void *heap;
	size_t len;
	int ret;

	if (force) {
		argc--;
		argv++;
	}
	d = source(sh, argv[1], &len, &heap);
	if (d == NULL) {
		return -ENOENT;
	}
	base = strrchr(argv[1], '/') ? strrchr(argv[1], '/') + 1 : argv[1];
	/* argv[1] stays as the extension's argv[0] */
	ret = lab_ext_run(sh, base, d, len, NULL, (int)argc - 1, &argv[1], force);
	k_free(heap);
	if (ret == 0) {
		shell_print(sh, "%s: PASS", base);
	} else {
		shell_error(sh, "%s: FAIL %d", base, ret);
	}
	return ret;
}

static int cmd_mod_load(const struct shell *sh, size_t argc, char **argv)
{
	int fails = 0;
	bool force = strcmp(argv[1], "-f") == 0;

	for (size_t i = force ? 2 : 1; i < argc; i++) {
		const char *base = strrchr(argv[i], '/') ? strrchr(argv[i], '/') + 1 : argv[i];
		const void *d;
		void *heap;
		size_t len;

		d = source(sh, argv[i], &len, &heap);
		if (d == NULL || lab_mod_load(sh, d, len, base, force) != 0) {
			fails++;
		}
		k_free(heap);
	}
	return fails ? -EIO : 0;
}

static int cmd_mod_unload(const struct shell *sh, size_t argc, char **argv)
{
	int ret = 0;

	for (size_t i = 1; i < argc; i++) {
		ret |= lab_mod_unload(sh, argv[i]);
	}
	return ret;
}

static int cmd_mod_ls(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	lab_mod_list(sh);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(lab_mod_cmds,
	SHELL_CMD_ARG(load, NULL, "[-f] <slot|/path>...  Load debug modules (reload by name; -f: other image)",
		      cmd_mod_load, 2, 8),
	SHELL_CMD_ARG(unload, NULL, "<command|all>...  Unload", cmd_mod_unload, 2, 8),
	SHELL_CMD_ARG(ls, NULL, "Loaded modules", cmd_mod_ls, 1, 0),
	SHELL_SUBCMD_SET_END);
#endif

SHELL_SUBCMD_SET_CREATE(lab_cmds, (lab));

SHELL_SUBCMD_ADD((lab), status, NULL, "Agent state", cmd_status, 1, 0);
SHELL_SUBCMD_ADD((lab), token, NULL, "token [<secret>|-]: shared secret (saved)", cmd_token, 1, 1);
SHELL_SUBCMD_ADD((lab), peer, NULL, "peer [<ipv4>|-]: the other board (saved)", cmd_peer, 1, 1);
SHELL_SUBCMD_ADD((lab), hello, NULL, "hello [<ipv4>]: who is there", cmd_hello, 1, 1);
SHELL_SUBCMD_ADD((lab), exec, NULL, "exec <cmd...>: run a shell command on the peer", cmd_exec,
		 2, 32);
SHELL_SUBCMD_ADD((lab), time, NULL, "time [n]: clock offset to the peer", cmd_time, 1, 1);
SHELL_SUBCMD_ADD((lab), put, NULL,
		 "put <slot|/path> [<peer slot|/peer path>]: send a file to the peer", cmd_put, 2,
		 1);
SHELL_SUBCMD_ADD((lab), files, NULL, "Received RAM slots", cmd_files, 1, 0);
SHELL_SUBCMD_ADD((lab), rm, NULL, "Drop all RAM slots", cmd_rm, 1, 0);
#ifdef CONFIG_LAB_EXT
SHELL_SUBCMD_ADD((lab), ext, NULL, "ext [-f] <slot|/path> [args...]: run a test extension", cmd_ext,
		 2, 16);
SHELL_SUBCMD_ADD((lab), mod, &lab_mod_cmds, "Debug modules: load, unload, ls", cmd_mod_ls, 1, 0);
#endif

SHELL_CMD_REGISTER(lab, &lab_cmds, "Lab agent: drive the peer board for tests", NULL);
