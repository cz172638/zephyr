/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Lab agent: boards on one network segment driving each other's
 *        shells for tests (debug builds only)
 *
 * UDP, one request per datagram, text header line plus optional raw
 * payload:
 *
 *   request  "LAB1 <token> <op> <id> [args]\n[payload]"
 *   reply    "LAB1 <op> <id> [args]\n[payload]"
 *
 *   HELLO                    -> HELLO role=.. id=.. fw=.. board=.. t=<ns>
 *   EXEC <cmd...>            -> OUT <seq>\n<text> ...  END <rc> <chunks> <len>
 *   TIME <t1>                -> TIME <t1> <t2> <t3>      (ns, NTP style)
 *   PUT <name> <off> <total>\n<bytes>  -> ACK <off>   (RAM slot or /path)
 *
 * A request repeated with the same id gets the cached reply (lost
 * datagrams are retried by the client). Requests without the token, or
 * from another address than the configured peer, are dropped.
 */

#ifndef ZEPHYR_INCLUDE_LAB_LAB_H_
#define ZEPHYR_INCLUDE_LAB_LAB_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Local clock for lab time stamps, ns */
uint64_t lab_now_ns(void);

/**
 * Run a shell command here; output (NUL terminated, CRLF as the shell
 * makes it) into @p out. Returns the command's return value.
 */
int lab_local_exec(const char *cmd, char *out, size_t len);

/**
 * Run a shell command on the peer. Returns the remote command's return
 * value, or -ETIMEDOUT / -ENOTCONN (no peer) / -EACCES (no token).
 */
int lab_peer_exec(const char *cmd, char *out, size_t len, k_timeout_t timeout);

/**
 * Clock offset to the peer (peer = local + offset) from @p n TIME
 * exchanges, the one with the shortest round trip wins.
 */
int lab_peer_time(int n, int64_t *offset_ns, uint64_t *rtt_ns);

/** Last measured offset (0 before lab_peer_time()) */
int64_t lab_peer_offset_ns(void);

/** Send @p len bytes to the peer as @p name (RAM slot, or a path from '/') */
int lab_peer_put(const char *name, const void *data, size_t len);

/** A received RAM slot by name: data, or NULL */
const void *lab_file_get(const char *name, size_t *len);

/**
 * @brief What the firmware hands a test extension (lab_ext_main()).
 *
 * Extensions only see what the kernel exports by EXPORT_SYMBOL (little
 * more than printk); everything else a test needs comes through this
 * table, so a module built once keeps working across firmware builds
 * as long as LAB_EXT_API_VERSION matches.
 */
#define LAB_EXT_API_VERSION 1

struct lab_ext_api {
	uint32_t version;
	uint32_t size;
	void (*printf)(const char *fmt, ...);
	uint64_t (*now_ns)(void);
	void (*busy_wait_us)(uint32_t us);
	void (*sleep_ms)(uint32_t ms);
	int (*local_exec)(const char *cmd, char *out, size_t len);
	int (*peer_exec)(const char *cmd, char *out, size_t len, uint32_t timeout_ms);
	int64_t (*peer_offset_ns)(void);
	const char *(*role)(void);
	/* device memory, mapped uncached; NULL on failure */
	void *(*map_io)(uintptr_t phys, size_t size);
	void (*cache_flush)(void *addr, size_t size);
	void (*cache_invd)(void *addr, size_t size);
	void *(*alloc)(size_t size, size_t align);
	void (*free)(void *ptr);
};

/** Entry point a test extension exports: return 0 = pass */
typedef int (*lab_ext_main_t)(const struct lab_ext_api *api, int argc, char **argv);
#define LAB_EXT_MAIN "lab_ext_main"

/**
 * Load an extension from memory, run lab_ext_main (or, missing that,
 * the void(void) function  fn), unload. Returns its result or a
 * negative loader error.
 */
int lab_ext_run(const char *name, const void *elf, size_t len, const char *fn, int argc,
		char **argv);

#ifdef __cplusplus
}
#endif

#endif
