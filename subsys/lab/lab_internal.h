/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_LAB_LAB_INTERNAL_H_
#define ZEPHYR_SUBSYS_LAB_LAB_INTERNAL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LAB_TOKEN_MAX 64

struct lab_state {
	char token[LAB_TOKEN_MAX + 1];
	char peer[16];
	uint32_t served, dropped, execs;
	uint32_t last_client; /* network order */
};

struct lab_file {
	char name[24];
	bool used;
	size_t off, cap, len;
};

extern struct lab_state lab;

const char *lab_role(void);
void lab_identity(char *buf, size_t len);
int lab_save(const char *key, const char *val);
struct lab_file *lab_file_slot(int i);
void lab_file_clear(void);

/* client side: one request, replies into rbuf; returns reply count */
int lab_hello(const char *addr, char *out, size_t len, uint32_t timeout_ms);

struct shell;
void lab_mod_list(const struct shell *sh);

#endif
