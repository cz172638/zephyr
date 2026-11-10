/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * The lab agent talking to itself over host UDP (native sim offloaded
 * sockets, peer 127.0.0.1): hello, exec with multi-datagram output,
 * remote return codes, time, put into RAM slots, and the access checks.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/lab/lab.h>
#include <zephyr/shell/shell.h>
#include <zephyr/ztest.h>

#include "lab_internal.h"

static char out[CONFIG_SHELL_BACKEND_DUMMY_BUF_SIZE];

/* prints a lot: several OUT datagrams */
static int cmd_spam(const struct shell *sh, size_t argc, char **argv)
{
	int n = argc > 1 ? atoi(argv[1]) : 100;

	for (int i = 0; i < n; i++) {
		shell_print(sh, "line %04d 0123456789abcdef0123456789abcdef", i);
	}
	return 0;
}
SHELL_CMD_ARG_REGISTER(spam, NULL, "spam [n]", cmd_spam, 1, 1);

static int cmd_fail(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "failing");
	return -EIO;
}
SHELL_CMD_REGISTER(fail, NULL, "returns -EIO", cmd_fail);

static void *setup(void)
{
	/* the server starts 2 s after boot */
	k_sleep(K_SECONDS(3));
	strcpy(lab.token, "s3cret");
	strcpy(lab.peer, "127.0.0.1");
	return NULL;
}

ZTEST(lab_agent, test_hello)
{
	zassert_ok(lab_hello(NULL, out, sizeof(out), 1000));
	zassert_not_null(strstr(out, "role=node"), "%s", out);
	zassert_not_null(strstr(out, "board="), "%s", out);
}

ZTEST(lab_agent, test_exec_small)
{
	zassert_equal(lab_peer_exec("spam 1", out, sizeof(out), K_SECONDS(2)), 0);
	zassert_not_null(strstr(out, "line 0000"), "%s", out);
}

ZTEST(lab_agent, test_exec_big)
{
	char want[64];

	/* ~50 bytes a line: 200 lines = 9 OUT datagrams */
	zassert_equal(lab_peer_exec("spam 200", out, sizeof(out), K_SECONDS(5)), 0);
	snprintf(want, sizeof(want), "line %04d", 199);
	zassert_not_null(strstr(out, want), "len %zu", strlen(out));
	zassert_not_null(strstr(out, "line 0100"));
}

ZTEST(lab_agent, test_exec_rc)
{
	zassert_equal(lab_peer_exec("fail", out, sizeof(out), K_SECONDS(2)), -EIO);
	zassert_not_null(strstr(out, "failing"));
	zassert_true(lab_peer_exec("no_such_command", out, sizeof(out), K_SECONDS(2)) != 0);
}

ZTEST(lab_agent, test_time)
{
	int64_t off;
	uint64_t rtt;

	/* the same clock on both ends */
	zassert_equal(lab_peer_time(8, &off, &rtt), 8);
	zassert_true(off > -(int64_t)rtt && off < (int64_t)rtt, "off %lld rtt %llu",
		     (long long)off, (unsigned long long)rtt);
}

ZTEST(lab_agent, test_put)
{
	static uint8_t blob[5000];
	const uint8_t *got;
	size_t len;

	for (int i = 0; i < sizeof(blob); i++) {
		blob[i] = (uint8_t)(i * 7);
	}
	zassert_ok(lab_peer_put("blob", blob, sizeof(blob)));
	got = lab_file_get("blob", &len);
	zassert_not_null(got);
	zassert_equal(len, sizeof(blob));
	zassert_mem_equal(got, blob, sizeof(blob));

	/* again with another size: the slot is replaced */
	zassert_ok(lab_peer_put("blob", blob, 300));
	got = lab_file_get("blob", &len);
	zassert_equal(len, 300);
	lab_file_clear();
	zassert_is_null(lab_file_get("blob", NULL));
}

ZTEST(lab_agent, test_wrong_peer)
{
	uint32_t dropped = lab.dropped;

	strcpy(lab.peer, "127.0.0.2");
	zassert_equal(lab_hello("127.0.0.1", out, sizeof(out), 300), -ETIMEDOUT);
	zassert_true(lab.dropped > dropped);
	strcpy(lab.peer, "127.0.0.1");
}

ZTEST(lab_agent, test_no_token)
{
	lab.token[0] = '\0';
	zassert_equal(lab_hello(NULL, out, sizeof(out), 300), -EACCES);
	strcpy(lab.token, "s3cret");
}

ZTEST_SUITE(lab_agent, NULL, setup, NULL, NULL, NULL);

/* -DCONFIG_LAB_TEST_SERVE_S=n: stay up for the host client */
ZTEST(lab_agent, test_zz_serve)
{
	if (CONFIG_LAB_TEST_SERVE_S == 0) {
		ztest_test_skip();
	}
	strcpy(lab.peer, "");
	k_sleep(K_SECONDS(CONFIG_LAB_TEST_SERVE_S));
}
