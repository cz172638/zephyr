/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * The lab test extension through the lab loader (lab_ext_run), with
 * the network side stubbed: relocations, data/bss reset between loads,
 * the api table, and errors for a bad image or a missing entry point.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/lab/lab.h>
#include <zephyr/logging/log.h>
#include <zephyr/ztest.h>

#include "lab_internal.h"

LOG_MODULE_REGISTER(lab, CONFIG_LAB_LOG_LEVEL);

static const uint8_t ext[] __aligned(8) = {
#include "lab_hello.inc"
};

uint64_t lab_now_ns(void)
{
	return k_cyc_to_ns_floor64(k_cycle_get_64());
}

int lab_local_exec(const char *cmd, char *out, size_t len)
{
	ARG_UNUSED(cmd);
	if (len > 0) {
		out[0] = '\0';
	}
	return 0;
}

int lab_peer_exec(const char *cmd, char *out, size_t len, k_timeout_t timeout)
{
	ARG_UNUSED(cmd);
	ARG_UNUSED(out);
	ARG_UNUSED(len);
	ARG_UNUSED(timeout);
	return -ENOTCONN;
}

int64_t lab_peer_offset_ns(void)
{
	return 1234;
}

const char *lab_role(void)
{
	return "test";
}

ZTEST(lab_ext, test_run)
{
	char *argv[] = {"lab_hello", "x"};

	zassert_equal(lab_ext_run("lab_hello", ext, sizeof(ext), NULL, 2, argv), 0);
}

ZTEST(lab_ext, test_reload)
{
	char *argv[] = {"lab_hello"};

	/* data and bss come back fresh on every load (counter == 7 check) */
	for (int i = 0; i < 5; i++) {
		zassert_equal(lab_ext_run("lab_hello", ext, sizeof(ext), NULL, 1, argv), 0,
			      "run %d", i);
	}
}

ZTEST(lab_ext, test_missing_fn)
{
	zassert_equal(lab_ext_run("lab_hello", ext, sizeof(ext), "nope", 0, NULL), -ENOENT);
}

ZTEST(lab_ext, test_bad_image)
{
	static uint8_t bad[256];

	memcpy(bad, ext, sizeof(bad));
	bad[1] = 'X'; /* ELF magic */
	zassert_true(lab_ext_run("bad", bad, sizeof(bad), NULL, 0, NULL) < 0);
}

ZTEST_SUITE(lab_ext, NULL, NULL, NULL, NULL, NULL);
