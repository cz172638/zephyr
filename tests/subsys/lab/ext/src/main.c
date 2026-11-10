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

	zassert_equal(lab_ext_run(NULL, "lab_hello", ext, sizeof(ext), NULL, 2, argv, false), 0);
}

ZTEST(lab_ext, test_reload)
{
	char *argv[] = {"lab_hello"};

	/* data and bss come back fresh on every load (counter == 7 check) */
	for (int i = 0; i < 5; i++) {
		zassert_equal(lab_ext_run(NULL, "lab_hello", ext, sizeof(ext), NULL, 1, argv, false), 0,
			      "run %d", i);
	}
}

ZTEST(lab_ext, test_missing_fn)
{
	zassert_equal(lab_ext_run(NULL, "lab_hello", ext, sizeof(ext), "nope", 0, NULL, false), -ENOENT);
}

ZTEST(lab_ext, test_bad_image)
{
	static uint8_t bad[256];

	memcpy(bad, ext, sizeof(bad));
	bad[1] = 'X'; /* ELF magic */
	zassert_true(lab_ext_run(NULL, "bad", bad, sizeof(bad), NULL, 0, NULL, false) < 0);
}



/* ---- debug modules (lab mod) ---- */

#include <zephyr/lab/mod.h>
#include <zephyr/llext/symbol.h>
#include <zephyr/shell/shell_dummy.h>

int lab_test_inits, lab_test_finis, lab_test_calls;
EXPORT_SYMBOL(lab_test_calls);
EXPORT_SYMBOL(lab_test_inits);
EXPORT_SYMBOL(lab_test_finis);

static const uint8_t mod[] __aligned(8) = {
#include "mod.inc"
};

static const uint8_t mod99[] __aligned(8) = {
#include "mod99.inc"
};

static const char *run(const char *cmd, int *ret)
{
	const struct shell *sh = shell_backend_dummy_get_ptr();
	size_t n;

	shell_backend_dummy_clear_output(sh);
	*ret = shell_execute_cmd(sh, cmd);
	return shell_backend_dummy_get_output(sh, &n);
}

ZTEST(lab_ext, test_mod)
{
	const struct shell *sh = shell_backend_dummy_get_ptr();
	struct shell_static_entry e;
	const char *out;
	int ret;

	lab_test_inits = lab_test_finis = 0;
	zassert_ok(lab_mod_load(sh, mod, sizeof(mod), "7-labt.llext", false));
	zassert_equal(lab_test_inits, 1);
	zassert_true(lab_mod_get("dbg", 0, &e));
	zassert_str_equal(e.syntax, "labt");
	zassert_false(lab_mod_get("pcie", 0, &e));

	out = run("dbg labt hello", &ret);
	zassert_ok(ret);
	zassert_not_null(strstr(out, "labt hello calls 1"), "ret %d out [%s]", ret, out);
	out = run("dbg labt sub", &ret);
	zassert_ok(ret);
	zassert_not_null(strstr(out, "labt sub ok"), "%s", out);

	/* reload by name: fini of the old, init of the new, fresh data */
	zassert_ok(lab_mod_load(sh, mod, sizeof(mod), "7-labt.llext", false));
	zassert_equal(lab_test_finis, 1);
	zassert_equal(lab_test_inits, 2);
	out = run("dbg labt again", &ret);
	zassert_not_null(strstr(out, "calls 1"), "%s", out);

	zassert_ok(lab_mod_unload(sh, "labt"));
	zassert_equal(lab_test_finis, 2);
	out = run("dbg labt", &ret);
	zassert_true(ret != 0);
	zassert_equal(lab_mod_unload(sh, "labt"), -ENOENT);
}

ZTEST(lab_ext, test_mod_image)
{
	const struct shell *sh = shell_backend_dummy_get_ptr();
	int ret;

	/* built for image 99, this is 7 */
	zassert_equal(lab_mod_load(sh, mod99, sizeof(mod99), "99-labt.llext", false), -ENOEXEC);
	zassert_ok(lab_mod_load(sh, mod99, sizeof(mod99), "99-labt.llext", true));
	(void)run("dbg labt", &ret);
	zassert_ok(ret);
	zassert_ok(lab_mod_unload(sh, "all"));
}

ZTEST(lab_ext, test_mod_slots)
{
	const struct shell *sh = shell_backend_dummy_get_ptr();

	/* the same command again replaces, so slots never run out */
	for (int i = 0; i < 2 * CONFIG_LAB_MODS; i++) {
		zassert_ok(lab_mod_load(sh, mod, sizeof(mod), "7-labt.llext", false), "%d", i);
	}
	zassert_ok(lab_mod_unload(sh, "all"));
}


static void *suite_setup(void)
{
	/* the dummy shell takes output only once its thread made it active */
	k_sleep(K_MSEC(200));
	return NULL;
}

ZTEST_SUITE(lab_ext, NULL, suite_setup, NULL, NULL, NULL);
