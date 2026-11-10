/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * "lab selftest": run the built-in test extension (subsys/lab/ext/lab_hello.c,
 * embedded at build time) through the same loader as lab ext.
 */

#include <stdlib.h>

#include <zephyr/lab/lab.h>
#include <zephyr/shell/shell.h>

static const uint8_t lab_hello_llext[] __aligned(8) = {
#include "lab_hello.inc"
};

static int cmd_selftest(const struct shell *sh, size_t argc, char **argv)
{
	int runs = argc > 1 ? atoi(argv[1]) : 1;
	char *xargv[] = {"lab_hello", NULL};

	for (int i = 0; i < runs; i++) {
		int ret = lab_ext_run("lab_hello", lab_hello_llext, sizeof(lab_hello_llext), NULL,
				      1, xargv);

		if (ret != 0) {
			shell_error(sh, "selftest run %d: FAIL %d", i, ret);
			return ret;
		}
	}
	shell_print(sh, "selftest: PASS (%d run%s, %zu byte extension)", runs, runs > 1 ? "s" : "",
		    sizeof(lab_hello_llext));
	return 0;
}

SHELL_SUBCMD_ADD((lab), selftest, NULL, "selftest [runs]: load/run/unload the built-in LLEXT",
		 cmd_selftest, 1, 1);
