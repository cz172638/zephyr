/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * A lab debug module: "dbg labt [arg]" with a subcommand, init and fini
 * reported to the firmware through exported counters.
 */

#ifdef LAB_TEST_OTHER_IMAGE
#undef CONFIG_LAB_IMAGE
#define CONFIG_LAB_IMAGE 99
#endif

#include <zephyr/lab/mod.h>
#include <zephyr/shell/shell.h>

extern int lab_test_inits, lab_test_finis, lab_test_calls;

static int calls;

static int cmd_labt(const struct shell *sh, size_t argc, char **argv)
{
	calls++;
	lab_test_calls++;
	shell_print(sh, "labt %s calls %d", argc > 1 ? argv[1] : "-", calls);
	return 0;
}

static int cmd_sub(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "labt sub ok");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(labt_cmds,
	SHELL_CMD_ARG(sub, NULL, "a subcommand", cmd_sub, 1, 0),
	SHELL_SUBCMD_SET_END);

static int labt_init(void)
{
	lab_test_inits++;
	return 0;
}

static void labt_fini(void)
{
	lab_test_finis++;
}

LAB_DBG_CMD((dbg), "dbg", labt, &labt_cmds, "lab module test", cmd_labt, 1, 1, labt_init,
	    labt_fini);
