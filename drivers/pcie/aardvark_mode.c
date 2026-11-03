/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Role of the Armada 3700 PCIe controller, from settings ("advk/mode").
 * Loaded once after the settings flash is up and before either driver
 * initializes; a change is applied at the next boot.
 */

#include <string.h>

#include <zephyr/drivers/pcie/aardvark.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(advk_mode, LOG_LEVEL_INF);

#define MODE_KEY "advk/mode"

BUILD_ASSERT(CONFIG_PCIE_AARDVARK_MODE_INIT_PRIORITY > CONFIG_SPI_NOR_INIT_PRIORITY ||
		     !IS_ENABLED(CONFIG_SPI_NOR),
	     "the mode is read from settings: after the SPI NOR");
#if defined(CONFIG_PCIE_EPC_AARDVARK)
BUILD_ASSERT(CONFIG_PCIE_AARDVARK_MODE_INIT_PRIORITY < CONFIG_PCIE_EPC_INIT_PRIORITY,
	     "the endpoint driver needs the mode");
#endif
#if defined(CONFIG_PCIE_AARDVARK)
BUILD_ASSERT(CONFIG_PCIE_AARDVARK_MODE_INIT_PRIORITY < CONFIG_PCIE_AARDVARK_INIT_PRIORITY,
	     "the root complex driver needs the mode");
#endif

static enum advk_mode mode = IS_ENABLED(CONFIG_PCIE_AARDVARK_MODE_DEFAULT_RC) ? ADVK_MODE_RC
									     : ADVK_MODE_EP;
static bool saved_valid;
static enum advk_mode saved;

static int parse(const char *s, enum advk_mode *m)
{
	if (strcmp(s, "rc") == 0) {
		*m = ADVK_MODE_RC;
	} else if (strcmp(s, "ep") == 0) {
		*m = ADVK_MODE_EP;
	} else {
		return -EINVAL;
	}
	return 0;
}

static int load_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
		   void *param)
{
	char buf[4] = {0};

	ARG_UNUSED(key);
	ARG_UNUSED(param);

	if (len >= sizeof(buf) || read_cb(cb_arg, buf, len) != (ssize_t)len) {
		return 0;
	}
	if (parse(buf, &saved) == 0) {
		saved_valid = true;
	}
	return 0;
}

enum advk_mode advk_mode_get(void)
{
	return mode;
}

static int advk_mode_init(void)
{
	int ret = settings_subsys_init();

	if (ret == 0) {
		(void)settings_load_subtree_direct(MODE_KEY, load_cb, NULL);
	}
	if (saved_valid) {
		mode = saved;
	}
	LOG_INF("PCIe controller as %s%s", mode == ADVK_MODE_RC ? "root complex" : "endpoint",
		saved_valid ? "" : " (default)");
	return 0;
}

SYS_INIT(advk_mode_init, POST_KERNEL, CONFIG_PCIE_AARDVARK_MODE_INIT_PRIORITY);

#if defined(CONFIG_SHELL)
static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
	enum advk_mode m;
	int ret;

	if (argc < 2) {
		shell_print(sh, "running as %s, saved: %s", mode == ADVK_MODE_RC ? "rc" : "ep",
			    !saved_valid ? "none (default)" : saved == ADVK_MODE_RC ? "rc" : "ep");
		return 0;
	}
	if (parse(argv[1], &m) != 0) {
		shell_error(sh, "rc or ep");
		return -EINVAL;
	}
	ret = settings_save_one(MODE_KEY, argv[1], strlen(argv[1]));
	if (ret != 0) {
		shell_error(sh, "saving failed (%d)", ret);
		return ret;
	}
	saved = m;
	saved_valid = true;
	shell_print(sh, "saved %s; %s", argv[1],
		    m == mode ? "already running so" : "takes effect after: kernel reboot cold");
	return 0;
}

SHELL_SUBCMD_SET_CREATE(advk_cmds, (advk));
SHELL_SUBCMD_ADD((advk), mode, NULL, "[rc|ep] Controller role, from the next boot", cmd_mode, 1,
		 1);
SHELL_CMD_REGISTER(advk, &advk_cmds, "Armada 3700 PCIe controller", NULL);
#endif
