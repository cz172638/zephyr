/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Role of a dual-mode PCIe controller, read once at boot before the root
 * complex and endpoint drivers initialize (see role.h).
 */

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/pcie/role.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#if defined(CONFIG_PCIE_ROLE_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

LOG_MODULE_REGISTER(pcie_role, LOG_LEVEL_INF);

#define ROLE_KEY   "pcie/mode"
/* retention register: magic in the upper half, role in bit 0 */
#define ROLE_MAGIC 0x50430000U

static enum pcie_role role = IS_ENABLED(CONFIG_PCIE_ROLE_DEFAULT_RC) ? PCIE_ROLE_RC
								    : PCIE_ROLE_EP;
static bool saved_valid;
static enum pcie_role saved;

#if defined(CONFIG_PCIE_ROLE_SETTINGS)
static int load_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
		   void *param)
{
	char buf[4] = {0};

	ARG_UNUSED(key);
	ARG_UNUSED(param);
	if (len >= sizeof(buf) || read_cb(cb_arg, buf, len) != (ssize_t)len) {
		return 0;
	}
	if (strcmp(buf, "rc") == 0) {
		saved = PCIE_ROLE_RC;
		saved_valid = true;
	} else if (strcmp(buf, "ep") == 0) {
		saved = PCIE_ROLE_EP;
		saved_valid = true;
	}
	return 0;
}
#elif CONFIG_PCIE_ROLE_RETENTION_REG != 0
static mm_reg_t ret_reg(void)
{
	static mm_reg_t va;

	if (va == 0) {
		device_map(&va, CONFIG_PCIE_ROLE_RETENTION_REG & ~0xfffUL, 0x1000,
			   K_MEM_CACHE_NONE);
	}
	return va + (CONFIG_PCIE_ROLE_RETENTION_REG & 0xfffUL);
}
#endif

enum pcie_role pcie_role_get(void)
{
	return role;
}

const char *pcie_role_storage(void)
{
#if defined(CONFIG_PCIE_ROLE_SETTINGS)
	return "settings";
#elif CONFIG_PCIE_ROLE_RETENTION_REG != 0
	return "retention register (warm reset only)";
#else
	return "none (build default)";
#endif
}

static int pcie_role_init(void)
{
#if defined(CONFIG_PCIE_ROLE_SETTINGS)
	if (settings_subsys_init() == 0) {
		(void)settings_load_subtree_direct(ROLE_KEY, load_cb, NULL);
	}
#elif CONFIG_PCIE_ROLE_RETENTION_REG != 0
	uint32_t v = sys_read32(ret_reg());

	if ((v & 0xffff0000U) == ROLE_MAGIC) {
		saved = (v & 1U) ? PCIE_ROLE_RC : PCIE_ROLE_EP;
		saved_valid = true;
	}
#endif
	if (saved_valid) {
		role = saved;
	}
	LOG_INF("PCIe controller as %s%s", role == PCIE_ROLE_RC ? "root complex" : "endpoint",
		saved_valid ? "" : " (default)");
	return 0;
}

SYS_INIT(pcie_role_init, POST_KERNEL, CONFIG_PCIE_ROLE_INIT_PRIORITY);

int pcie_role_save(enum pcie_role r)
{
#if defined(CONFIG_PCIE_ROLE_SETTINGS)
	const char *v = r == PCIE_ROLE_RC ? "rc" : "ep";
	int ret = settings_save_one(ROLE_KEY, v, strlen(v));

	if (ret != 0) {
		return ret;
	}
#elif CONFIG_PCIE_ROLE_RETENTION_REG != 0
	sys_write32(ROLE_MAGIC | (r == PCIE_ROLE_RC ? 1U : 0U), ret_reg());
#else
	ARG_UNUSED(r);
	return -ENOTSUP;
#endif
	saved = r;
	saved_valid = true;
	return 0;
}

bool pcie_role_saved(enum pcie_role *r)
{
	if (saved_valid) {
		*r = saved;
	}
	return saved_valid;
}
