/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Which endpoint function binds: several can be described for the same
 * function number (test, nvme, vntb ...); only the selected one claims
 * it this boot. The selection is the setting pcie/ep/func ("pcie
 * controller ep function <name>"), else CONFIG_PCIE_EPF_DEFAULT_FUNCTION,
 * else the first function that asks. Every function asking is listed.
 */

#include <string.h>

#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(pcie_epf_select, CONFIG_PCIE_EP_LOG_LEVEL);

#define EPF_FUNC_KEY "pcie/ep/func"
#define MAX_NAMES    8

static const char *names[MAX_NAMES];
static size_t n_names;
static char saved[16];
static bool loaded;
static const char *bound;

static int load_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
		   void *param)
{
	ARG_UNUSED(key);
	ARG_UNUSED(param);
	if (len < sizeof(saved) && read_cb(cb_arg, saved, len) == (ssize_t)len) {
		saved[len] = '\0';
	}
	return 0;
}

static void load(void)
{
	if (loaded) {
		return;
	}
	loaded = true;
	if (IS_ENABLED(CONFIG_SETTINGS) && settings_subsys_init() == 0) {
		(void)settings_load_subtree_direct(EPF_FUNC_KEY, load_cb, NULL);
	}
}

const char *pcie_epf_selection(void)
{
	load();
	if (saved[0] != '\0') {
		return saved;
	}
	return CONFIG_PCIE_EPF_DEFAULT_FUNCTION[0] != '\0' ? CONFIG_PCIE_EPF_DEFAULT_FUNCTION
							   : NULL;
}

bool pcie_epf_selected(const char *name)
{
	const char *sel = pcie_epf_selection();
	bool yes;

	if (n_names < MAX_NAMES) {
		names[n_names++] = name;
	}
	if (sel != NULL) {
		yes = strcmp(sel, name) == 0;
	} else {
		yes = bound == NULL; /* nothing chosen: the first one */
	}
	if (yes && bound == NULL) {
		bound = name;
	} else if (yes) {
		yes = false; /* one function per controller function number */
	}
	if (!yes) {
		LOG_INF("function \"%s\" not selected (%s)", name,
			sel != NULL ? sel : (bound != NULL ? bound : "-"));
	}
	return yes;
}

const char *pcie_epf_bound(void)
{
	return bound;
}

size_t pcie_epf_names(const char **out, size_t max)
{
	size_t n = MIN(max, n_names);

	for (size_t i = 0; i < n; i++) {
		out[i] = names[i];
	}
	return n_names;
}

int pcie_epf_select(const char *name)
{
	bool known = false;

	for (size_t i = 0; i < n_names; i++) {
		known |= strcmp(names[i], name) == 0;
	}
	if (!known || strlen(name) >= sizeof(saved)) {
		return -ENOENT;
	}
	strcpy(saved, name);
	if (!IS_ENABLED(CONFIG_SETTINGS)) {
		return -ENOTSUP;
	}
	return settings_save_one(EPF_FUNC_KEY, name, strlen(name));
}

int pcie_epf_select_reset(void)
{
	saved[0] = '\0';
	return IS_ENABLED(CONFIG_SETTINGS) ? settings_delete(EPF_FUNC_KEY) : -ENOTSUP;
}
