/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Debug modules: LLEXT extensions that stay loaded and add shell
 * commands under a mount point (zephyr/lab/mod.h). The firmware keeps
 * only this loader; debug commands change without reflashing.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/lab/mod.h>
#include <zephyr/llext/buf_loader.h>
#include <zephyr/llext/llext.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include "lab_internal.h"

LOG_MODULE_DECLARE(lab, CONFIG_LAB_LOG_LEVEL);

struct mod_slot {
	struct llext *ext;
	const struct lab_mod *m;
	char file[24];
	size_t size;
	int64_t loaded_ms;
};

static struct mod_slot mods[CONFIG_LAB_MODS];
static K_MUTEX_DEFINE(mod_lock);

static struct mod_slot *find(const char *name)
{
	for (int i = 0; i < ARRAY_SIZE(mods); i++) {
		if (mods[i].m != NULL && strcmp(mods[i].m->cmd.syntax, name) == 0) {
			return &mods[i];
		}
	}
	return NULL;
}

static void drop(struct mod_slot *s)
{
	const struct lab_mod *m = s->m;

	/* hide the command first: nothing new enters the module */
	s->m = NULL;
	if (m->fini != NULL) {
		m->fini();
	}
	(void)llext_teardown(s->ext);
	llext_unload(&s->ext);
	memset(s, 0, sizeof(*s));
}

int lab_mod_load(const struct shell *sh, const void *elf, size_t len, const char *file,
		 bool force)
{
	struct llext_buf_loader buf_loader = LLEXT_TEMPORARY_BUF_LOADER(elf, len);
	struct llext_load_param ldr_parm = LLEXT_LOAD_PARAM_DEFAULT;
	struct mod_slot *s = NULL, *old;
	const struct lab_mod *m;
	char lname[LLEXT_MAX_NAME_LEN + 1];
	struct llext *ext = NULL;
	int ret;

	k_mutex_lock(&mod_lock, K_FOREVER);
	for (int i = 0; i < ARRAY_SIZE(mods); i++) {
		if (mods[i].ext == NULL) {
			s = &mods[i];
			break;
		}
	}
	if (s == NULL) {
		shell_error(sh, "no free module slot (%d)", CONFIG_LAB_MODS);
		ret = -ENOMEM;
		goto out;
	}
	/* llext names must be unique: slot number keeps a reload apart */
	snprintk(lname, sizeof(lname), "m%d_%s", (int)(s - mods), file);
	ret = llext_load(&buf_loader.loader, lname, &ext, &ldr_parm);
	if (ret != 0) {
		shell_error(sh, "%s: load failed %d (missing symbol? see log; firmware too old?)",
			    file, ret);
		goto out;
	}
	ret = llext_bringup(ext);
	m = llext_find_sym(&ext->exp_tab, LAB_MOD_SYMBOL);
	if (ret != 0 || m == NULL || m->version != LAB_MOD_VERSION || m->cmd.syntax == NULL) {
		shell_error(sh, "%s: not a lab module (%s, version %u, want %u)", file,
			    m ? "descriptor" : "no lab_mod", m ? m->version : 0, LAB_MOD_VERSION);
		ret = -ENOEXEC;
		goto fail;
	}
	ret = lab_image_check(sh, ext, file, force);
	if (ret != 0) {
		goto fail;
	}
	old = find(m->cmd.syntax);
	if (old != NULL) {
		shell_print(sh, "%s: replacing the loaded %s", m->cmd.syntax, old->file);
		drop(old);
	}
	if (m->init != NULL) {
		ret = m->init();
		if (ret != 0) {
			shell_error(sh, "%s: init %d", m->cmd.syntax, ret);
			goto fail;
		}
	}
	s->ext = ext;
	strncpy(s->file, file, sizeof(s->file) - 1);
	s->size = ext->alloc_size;
	s->loaded_ms = k_uptime_get();
	s->m = m;
	shell_print(sh, "%s %s: %s %s (%zu bytes)", m->mount, m->cmd.syntax, file,
		    strcmp(m->mount, "pcie") == 0 ? "-> pcie debug" : "-> dbg", s->size);
	ret = 0;
	goto out;
fail:
	(void)llext_teardown(ext);
	llext_unload(&ext);
out:
	k_mutex_unlock(&mod_lock);
	return ret;
}

int lab_mod_unload(const struct shell *sh, const char *name)
{
	int n = 0;

	k_mutex_lock(&mod_lock, K_FOREVER);
	for (int i = 0; i < ARRAY_SIZE(mods); i++) {
		if (mods[i].m != NULL &&
		    (strcmp(name, "all") == 0 || strcmp(mods[i].m->cmd.syntax, name) == 0)) {
			drop(&mods[i]);
			n++;
		}
	}
	k_mutex_unlock(&mod_lock);
	if (n == 0) {
		shell_error(sh, "%s: not loaded", name);
		return -ENOENT;
	}
	return 0;
}

bool lab_mod_get(const char *mount, size_t idx, struct shell_static_entry *entry)
{
	for (int i = 0; i < ARRAY_SIZE(mods); i++) {
		const struct lab_mod *m = mods[i].m;

		if (m != NULL && strcmp(m->mount, mount) == 0 && idx-- == 0) {
			*entry = m->cmd;
			return true;
		}
	}
	return false;
}

void lab_mod_list(const struct shell *sh)
{
	int64_t now = k_uptime_get();

	shell_print(sh, "%-12s %-6s %-24s %8s %8s", "command", "mount", "file", "bytes", "age s");
	for (int i = 0; i < ARRAY_SIZE(mods); i++) {
		const struct lab_mod *m = mods[i].m;

		if (m != NULL) {
			shell_print(sh, "%-12s %-6s %-24s %8zu %8lld", m->cmd.syntax, m->mount,
				    mods[i].file, mods[i].size, (now - mods[i].loaded_ms) / 1000);
		}
	}
}

/* dbg <module> ...: mount point for modules that are not PCIe specific */
static void dbg_get(size_t idx, struct shell_static_entry *entry)
{
	if (!lab_mod_get("dbg", idx, entry)) {
		entry->syntax = NULL;
	}
}

SHELL_DYNAMIC_CMD_CREATE(dbg_mods, dbg_get);
SHELL_CMD_REGISTER(dbg, &dbg_mods, "Debug modules (lab mod load <slot|/path>)", NULL);

int lab_image_check(const struct shell *sh, struct llext *ext, const char *file, bool force)
{
	const uint32_t *img = llext_find_sym(&ext->exp_tab, LAB_IMAGE_SYMBOL);

	if (img == NULL) {
		if (sh != NULL) {
			shell_warn(sh, "%s: no image tag, cannot check it fits image %d", file,
				   CONFIG_LAB_IMAGE);
		}
		return 0;
	}
	if (*img == CONFIG_LAB_IMAGE) {
		return 0;
	}
	if (sh != NULL) {
		shell_fprintf(sh, force ? SHELL_WARNING : SHELL_ERROR,
			      "%s: built for image %u, this firmware is image %d%s\n", file, *img,
			      CONFIG_LAB_IMAGE, force ? " (forced)" : "; rebuild it, or -f");
	}
	return force ? 0 : -ENOEXEC;
}
