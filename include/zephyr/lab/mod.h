/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Debug commands that build either into the firmware or as LLEXT
 *        modules loaded at run time ("lab mod load")
 *
 * A debug command file ends with LAB_DBG_CMD() instead of
 * SHELL_SUBCMD_ADD(). Built into the firmware it is a plain subcommand
 * of @p _parent; built with add_llext_target() (LL_EXTENSION_BUILD) it
 * becomes a module descriptor, and once loaded the command shows up
 * under its mount point:
 *
 *   mount "pcie"  ->  pcie debug <name> ...
 *   mount "dbg"   ->  dbg <name> ...
 *
 * The module may keep timers, interrupts or hardware state; @p _fini
 * must undo all of it, it runs before the module memory is freed.
 */

#ifndef ZEPHYR_INCLUDE_LAB_MOD_H_
#define ZEPHYR_INCLUDE_LAB_MOD_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/llext/symbol.h>
#include <zephyr/shell/shell.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONFIG_LAB_IMAGE
#define CONFIG_LAB_IMAGE 0
#endif

#define LAB_MOD_VERSION 1
#define LAB_MOD_SYMBOL  "lab_mod"
#define LAB_IMAGE_SYMBOL "lab_image"

struct lab_mod {
	uint32_t version;
	/* CONFIG_LAB_IMAGE the module was built for */
	uint32_t image;
	const char *mount;
	struct shell_static_entry cmd;
	/* after loading, 0 or the load fails (and the module is freed) */
	int (*init)(void);
	/* before unloading */
	void (*fini)(void);
};

#ifdef LL_EXTENSION_BUILD
/* Tag a module with the firmware image it is built for (lab ext modules) */
#define LAB_IMAGE_TAG()                                                                        \
	const uint32_t lab_image = CONFIG_LAB_IMAGE;                                           \
	EXPORT_SYMBOL(lab_image)

#define LAB_DBG_CMD(_parent, _mount, _syntax, _subcmd, _help, _handler, _mand, _opt, _init,  \
		    _fini)                                                                     \
	const struct lab_mod lab_mod = {                                                       \
		.version = LAB_MOD_VERSION,                                                    \
		.image = CONFIG_LAB_IMAGE,                                                     \
		.mount = _mount,                                                               \
		.cmd = SHELL_CMD_ARG(_syntax, _subcmd, _help, _handler, _mand, _opt),          \
		.init = _init,                                                                 \
		.fini = _fini,                                                                 \
	};                                                                                     \
	EXPORT_SYMBOL(lab_mod);                                                                \
	LAB_IMAGE_TAG()
#else
#define LAB_IMAGE_TAG()
#define LAB_DBG_CMD(_parent, _mount, _syntax, _subcmd, _help, _handler, _mand, _opt, _init,  \
		    _fini)                                                                     \
	static int (*const _syntax##_lab_init)(void) __unused = _init;                         \
	static void (*const _syntax##_lab_fini)(void) __unused = _fini;                        \
	SHELL_SUBCMD_ADD(_parent, _syntax, _subcmd, _help, _handler, _mand, _opt)
#endif

/** Load (or reload, by command name) a module from memory */
int lab_mod_load(const struct shell *sh, const void *elf, size_t len, const char *file,
		 bool force);

/**
 * Is a module built for this firmware? Checks the lab_image tag; logs to
 * @p sh. Untagged: allowed with a warning. Returns 0 or -ENOEXEC.
 */
struct llext;
int lab_image_check(const struct shell *sh, struct llext *ext, const char *file, bool force);

/** Unload by command name; "all" for all */
int lab_mod_unload(const struct shell *sh, const char *name);

/**
 * The idx-th loaded module mounted at @p mount, as a shell entry (for
 * dynamic subcommand sets). Returns false past the last one.
 */
bool lab_mod_get(const char *mount, size_t idx, struct shell_static_entry *entry);

#ifdef __cplusplus
}
#endif

#endif
