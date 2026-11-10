/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Lab agent: run test extensions (LLEXT) from memory. An extension
 * exports lab_ext_main(api, argc, argv) and gets everything beyond
 * printk through the api table.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/lab/lab.h>
#include <zephyr/llext/buf_loader.h>
#include <zephyr/llext/llext.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "lab_internal.h"

LOG_MODULE_DECLARE(lab, CONFIG_LAB_LOG_LEVEL);

static void api_busy_wait(uint32_t us)
{
	k_busy_wait(us);
}

static void api_sleep(uint32_t ms)
{
	k_msleep(ms);
}

static int api_peer_exec(const char *cmd, char *out, size_t len, uint32_t timeout_ms)
{
	return lab_peer_exec(cmd, out, len, K_MSEC(timeout_ms));
}

static void *api_map_io(uintptr_t phys, size_t size)
{
	uint8_t *virt = NULL;

	k_mem_map_phys_bare(&virt, phys, size, K_MEM_PERM_RW | K_MEM_CACHE_NONE);
	return virt;
}

static void api_flush(void *addr, size_t size)
{
	(void)sys_cache_data_flush_range(addr, size);
}

static void api_invd(void *addr, size_t size)
{
	(void)sys_cache_data_invd_range(addr, size);
}

static void *api_alloc(size_t size, size_t align)
{
	return k_aligned_alloc(MAX(align, sizeof(void *)), size);
}

static const struct lab_ext_api api = {
	.version = LAB_EXT_API_VERSION,
	.size = sizeof(struct lab_ext_api),
	.printf = printk,
	.now_ns = lab_now_ns,
	.busy_wait_us = api_busy_wait,
	.sleep_ms = api_sleep,
	.local_exec = lab_local_exec,
	.peer_exec = api_peer_exec,
	.peer_offset_ns = lab_peer_offset_ns,
	.role = lab_role,
	.map_io = api_map_io,
	.cache_flush = api_flush,
	.cache_invd = api_invd,
	.alloc = api_alloc,
	.free = k_free,
};

int lab_ext_run(const char *name, const void *elf, size_t len, const char *fn, int argc,
		char **argv)
{
	struct llext_buf_loader buf_loader = LLEXT_TEMPORARY_BUF_LOADER(elf, len);
	struct llext_load_param ldr_parm = LLEXT_LOAD_PARAM_DEFAULT;
	struct llext *ext = NULL;
	char lname[LLEXT_MAX_NAME_LEN + 1];
	lab_ext_main_t entry;
	int64_t t0;
	int ret;

	strncpy(lname, name, sizeof(lname) - 1);
	lname[sizeof(lname) - 1] = '\0';
	t0 = k_uptime_get();
	ret = llext_load(&buf_loader.loader, lname, &ext, &ldr_parm);
	if (ret != 0) {
		LOG_ERR("llext %s: load %d", lname, ret);
		return ret;
	}
	LOG_INF("llext %s loaded in %lld ms", lname, k_uptime_get() - t0);
	ret = llext_bringup(ext);
	if (ret != 0) {
		goto out;
	}
	entry = (lab_ext_main_t)llext_find_sym(&ext->exp_tab, fn ? fn : LAB_EXT_MAIN);
	if (entry != NULL) {
		ret = entry(&api, argc, argv);
	} else if (fn != NULL) {
		ret = -ENOENT;
	} else {
		/* a plain hello: void test_entry(void) */
		ret = llext_call_fn(ext, "test_entry");
	}
	(void)llext_teardown(ext);
out:
	llext_unload(&ext);
	return ret;
}
