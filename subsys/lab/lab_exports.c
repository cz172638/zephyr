/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * What debug modules may call in the firmware beyond the default groups
 * (syscalls, libc, printk). Adding a symbol here changes the firmware:
 * bump CONFIG_LAB_IMAGE. scripts/lab/llext-symcheck.pl lists what a
 * module still misses.
 */

#include <zephyr/cache.h>
#include <zephyr/irq.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/llext/symbol.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>

/* shell output from command handlers */
EXPORT_SYMBOL(shell_fprintf_impl);
EXPORT_SYMBOL(shell_vfprintf);
EXPORT_SYMBOL(shell_hexdump);
EXPORT_SYMBOL(shell_hexdump_line);
EXPORT_SYMBOL(vprintk);

/* kernel objects a module may own (stopped by its fini) */
EXPORT_SYMBOL(k_timer_init);
EXPORT_SYMBOL(k_work_init);
EXPORT_SYMBOL(k_work_submit);
EXPORT_SYMBOL(k_work_cancel_sync);
EXPORT_SYMBOL(k_aligned_alloc);
EXPORT_SYMBOL(k_malloc);
EXPORT_SYMBOL(k_calloc);
EXPORT_SYMBOL(k_free);
EXPORT_SYMBOL(k_mem_map_phys_bare);
EXPORT_SYMBOL(k_mem_unmap_phys_bare);

/* interrupts */
#ifdef CONFIG_DYNAMIC_INTERRUPTS
EXPORT_SYMBOL(arch_irq_connect_dynamic);
/* no arch_irq_disconnect_dynamic on arm64: a module hands its line back to this */
EXPORT_SYMBOL(z_irq_spurious);
#endif
EXPORT_SYMBOL(arch_irq_enable);
EXPORT_SYMBOL(arch_irq_disable);
EXPORT_SYMBOL(arch_irq_is_enabled);

/* caches */
#ifdef CONFIG_CACHE_MANAGEMENT
EXPORT_SYMBOL(arch_dcache_flush_range);
EXPORT_SYMBOL(arch_dcache_invd_range);
EXPORT_SYMBOL(arch_dcache_flush_and_invd_range);
#endif

/* libc beyond the default group */
#include <stdlib.h>
#include <string.h>
#include <strings.h>
EXPORT_SYMBOL(strtoull);
EXPORT_SYMBOL(strtok_r);
EXPORT_SYMBOL(strstr);
EXPORT_SYMBOL(strcasecmp);

/* k_cyc_to_*() with a run-time timer frequency */
#ifdef CONFIG_TIMER_READS_ITS_FREQUENCY_AT_RUNTIME
extern unsigned int z_clock_hw_cycles_per_sec;
EXPORT_SYMBOL(z_clock_hw_cycles_per_sec);
#endif

EXPORT_SYMBOL(k_work_init_delayable);
EXPORT_SYMBOL(k_work_reschedule);
EXPORT_SYMBOL(k_work_cancel_delayable_sync);
