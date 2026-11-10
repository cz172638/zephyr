/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Firmware functions the ESPRESSObin debug modules (dbg/, the Aardvark
 * irqtest and xortest) call. A change here is a firmware change: bump
 * CONFIG_LAB_IMAGE.
 */

#include <zephyr/drivers/pcie/aardvark.h>
#include <zephyr/lab/lab.h>
#include <zephyr/llext/symbol.h>

#if defined(CONFIG_GIC)
#include <zephyr/drivers/interrupt_controller/gic.h>
#endif

#include <soc.h>

#include "pcie_epc_aardvark_a3700_xor.h"

EXPORT_SYMBOL(a3700_probe_read32);
EXPORT_SYMBOL(advk_mode_get);

EXPORT_SYMBOL(a3700_xor_memcpy);
EXPORT_SYMBOL(a3700_xor_memcpy_sg);
EXPORT_SYMBOL(a3700_xor_xor);
EXPORT_SYMBOL(a3700_xor_get_stats);
EXPORT_SYMBOL(a3700_xor_pcie_window);

EXPORT_SYMBOL(lab_local_exec);
EXPORT_SYMBOL(lab_peer_exec);
EXPORT_SYMBOL(lab_now_ns);
EXPORT_SYMBOL(lab_peer_offset_ns);

#if defined(CONFIG_GIC)
EXPORT_SYMBOL(arm_gic_irq_set_pending);
EXPORT_SYMBOL(arm_gic_irq_is_pending);
EXPORT_SYMBOL(arm_gic_irq_clear_pending);
#endif
