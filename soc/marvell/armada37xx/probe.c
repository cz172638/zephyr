/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Reads of addresses that may not decode (bring-up shells): an external
 * abort on the probing load is turned into -EIO instead of a fatal error.
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/arm64/exception.h>

#include <soc.h>

extern char a3700_probe_insn[], a3700_probe_fixup[];
extern char a3700_probew_start[], a3700_probew_end[], a3700_probew_fixup[];

__asm__(".text\n"
	".global a3700_probe_read32\n"
	".type a3700_probe_read32, %function\n"
	"a3700_probe_read32:\n"
	".global a3700_probe_insn\n"
	"a3700_probe_insn:\n"
	"	ldr w2, [x0]\n"
	"	str w2, [x1]\n"
	"	mov w0, #0\n"
	"	ret\n"
	".global a3700_probe_fixup\n"
	"a3700_probe_fixup:\n"
	"	mov w0, #-5\n"
	"	ret\n"
	/*
	 * Write: a bus error on a store comes back as an asynchronous
	 * SError; the barriers make it land inside this window.
	 */
	".global a3700_probe_write32\n"
	".type a3700_probe_write32, %function\n"
	"a3700_probe_write32:\n"
	".global a3700_probew_start\n"
	"a3700_probew_start:\n"
	"	str w1, [x0]\n"
	"	dsb sy\n"
	"	ldr w2, [x0]\n"
	"	dsb sy\n"
	"	isb\n"
	"	nop\n"
	"	nop\n"
	".global a3700_probew_end\n"
	"a3700_probew_end:\n"
	"	mov w0, #0\n"
	"	ret\n"
	".global a3700_probew_fixup\n"
	"a3700_probew_fixup:\n"
	"	mov w0, #-5\n"
	"	ret\n");

volatile uint32_t a3700_probe_faults;

bool z_arm64_fault_fixup(struct arch_esf *esf, uint64_t esr, uint64_t far)
{
	ARG_UNUSED(esr);
	ARG_UNUSED(far);

	if (esf->elr == (uint64_t)(uintptr_t)a3700_probe_insn) {
		esf->elr = (uint64_t)(uintptr_t)a3700_probe_fixup;
		a3700_probe_faults++;
		return true;
	}
	if (esf->elr >= (uint64_t)(uintptr_t)a3700_probew_start &&
	    esf->elr <= (uint64_t)(uintptr_t)a3700_probew_end) {
		esf->elr = (uint64_t)(uintptr_t)a3700_probew_fixup;
		a3700_probe_faults++;
		return true;
	}
	return false;
}
