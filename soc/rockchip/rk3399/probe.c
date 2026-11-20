/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Reads that may end in a bus error: the PCIe root complex answers a
 * configuration read that completes with Unsupported Request (no device
 * at that address, e.g. behind a switch) with an AXI slave error, which
 * the CPU takes as a synchronous external abort on the load. Such an
 * abort in rk3399_probe_read32() returns -EIO instead of a fatal error.
 */

#include <errno.h>

#include <zephyr/arch/arm64/exception.h>
#include <zephyr/kernel.h>

#include <soc.h>

extern char rk3399_probe_insn[], rk3399_probe_fixup[];

__asm__(".text\n"
	".global rk3399_probe_read32\n"
	".type rk3399_probe_read32, %function\n"
	"rk3399_probe_read32:\n"
	".global rk3399_probe_insn\n"
	"rk3399_probe_insn:\n"
	"	ldr w2, [x0]\n"
	"	str w2, [x1]\n"
	"	mov w0, #0\n"
	"	ret\n"
	".global rk3399_probe_fixup\n"
	"rk3399_probe_fixup:\n"
	"	mov w0, #-" STRINGIFY(EIO) "\n"
	"	ret\n");

volatile uint32_t rk3399_probe_faults;

bool z_arm64_fault_fixup(struct arch_esf *esf, uint64_t esr, uint64_t far)
{
	ARG_UNUSED(esr);
	ARG_UNUSED(far);

	if (esf->elr == (uint64_t)(uintptr_t)rk3399_probe_insn) {
		esf->elr = (uint64_t)(uintptr_t)rk3399_probe_fixup;
		rk3399_probe_faults++;
		return true;
	}
	return false;
}
