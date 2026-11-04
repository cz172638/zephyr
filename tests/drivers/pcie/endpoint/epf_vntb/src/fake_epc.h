/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FAKE_EPC_H_
#define FAKE_EPC_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>

/* Host memory as the endpoint sees it: PCI addresses HOST_BASE.. */
#define HOST_BASE 0x200000000ULL
#define HOST_SIZE (128 * 1024)

extern uint8_t fake_host_mem[HOST_SIZE];
extern struct pcie_epc_header fake_hdr;
extern struct pcie_epc_bar fake_bar[PCIE_EPC_NUM_BARS];
extern uint8_t fake_bars_cleared;
extern uint16_t fake_msix_nr;
extern uint8_t fake_msix_bir;
extern uint32_t fake_msix_off;
extern uint8_t fake_msi_nr;
extern uint32_t fake_irq_msix[33];
extern uint32_t fake_irq_msi[33];
extern uint32_t fake_maps;
extern bool fake_bme;
extern struct k_sem fake_irq_sem;

/* copy what the endpoint wrote through live outbound mappings to the host */
void fake_ob_flush(void);

static inline void *host_ptr(uint64_t pci)
{
	return &fake_host_mem[pci - HOST_BASE];
}

#endif
