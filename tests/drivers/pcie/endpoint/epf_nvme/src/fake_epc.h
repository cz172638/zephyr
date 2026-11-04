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
#define HOST_BASE 0x100000000ULL
#define HOST_SIZE (256 * 1024)

extern uint8_t fake_host_mem[HOST_SIZE];
extern struct pcie_epc_header fake_hdr;
extern struct pcie_epc_bar fake_bar0;
extern uint32_t fake_irq_count[32];
extern uint32_t fake_maps;
extern bool fake_bme;
extern struct k_sem fake_irq_sem;

#define FAKE_DSN 0x0123456789abcdefULL

static inline void *host_ptr(uint64_t pci)
{
	return &fake_host_mem[pci - HOST_BASE];
}

#endif
