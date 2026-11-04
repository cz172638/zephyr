/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Endpoint controller stand-in: host memory is an array. An outbound
 * mapping copies the host range into the window when it is made; the
 * bytes the endpoint changed go back to the host on fake_ob_flush() and
 * when the mapping is torn down. BARs, MSI and MSI-X are recorded.
 */

#define DT_DRV_COMPAT zephyr_pcie_epc_fake

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>

#include "fake_epc.h"

#define WIN_SIZE (1024 * 1024)

uint8_t fake_host_mem[HOST_SIZE] __aligned(4096);
struct pcie_epc_header fake_hdr;
struct pcie_epc_bar fake_bar[PCIE_EPC_NUM_BARS];
uint8_t fake_bars_cleared;
uint16_t fake_msix_nr;
uint8_t fake_msix_bir;
uint32_t fake_msix_off;
uint8_t fake_msi_nr;
uint32_t fake_irq_msix[33];
uint32_t fake_irq_msi[33];
uint32_t fake_maps;
bool fake_bme = true;
K_SEM_DEFINE(fake_irq_sem, 0, 1000);

static uint8_t window[WIN_SIZE] __aligned(WIN_SIZE);
static uint8_t shadow[WIN_SIZE];

static struct {
	uintptr_t cpu;
	uint64_t pci;
	size_t size;
} map[16];

struct fake_data {
	struct pcie_epc_data common; /* must be first */
};

static const struct pcie_epc_features feat = {
	.msi_capable = true,
	.msix_capable = true,
	.align = 256,
	.max_functions = 1,
	.bar_fixed_64bit = BIT(0) | BIT(2) | BIT(4),
	.reserved_bar = BIT(1) | BIT(3) | BIT(5),
};

static int fake_start(const struct device *dev)
{
	return 0;
}

static int fake_write_header(const struct device *dev, uint8_t func,
			     const struct pcie_epc_header *hdr)
{
	fake_hdr = *hdr;
	return 0;
}

static int fake_set_bar(const struct device *dev, uint8_t func, const struct pcie_epc_bar *bar)
{
	if ((bar->phys & (bar->size - 1)) != 0) {
		return -EINVAL;
	}
	fake_bar[bar->barno] = *bar;
	return 0;
}

static void fake_clear_bar(const struct device *dev, uint8_t func, uint8_t barno)
{
	memset(&fake_bar[barno], 0, sizeof(fake_bar[barno]));
	fake_bars_cleared |= BIT(barno);
}

/* host range of an outbound mapping, clipped to the simulated memory */
static size_t clip(uint64_t pci, size_t size)
{
	if (pci < HOST_BASE || pci >= HOST_BASE + HOST_SIZE) {
		return 0;
	}
	return MIN(size, (size_t)(HOST_BASE + HOST_SIZE - pci));
}

static void flush_one(int i)
{
	uint8_t *w = (uint8_t *)map[i].cpu;
	uint8_t *sh = &shadow[map[i].cpu - (uintptr_t)window];
	uint8_t *h = host_ptr(MAX(map[i].pci, HOST_BASE));

	for (size_t j = 0; j < clip(map[i].pci, map[i].size); j++) {
		if (w[j] != sh[j]) {
			h[j] = w[j];
			sh[j] = w[j];
		}
	}
}

void fake_ob_flush(void)
{
	for (int i = 0; i < ARRAY_SIZE(map); i++) {
		if (map[i].size != 0) {
			flush_one(i);
		}
	}
}

static int fake_map(const struct device *dev, uint8_t func, uintptr_t cpu, uint64_t pci,
		    size_t size)
{
	for (int i = 0; i < ARRAY_SIZE(map); i++) {
		if (map[i].size == 0) {
			map[i].cpu = cpu;
			map[i].pci = pci;
			map[i].size = size;
			memset((void *)cpu, 0xee, size);
			memcpy((void *)cpu, host_ptr(MAX(pci, HOST_BASE)), clip(pci, size));
			memcpy(&shadow[cpu - (uintptr_t)window], (void *)cpu, size);
			fake_maps++;
			return 0;
		}
	}
	return -ENOMEM;
}

static void fake_unmap(const struct device *dev, uint8_t func, uintptr_t cpu)
{
	for (int i = 0; i < ARRAY_SIZE(map); i++) {
		if (map[i].size != 0 && map[i].cpu == cpu) {
			flush_one(i);
			map[i].size = 0;
			fake_maps--;
			return;
		}
	}
}

static int fake_set_msi(const struct device *dev, uint8_t func, uint8_t nr)
{
	fake_msi_nr = nr;
	return 0;
}

static int fake_get_msi(const struct device *dev, uint8_t func)
{
	return fake_msi_nr;
}

static int fake_set_msix(const struct device *dev, uint8_t func, uint16_t nr, uint8_t bir,
			 uint32_t off)
{
	fake_msix_nr = nr;
	fake_msix_bir = bir;
	fake_msix_off = off;
	return 0;
}

static int fake_get_msix(const struct device *dev, uint8_t func)
{
	return fake_msix_nr;
}

static int fake_raise_irq(const struct device *dev, uint8_t func, enum pcie_epc_irq_type type,
			  uint16_t irq_num)
{
	if (!fake_bme) {
		return -EACCES;
	}
	if (irq_num == 0 || irq_num > 32) {
		return -EINVAL;
	}
	if (type == PCIE_EPC_IRQ_MSIX) {
		fake_irq_msix[irq_num]++;
	} else if (type == PCIE_EPC_IRQ_MSI) {
		fake_irq_msi[irq_num]++;
	} else {
		return -EINVAL;
	}
	k_sem_give(&fake_irq_sem);
	return 0;
}

static int fake_cfg_read(const struct device *dev, uint8_t func, uint32_t off, uint32_t *val)
{
	*val = off == 0x04 ? (fake_bme ? BIT(2) | BIT(1) : BIT(1)) : 0;
	return 0;
}

static const struct pcie_epc_features *fake_features(const struct device *dev, uint8_t func)
{
	return &feat;
}

static bool fake_link_up(const struct device *dev)
{
	return true;
}

static DEVICE_API(pcie_epc, fake_api) = {
	.start = fake_start,
	.write_header = fake_write_header,
	.set_bar = fake_set_bar,
	.clear_bar = fake_clear_bar,
	.map_addr = fake_map,
	.unmap_addr = fake_unmap,
	.set_msi = fake_set_msi,
	.get_msi = fake_get_msi,
	.set_msix = fake_set_msix,
	.get_msix = fake_get_msix,
	.raise_irq = fake_raise_irq,
	.cfg_read = fake_cfg_read,
	.get_features = fake_features,
	.link_up = fake_link_up,
};

static int fake_init(const struct device *dev)
{
	struct pcie_epc_ob_window ob = {
		.cpu_base = (uintptr_t)window,
		.axi_base = 0x10000000,
		.size = WIN_SIZE,
		.page_size = 4096,
	};

	return pcie_epc_init_common(dev, &ob, false);
}

static struct fake_data fake_data0;
DEVICE_DT_INST_DEFINE(0, fake_init, NULL, &fake_data0, NULL, POST_KERNEL,
		      CONFIG_PCIE_EPC_INIT_PRIORITY, &fake_api);
