/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Endpoint controller stand-in: host memory is an array. An outbound
 * mapping copies the host range into the window when it is made; when it
 * is torn down, the bytes the endpoint changed go back to the host. The
 * test (the host) may write its own memory meanwhile.
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
struct pcie_epc_bar fake_bar0;
uint32_t fake_irq_count[32];
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
	.bar_fixed_64bit = BIT(0),
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
	if (bar->barno == 0) {
		fake_bar0 = *bar;
	}
	return 0;
}

static void fake_clear_bar(const struct device *dev, uint8_t func, uint8_t barno)
{
}

/* host range of an outbound mapping, clipped to the simulated memory */
static size_t clip(uint64_t pci, size_t size)
{
	if (pci < HOST_BASE || pci >= HOST_BASE + HOST_SIZE) {
		return 0;
	}
	return MIN(size, (size_t)(HOST_BASE + HOST_SIZE - pci));
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
			uint8_t *w = (uint8_t *)cpu;
			uint8_t *sh = &shadow[cpu - (uintptr_t)window];
			uint8_t *h = host_ptr(MAX(map[i].pci, HOST_BASE));

			for (size_t j = 0; j < clip(map[i].pci, map[i].size); j++) {
				if (w[j] != sh[j]) {
					h[j] = w[j];
				}
			}
			map[i].size = 0;
			fake_maps--;
			return;
		}
	}
}

static int fake_set_msi(const struct device *dev, uint8_t func, uint8_t nr)
{
	return 0;
}

static int fake_get_msi(const struct device *dev, uint8_t func)
{
	return -EINVAL;
}

static int fake_set_msix(const struct device *dev, uint8_t func, uint16_t nr, uint8_t bir,
			 uint32_t off)
{
	return 0;
}

static int fake_get_msix(const struct device *dev, uint8_t func)
{
	return 8;
}

static int fake_raise_irq(const struct device *dev, uint8_t func, enum pcie_epc_irq_type type,
			  uint16_t irq_num)
{
	if (type != PCIE_EPC_IRQ_MSIX || irq_num == 0 || irq_num > ARRAY_SIZE(fake_irq_count)) {
		return -EINVAL;
	}
	fake_irq_count[irq_num - 1]++;
	k_sem_give(&fake_irq_sem);
	return 0;
}

static int fake_cfg_read(const struct device *dev, uint8_t func, uint32_t off, uint32_t *val)
{
	switch (off) {
	case 0x04:
		*val = fake_bme ? BIT(2) | BIT(1) : BIT(1);
		break;
	case 0x08:
		*val = ((uint32_t)fake_hdr.baseclass << 24) | ((uint32_t)fake_hdr.subclass << 16) |
		       ((uint32_t)fake_hdr.prog_if << 8);
		break;
	case 0x100:
		/* AER, next 0x150 */
		*val = 0x15010001;
		break;
	case 0x150:
		/* DSN, last */
		*val = 0x00010003;
		break;
	case 0x154:
		*val = (uint32_t)FAKE_DSN;
		break;
	case 0x158:
		*val = (uint32_t)(FAKE_DSN >> 32);
		break;
	default:
		*val = 0;
		break;
	}
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
