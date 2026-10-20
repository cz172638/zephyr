/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * PCIe endpoint test function.
 *
 * Implements the device side of the register protocol used by the Linux
 * pci_endpoint_test host driver, so a stock Linux host (modprobe
 * pci_endpoint_test; pcitest / kselftest pci_endpoint) can exercise BARs,
 * INTx/MSI/MSI-X and EP-initiated reads, writes and copies.
 *
 * Only pcie_epc.h is used: the function knows nothing about the controller.
 */

#define DT_DRV_COMPAT zephyr_pcie_epf_test

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epf.h>

LOG_MODULE_REGISTER(pcie_epf_test, CONFIG_PCIE_EP_LOG_LEVEL);

/* Register block at offset 0 of the test register BAR (host driver ABI) */
#define REG_MAGIC      0x00
#define REG_COMMAND    0x04
#define REG_STATUS     0x08
#define REG_SRC_LO     0x0c
#define REG_SRC_HI     0x10
#define REG_DST_LO     0x14
#define REG_DST_HI     0x18
#define REG_SIZE       0x1c
#define REG_CHECKSUM   0x20
#define REG_IRQ_TYPE   0x24
#define REG_IRQ_NUMBER 0x28
#define REG_FLAGS      0x2c
#define REG_CAPS       0x30
#define REG_DB_BAR     0x34
#define REG_DB_OFFSET  0x38
#define REG_DB_DATA    0x3c
#define REG_BLOCK_SIZE 0x40

#define CMD_RAISE_INTX BIT(0)
#define CMD_RAISE_MSI  BIT(1)
#define CMD_RAISE_MSIX BIT(2)
#define CMD_READ       BIT(3)
#define CMD_WRITE      BIT(4)
#define CMD_COPY       BIT(5)

#define STS_READ_SUCCESS     BIT(0)
#define STS_READ_FAIL        BIT(1)
#define STS_WRITE_SUCCESS    BIT(2)
#define STS_WRITE_FAIL       BIT(3)
#define STS_COPY_SUCCESS     BIT(4)
#define STS_COPY_FAIL        BIT(5)
#define STS_IRQ_RAISED       BIT(6)
#define STS_SRC_ADDR_INVALID BIT(7)
#define STS_DST_ADDR_INVALID BIT(8)
#define STS_NO_RESOURCE      BIT(18)

#define CAP_UNALIGNED_ACCESS BIT(0)
#define CAP_MSI              BIT(1)
#define CAP_MSIX             BIT(2)
#define CAP_INTX             BIT(3)
#define CAP_BAR_RESERVED(n)  BIT(6 + (n))

#define IRQ_TYPE_INTX 0
#define IRQ_TYPE_MSI  1
#define IRQ_TYPE_MSIX 2

struct epf_test_config {
	const struct device *epc;
	uint8_t func;
	struct pcie_epc_header hdr;
	uintptr_t mem_base;
	size_t mem_size;
	uint32_t bar_size[PCIE_EPC_NUM_BARS];
	uint8_t reg_bar;
	uint8_t msi_irqs;
	uint16_t msix_irqs;
};

struct epf_test_data {
	const struct device *dev;
	struct pcie_epc_listener listener;
	struct k_work_delayable cmd_work;
	struct pcie_epc_bar bar[PCIE_EPC_NUM_BARS];
	volatile uint8_t *regs;
	/* CPU view of the memory-region (mapped on MMU targets) */
	uintptr_t mem_virt;
	bool bme;
	bool link;
	uint8_t chunk[CONFIG_PCIE_EPF_TEST_CHUNK] __aligned(8);
};

static inline uint32_t reg_rd(struct epf_test_data *d, uint32_t off)
{
	return sys_read32((mem_addr_t)(d->regs + off));
}

static inline void reg_wr(struct epf_test_data *d, uint32_t off, uint32_t v)
{
	sys_write32(v, (mem_addr_t)(d->regs + off));
}

static inline void sts_set(struct epf_test_data *d, uint32_t bits)
{
	reg_wr(d, REG_STATUS, reg_rd(d, REG_STATUS) | bits);
}

/* Test pattern only, no need for an entropy source on the EP core. */
static void fill_pattern(uint8_t *buf, size_t len)
{
	static uint32_t state;

	if (state == 0) {
		state = k_cycle_get_32() | 1U;
	}
	for (size_t i = 0; i < len; i++) {
		state ^= state << 13;
		state ^= state >> 17;
		state ^= state << 5;
		buf[i] = (uint8_t)state;
	}
}

/* Linux crc32_le(~0, ...) == ~(Zephyr crc32_ieee chained from 0) */
static inline uint32_t crc_linux(uint32_t zephyr_crc)
{
	return ~zephyr_crc;
}

static void raise_irq(const struct device *dev, struct epf_test_data *d)
{
	const struct epf_test_config *cfg = dev->config;
	uint32_t type = reg_rd(d, REG_IRQ_TYPE);
	uint32_t num = reg_rd(d, REG_IRQ_NUMBER);
	int ret;

	sts_set(d, STS_IRQ_RAISED);
	barrier_dmem_fence_full();

	switch (type) {
	case IRQ_TYPE_INTX:
		ret = pcie_epc_raise_irq(cfg->epc, cfg->func, PCIE_EPC_IRQ_INTX, 0);
		break;
	case IRQ_TYPE_MSI:
		ret = pcie_epc_raise_irq(cfg->epc, cfg->func, PCIE_EPC_IRQ_MSI, num);
		break;
	case IRQ_TYPE_MSIX:
		ret = pcie_epc_raise_irq(cfg->epc, cfg->func, PCIE_EPC_IRQ_MSIX, num);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	if (ret != 0) {
		LOG_WRN("raise irq type %u num %u failed (%d)", type, num, ret);
	}
}

/* EP reads host buffer at SRC and checks it against the host's CRC. */
static int cmd_read(const struct device *dev, struct epf_test_data *d)
{
	const struct epf_test_config *cfg = dev->config;
	uint64_t src = ((uint64_t)reg_rd(d, REG_SRC_HI) << 32) | reg_rd(d, REG_SRC_LO);
	uint32_t size = reg_rd(d, REG_SIZE);
	uint32_t crc = 0;
	int ret;

	for (uint32_t off = 0; off < size; off += sizeof(d->chunk)) {
		size_t n = MIN(sizeof(d->chunk), size - off);

		ret = pcie_epc_xfer_memcpy(cfg->epc, cfg->func, src + off, d->chunk, n, false);
		if (ret != 0) {
			sts_set(d, STS_SRC_ADDR_INVALID);
			return ret;
		}
		crc = crc32_ieee_update(crc, d->chunk, n);
	}

	return crc_linux(crc) == reg_rd(d, REG_CHECKSUM) ? 0 : -EIO;
}

/* EP fills host buffer at DST with random data and publishes the CRC. */
static int cmd_write(const struct device *dev, struct epf_test_data *d)
{
	const struct epf_test_config *cfg = dev->config;
	uint64_t dst = ((uint64_t)reg_rd(d, REG_DST_HI) << 32) | reg_rd(d, REG_DST_LO);
	uint32_t size = reg_rd(d, REG_SIZE);
	uint32_t crc = 0;
	int ret;

	for (uint32_t off = 0; off < size; off += sizeof(d->chunk)) {
		size_t n = MIN(sizeof(d->chunk), size - off);

		fill_pattern(d->chunk, n);
		crc = crc32_ieee_update(crc, d->chunk, n);
		ret = pcie_epc_xfer_memcpy(cfg->epc, cfg->func, dst + off, d->chunk, n, true);
		if (ret != 0) {
			sts_set(d, STS_DST_ADDR_INVALID);
			return ret;
		}
	}

	reg_wr(d, REG_CHECKSUM, crc_linux(crc));
	return 0;
}

/* EP copies host SRC to host DST; the host compares the CRCs itself. */
static int cmd_copy(const struct device *dev, struct epf_test_data *d)
{
	const struct epf_test_config *cfg = dev->config;
	uint64_t src = ((uint64_t)reg_rd(d, REG_SRC_HI) << 32) | reg_rd(d, REG_SRC_LO);
	uint64_t dst = ((uint64_t)reg_rd(d, REG_DST_HI) << 32) | reg_rd(d, REG_DST_LO);
	uint32_t size = reg_rd(d, REG_SIZE);
	int ret;

	for (uint32_t off = 0; off < size; off += sizeof(d->chunk)) {
		size_t n = MIN(sizeof(d->chunk), size - off);

		ret = pcie_epc_xfer_memcpy(cfg->epc, cfg->func, src + off, d->chunk, n, false);
		if (ret != 0) {
			sts_set(d, STS_SRC_ADDR_INVALID);
			return ret;
		}
		ret = pcie_epc_xfer_memcpy(cfg->epc, cfg->func, dst + off, d->chunk, n, true);
		if (ret != 0) {
			sts_set(d, STS_DST_ADDR_INVALID);
			return ret;
		}
	}
	return 0;
}

static void cmd_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct epf_test_data *d = CONTAINER_OF(dwork, struct epf_test_data, cmd_work);
	const struct device *dev = d->dev;
	const struct epf_test_config *cfg = dev->config;
	uint32_t cmd = reg_rd(d, REG_COMMAND);
	int ret;

	if (cmd == 0) {
		goto resched;
	}

	reg_wr(d, REG_COMMAND, 0);
	reg_wr(d, REG_STATUS, 0);

	if (cmd & (CMD_READ | CMD_WRITE | CMD_COPY)) {
		if (!d->bme) {
			LOG_WRN("command 0x%x before bus mastering enabled", cmd);
		}
		if (cmd & CMD_READ) {
			ret = cmd_read(dev, d);
			sts_set(d, ret == 0 ? STS_READ_SUCCESS : STS_READ_FAIL);
		} else if (cmd & CMD_WRITE) {
			ret = cmd_write(dev, d);
			sts_set(d, ret == 0 ? STS_WRITE_SUCCESS : STS_WRITE_FAIL);
		} else {
			ret = cmd_copy(dev, d);
			sts_set(d, ret == 0 ? STS_COPY_SUCCESS : STS_COPY_FAIL);
		}
		raise_irq(dev, d);
	} else if (cmd & CMD_RAISE_INTX) {
		reg_wr(d, REG_IRQ_TYPE, IRQ_TYPE_INTX);
		raise_irq(dev, d);
	} else if (cmd & (CMD_RAISE_MSI | CMD_RAISE_MSIX)) {
		bool msix = cmd & CMD_RAISE_MSIX;
		int count = msix ? pcie_epc_get_msix(cfg->epc, cfg->func)
				 : pcie_epc_get_msi(cfg->epc, cfg->func);

		if (count < 0 || reg_rd(d, REG_IRQ_NUMBER) > (uint32_t)count) {
			LOG_WRN("irq %u not enabled by host (%d)", reg_rd(d, REG_IRQ_NUMBER),
				count);
			goto resched;
		}
		reg_wr(d, REG_IRQ_TYPE, msix ? IRQ_TYPE_MSIX : IRQ_TYPE_MSI);
		raise_irq(dev, d);
	} else {
		LOG_DBG("unsupported command 0x%x", cmd);
		reg_wr(d, REG_STATUS, STS_NO_RESOURCE);
	}

resched:
	k_work_reschedule(dwork, K_USEC(CONFIG_PCIE_EPF_TEST_POLL_US));
}

static void epf_test_event(const struct device *epc, struct pcie_epc_listener *l,
			   enum pcie_epc_event evt, uint8_t func)
{
	struct epf_test_data *d = CONTAINER_OF(l, struct epf_test_data, listener);

	ARG_UNUSED(epc);
	ARG_UNUSED(func);

	switch (evt) {
	case PCIE_EPC_EVT_LINK_UP:
		LOG_INF("link up");
		d->link = true;
		break;
	case PCIE_EPC_EVT_LINK_DOWN:
		LOG_INF("link down");
		d->link = false;
		d->bme = false;
		break;
	case PCIE_EPC_EVT_BME:
		LOG_INF("bus mastering enabled");
		d->bme = true;
		break;
	case PCIE_EPC_EVT_BME_CLEARED:
		d->bme = false;
		break;
	default:
		break;
	}
}

static int epf_test_setup_bars(const struct device *dev)
{
	const struct epf_test_config *cfg = dev->config;
	struct epf_test_data *d = dev->data;
	const struct pcie_epc_features *feat = pcie_epc_get_features(cfg->epc, cfg->func);
	uintptr_t next = cfg->mem_base;
	uint32_t caps = 0;
	int ret;

	for (uint8_t b = 0; b < PCIE_EPC_NUM_BARS; b++) {
		size_t want = cfg->bar_size[b];
		size_t sz;
		uintptr_t base;

		if (b == cfg->reg_bar) {
			want = MAX(want, REG_BLOCK_SIZE);
			if (cfg->msix_irqs != 0) {
				/* table + PBA after the register block */
				want = MAX(want, REG_BLOCK_SIZE + cfg->msix_irqs * 16U +
							 DIV_ROUND_UP(cfg->msix_irqs, 64) * 8U);
			}
		}

		sz = (want != 0) ? pcie_epf_bar_size(feat, b, want) : 0;
		if (sz == 0) {
			if (feat != NULL && (feat->reserved_bar & BIT(b))) {
				caps |= CAP_BAR_RESERVED(b);
			}
			continue;
		}

		base = ROUND_UP(next, MAX(sz, feat != NULL ? feat->align : 256));
		if (base + sz > cfg->mem_base + cfg->mem_size) {
			LOG_ERR("memory-region too small for BAR%u (0x%zx)", b, sz);
			return -ENOMEM;
		}
		next = base + sz;

		memset((void *)(d->mem_virt + (base - cfg->mem_base)), 0, sz);
		d->bar[b] = (struct pcie_epc_bar){
			.barno = b,
			.flags = PCIE_EPC_BAR_MEM32,
			.phys = pcie_epc_cpu_to_axi(cfg->epc, base),
			.size = sz,
			.virt = (void *)(d->mem_virt + (base - cfg->mem_base)),
		};
		ret = pcie_epc_set_bar(cfg->epc, cfg->func, &d->bar[b]);
		if (ret != 0) {
			LOG_ERR("set_bar %u failed (%d)", b, ret);
			return ret;
		}
		LOG_DBG("BAR%u: 0x%zx bytes @ 0x%lx", b, sz, (unsigned long)base);
	}

	if (d->bar[cfg->reg_bar].virt == NULL) {
		LOG_ERR("test register BAR%u not available", cfg->reg_bar);
		return -EINVAL;
	}
	d->regs = d->bar[cfg->reg_bar].virt;

	if (feat == NULL || feat->align <= 256) {
		caps |= CAP_UNALIGNED_ACCESS;
	}
	if (feat != NULL && feat->intx_capable) {
		caps |= CAP_INTX;
	}
	if (cfg->msi_irqs != 0 && pcie_epc_set_msi(cfg->epc, cfg->func, cfg->msi_irqs) == 0) {
		caps |= CAP_MSI;
	}
	if (cfg->msix_irqs != 0 && pcie_epc_set_msix(cfg->epc, cfg->func, cfg->msix_irqs,
						     cfg->reg_bar, REG_BLOCK_SIZE) == 0) {
		caps |= CAP_MSIX;
	}
	reg_wr(d, REG_CAPS, caps);

	return 0;
}

static int epf_test_init(const struct device *dev)
{
	const struct epf_test_config *cfg = dev->config;
	struct epf_test_data *d = dev->data;
	int ret;

	d->dev = dev;

	if (!device_is_ready(cfg->epc)) {
		LOG_ERR("controller %s not ready", cfg->epc->name);
		return -ENODEV;
	}

	ret = pcie_epc_claim_function(cfg->epc, cfg->func);
	if (ret != 0) {
		LOG_ERR("function %u: claim failed (%d)", cfg->func, ret);
		return ret;
	}

	ret = pcie_epc_write_header(cfg->epc, cfg->func, &cfg->hdr);
	if (ret != 0) {
		return ret;
	}

#ifdef CONFIG_MMU
	/*
	 * The carveout is outside the kernel image, nothing maps it. Normal
	 * non-cacheable where the architecture has it: memset/memcpy use
	 * unaligned and DC ZVA accesses, which fault on Device memory.
	 */
	device_map((mm_reg_t *)&d->mem_virt, cfg->mem_base, cfg->mem_size,
#ifdef K_MEM_ARM_NORMAL_NC
		   K_MEM_ARM_NORMAL_NC
#else
		   K_MEM_CACHE_NONE
#endif
	);
#else
	d->mem_virt = cfg->mem_base;
#endif

	ret = epf_test_setup_bars(dev);
	if (ret != 0) {
		return ret;
	}

	d->listener.func = cfg->func;
	d->listener.event = epf_test_event;
	pcie_epc_add_listener(cfg->epc, &d->listener);

	k_work_init_delayable(&d->cmd_work, cmd_work_handler);
	k_work_schedule(&d->cmd_work, K_NO_WAIT);

	LOG_INF("%s: fn%u %04x:%04x on %s, test regs in BAR%u", dev->name, cfg->func,
		cfg->hdr.vendor_id, cfg->hdr.device_id, cfg->epc->name, cfg->reg_bar);
	return 0;
}

#define EPF_TEST_BAR_SIZE(n, i) DT_INST_PROP_BY_IDX(n, bar_sizes, i)

#define EPF_TEST_DEFINE(n)                                                                         \
	BUILD_ASSERT(DT_INST_PROP_LEN(n, bar_sizes) == PCIE_EPC_NUM_BARS,                         \
		     "bar-sizes needs 6 entries");                                                 \
	static const struct epf_test_config epf_test_cfg_##n = {                                   \
		.epc = PCIE_EPF_DT_INST_EPC(n),                                                    \
		.func = PCIE_EPF_DT_INST_FUNC(n),                                                  \
		.hdr = PCIE_EPF_DT_HEADER(DT_DRV_INST(n)),                                         \
		.mem_base = DT_REG_ADDR(DT_INST_PHANDLE(n, memory_region)),                        \
		.mem_size = DT_REG_SIZE(DT_INST_PHANDLE(n, memory_region)),                        \
		.bar_size = {EPF_TEST_BAR_SIZE(n, 0), EPF_TEST_BAR_SIZE(n, 1),                     \
			     EPF_TEST_BAR_SIZE(n, 2), EPF_TEST_BAR_SIZE(n, 3),                     \
			     EPF_TEST_BAR_SIZE(n, 4), EPF_TEST_BAR_SIZE(n, 5)},                    \
		.reg_bar = DT_INST_PROP(n, test_reg_bar),                                          \
		.msi_irqs = DT_INST_PROP(n, msi_interrupts),                                       \
		.msix_irqs = DT_INST_PROP(n, msix_interrupts),                                     \
	};                                                                                         \
	static struct epf_test_data epf_test_data_##n;                                             \
	DEVICE_DT_INST_DEFINE(n, epf_test_init, NULL, &epf_test_data_##n, &epf_test_cfg_##n,       \
			      POST_KERNEL, CONFIG_PCIE_EPF_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(EPF_TEST_DEFINE)
