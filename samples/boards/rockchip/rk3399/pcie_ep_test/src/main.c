/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * PCIe endpoint test function compatible with the Linux pci-epf-test
 * register protocol, so the host side can be exercised with the
 * pci_endpoint_test driver and the pcitest tool.
 */

#include <stdint.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/pcie/endpoint/pcie_ep.h>
#include <zephyr/drivers/pcie/endpoint/pcie_ep_cdns.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(pcie_ep_test, LOG_LEVEL_INF);

#define EP_NODE  DT_NODELABEL(pcie0_ep)
#define MEM_NODE DT_NODELABEL(pcie_ep_mem)

/* Linux drivers/pci/endpoint/functions/pci-epf-test.c */
#define COMMAND_RAISE_INTX_IRQ BIT(0)
#define COMMAND_RAISE_MSI_IRQ  BIT(1)
#define COMMAND_RAISE_MSIX_IRQ BIT(2)
#define COMMAND_READ           BIT(3)
#define COMMAND_WRITE          BIT(4)
#define COMMAND_COPY           BIT(5)

#define STATUS_READ_SUCCESS     BIT(0)
#define STATUS_READ_FAIL        BIT(1)
#define STATUS_WRITE_SUCCESS    BIT(2)
#define STATUS_WRITE_FAIL       BIT(3)
#define STATUS_COPY_SUCCESS     BIT(4)
#define STATUS_COPY_FAIL        BIT(5)
#define STATUS_IRQ_RAISED       BIT(6)
#define STATUS_SRC_ADDR_INVALID BIT(7)
#define STATUS_DST_ADDR_INVALID BIT(8)

#define FLAG_USE_DMA BIT(0)

#define CAP_UNALIGNED_ACCESS BIT(0)
#define CAP_MSI              BIT(1)
#define CAP_INTX             BIT(3)

#define IRQ_TYPE_INTX 0U
#define IRQ_TYPE_MSI  1U

#define TEST_MAGIC 0xedfeedbeU

struct epf_test_reg {
	uint32_t magic;
	uint32_t command;
	uint32_t status;
	uint64_t src_addr;
	uint64_t dst_addr;
	uint32_t size;
	uint32_t checksum;
	uint32_t irq_type;
	uint32_t irq_number;
	uint32_t flags;
	uint32_t caps;
	uint32_t doorbell_bar;
	uint32_t doorbell_offset;
	uint32_t doorbell_data;
} __packed;

/* BAR n is backed by pcie_ep_mem + n MiB; BAR5 is 1 MiB, the others 128 KiB. */
#define BAR_STRIDE MB(1)
#define BAR_SIZE(n) (((n) == 5U) ? MB(1) : KB(128))

#define XFER_CHUNK KB(64)

static uint8_t xfer_buf[XFER_CHUNK] __aligned(64);

static uint32_t linux_crc32_le(uint32_t crc, const uint8_t *buf, size_t len)
{
	/* crc32_le(seed, ...) from Linux has no pre/post inversion. */
	return ~crc32_ieee_update(~crc, buf, len);
}

static int host_to_local_crc(const struct device *ep, uint64_t src, uint32_t size, uint32_t *crc)
{
	*crc = UINT32_MAX;

	while (size > 0U) {
		uint32_t len = MIN(size, (uint32_t)XFER_CHUNK);
		int ret = pcie_ep_xfer_data_memcpy(ep, src, (uintptr_t *)xfer_buf, len,
						   PCIE_OB_LOWMEM, HOST_TO_DEVICE);

		if (ret != 0) {
			return ret;
		}

		*crc = linux_crc32_le(*crc, xfer_buf, len);
		src += len;
		size -= len;
	}

	return 0;
}

static int local_to_host_random(const struct device *ep, uint64_t dst, uint32_t size,
				uint32_t *crc)
{
	*crc = UINT32_MAX;

	while (size > 0U) {
		uint32_t len = MIN(size, (uint32_t)XFER_CHUNK);
		int ret;

		sys_rand_get(xfer_buf, len);
		*crc = linux_crc32_le(*crc, xfer_buf, len);

		ret = pcie_ep_xfer_data_memcpy(ep, dst, (uintptr_t *)xfer_buf, len, PCIE_OB_LOWMEM,
					       DEVICE_TO_HOST);
		if (ret != 0) {
			return ret;
		}

		dst += len;
		size -= len;
	}

	return 0;
}

static int host_copy(const struct device *ep, uint64_t src, uint64_t dst, uint32_t size)
{
	while (size > 0U) {
		uint32_t len = MIN(size, (uint32_t)XFER_CHUNK);
		int ret;

		ret = pcie_ep_xfer_data_memcpy(ep, src, (uintptr_t *)xfer_buf, len, PCIE_OB_LOWMEM,
					       HOST_TO_DEVICE);
		if (ret != 0) {
			return ret;
		}

		ret = pcie_ep_xfer_data_memcpy(ep, dst, (uintptr_t *)xfer_buf, len, PCIE_OB_LOWMEM,
					       DEVICE_TO_HOST);
		if (ret != 0) {
			return ret;
		}

		src += len;
		dst += len;
		size -= len;
	}

	return 0;
}

static void raise_irq(const struct device *ep, volatile struct epf_test_reg *reg)
{
	uint32_t type = reg->irq_type;
	uint32_t num = reg->irq_number;
	int ret;

	reg->status |= STATUS_IRQ_RAISED;
	barrier_dmem_fence_full();

	if (type == IRQ_TYPE_INTX) {
		ret = pcie_ep_raise_irq(ep, PCIE_EP_IRQ_LEGACY, 0);
	} else if (type == IRQ_TYPE_MSI) {
		ret = pcie_ep_raise_irq(ep, PCIE_EP_IRQ_MSI, num);
	} else {
		ret = -ENOTSUP;
	}

	if (ret != 0) {
		LOG_WRN("raise irq type %u num %u: %d", type, num, ret);
	}
}

static void handle_command(const struct device *ep, volatile struct epf_test_reg *reg,
			   uint32_t command)
{
	uint32_t status = 0U;
	uint32_t crc;
	int ret;

	if ((reg->flags & FLAG_USE_DMA) != 0U) {
		LOG_ERR("DMA transfers are not supported");
		return;
	}

	switch (command) {
	case COMMAND_RAISE_INTX_IRQ:
	case COMMAND_RAISE_MSI_IRQ:
		break;
	case COMMAND_READ:
		ret = host_to_local_crc(ep, reg->src_addr, reg->size, &crc);
		if (ret != 0) {
			status = STATUS_SRC_ADDR_INVALID | STATUS_READ_FAIL;
		} else {
			status = (crc == reg->checksum) ? STATUS_READ_SUCCESS : STATUS_READ_FAIL;
		}
		break;
	case COMMAND_WRITE:
		ret = local_to_host_random(ep, reg->dst_addr, reg->size, &crc);
		reg->checksum = crc;
		status = (ret == 0) ? STATUS_WRITE_SUCCESS
				    : (STATUS_DST_ADDR_INVALID | STATUS_WRITE_FAIL);
		break;
	case COMMAND_COPY:
		ret = host_copy(ep, reg->src_addr, reg->dst_addr, reg->size);
		status = (ret == 0) ? STATUS_COPY_SUCCESS : STATUS_COPY_FAIL;
		break;
	default:
		LOG_ERR("unsupported command 0x%x", command);
		return;
	}

	reg->status = status;
	raise_irq(ep, reg);
}

int main(void)
{
	const struct device *ep = DEVICE_DT_GET(EP_NODE);
	uintptr_t mem = DT_REG_ADDR(MEM_NODE);
	volatile struct epf_test_reg *reg = (volatile struct epf_test_reg *)mem;
	int ret;

	BUILD_ASSERT(DT_REG_SIZE(MEM_NODE) >= (6 * BAR_STRIDE), "pcie_ep_mem too small");

	if (!device_is_ready(ep)) {
		LOG_ERR("endpoint controller not ready");
		return 0;
	}

	for (uint8_t bar = 0U; bar < 6U; bar++) {
		uintptr_t phys = mem + (bar * BAR_STRIDE);

		memset((void *)phys, 0, BAR_SIZE(bar));
		ret = pcie_ep_cdns_set_bar(ep, 0, bar, phys, BAR_SIZE(bar), 0);
		if (ret != 0) {
			LOG_ERR("BAR%u setup failed: %d", bar, ret);
			return 0;
		}
	}

	reg->caps = CAP_UNALIGNED_ACCESS | CAP_MSI | CAP_INTX;
	reg->magic = TEST_MAGIC;

	ret = pcie_ep_cdns_start(ep, BIT(0));
	if (ret != 0) {
		LOG_ERR("link start failed: %d", ret);
		return 0;
	}

	LOG_INF("pci-epf-test ready, BARs at 0x%lx", (unsigned long)mem);

	for (;;) {
		uint32_t command = reg->command;

		if (command != 0U) {
			reg->command = 0U;
			reg->status = 0U;
			handle_command(ep, reg, command);
		}

		k_msleep(1);
	}

	return 0;
}
