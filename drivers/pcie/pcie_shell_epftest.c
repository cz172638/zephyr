/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * "pcie epftest": host side of the PCI endpoint test function (the
 * register block of Linux pci-epf-test and Zephyr's zephyr,pcie-epf-test in
 * a BAR of the endpoint), like Linux pcitest: the endpoint reads, writes or
 * copies a buffer in this host's memory and the host checks the CRC or the
 * data. The buffers are static; their bus address is their physical
 * address (no inbound translation in the root complex). Memory space and
 * bus mastering are turned on in the endpoint first. Status is polled,
 * the endpoint's INTx is not used.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/drivers/pcie/pcie.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

#include "pcie_shell_priv.h"

#define REG_MAGIC      0x00U
#define REG_COMMAND    0x04U
#define REG_STATUS     0x08U
#define REG_SRC_LO     0x0cU
#define REG_SRC_HI     0x10U
#define REG_DST_LO     0x14U
#define REG_DST_HI     0x18U
#define REG_SIZE       0x1cU
#define REG_CHECKSUM   0x20U
#define REG_IRQ_TYPE   0x24U
#define REG_IRQ_NUMBER 0x28U
#define REG_FLAGS      0x2cU
#define REG_CAPS       0x30U

#define CMD_READ  BIT(3)
#define CMD_WRITE BIT(4)
#define CMD_COPY  BIT(5)

#define STS_READ_SUCCESS     BIT(0)
#define STS_READ_FAIL        BIT(1)
#define STS_WRITE_SUCCESS    BIT(2)
#define STS_WRITE_FAIL       BIT(3)
#define STS_COPY_SUCCESS     BIT(4)
#define STS_COPY_FAIL        BIT(5)
#define STS_SRC_ADDR_INVALID BIT(7)
#define STS_DST_ADDR_INVALID BIT(8)

#define IRQ_TYPE_INTX 0U
#define TIMEOUT_MS    2000

#define BUF_SIZE CONFIG_PCIE_CTRL_SHELL_EPFTEST_SIZE

static uint8_t src_buf[BUF_SIZE] __aligned(4096);
static uint8_t dst_buf[BUF_SIZE] __aligned(4096);

static uint64_t bus_addr(void *p)
{
#if defined(CONFIG_MMU)
	return k_mem_phys_addr(p);
#else
	return (uintptr_t)p;
#endif
}

/* Linux crc32_le(~0, ...), as the test function stores it */
static uint32_t crc_linux(const uint8_t *p, size_t n)
{
	return ~crc32_ieee(p, n);
}

/* test pattern, xorshift32 seeded from the cycle counter */
static void fill(uint8_t *p, size_t n)
{
	uint32_t x = k_cycle_get_32() | 1U;

	for (size_t i = 0; i < n; i++) {
		x ^= x << 13;
		x ^= x >> 17;
		x ^= x << 5;
		p[i] = (uint8_t)x;
	}
	sys_cache_data_flush_range(p, n);
}

int pcie_shell_cmd_epftest(const struct shell *sh, size_t argc, char **argv)
{
	const struct pcie_shell_ctrl *c = pcie_shell_active(sh);
	unsigned int b, d, f;
	struct pcie_bar bar;
	uint32_t cmd, cmdsts, ok, size = 4096U, sts = 0;
	unsigned long barno = 0;
	pcie_bdf_t bdf;
	mm_reg_t regs;
	int64_t t0, t1;
	int ret = 0;

	if (c == NULL) {
		return -ENODEV;
	}
	if (sscanf(argv[1], "%x:%x.%x", &b, &d, &f) != 3) {
		shell_error(sh, "b:d.f expected");
		return -EINVAL;
	}
	bdf = PCIE_BDF(b, d, f);
	if (strcmp(argv[2], "read") == 0) {
		cmd = CMD_READ;
		ok = STS_READ_SUCCESS;
	} else if (strcmp(argv[2], "write") == 0) {
		cmd = CMD_WRITE;
		ok = STS_WRITE_SUCCESS;
	} else if (strcmp(argv[2], "copy") == 0) {
		cmd = CMD_COPY;
		ok = STS_COPY_SUCCESS;
	} else {
		shell_error(sh, "read, write or copy");
		return -EINVAL;
	}
	if (argc > 3) {
		size = strtoul(argv[3], NULL, 0);
	}
	if (argc > 4) {
		barno = strtoul(argv[4], NULL, 0);
	}
	if (size == 0U || size > BUF_SIZE || barno > 5U) {
		shell_error(sh, "size 1..%u, BAR 0..5", BUF_SIZE);
		return -EINVAL;
	}
	/*
	 * Memory space for the registers, bus mastering for the transfers
	 * (generic enumeration leaves both to the device driver); without bus
	 * mastering the test function keeps the command pending.
	 */
	cmdsts = c->cfg_read(c->dev, bdf, 1);
	if ((cmdsts & (PCIE_CONF_CMDSTAT_MEM | PCIE_CONF_CMDSTAT_MASTER)) !=
	    (PCIE_CONF_CMDSTAT_MEM | PCIE_CONF_CMDSTAT_MASTER)) {
		if (c->cfg_write == NULL) {
			shell_error(sh, "command register %04x: memory space or bus mastering off",
				    cmdsts & 0xffffU);
			return -EACCES;
		}
		c->cfg_write(c->dev, bdf, 1,
			     (cmdsts & 0xffffU) | PCIE_CONF_CMDSTAT_MEM | PCIE_CONF_CMDSTAT_MASTER);
		shell_print(sh, "command register %04x -> %04x", cmdsts & 0xffffU,
			    c->cfg_read(c->dev, bdf, 1) & 0xffffU);
	}
	if (!pcie_get_mbar(bdf, barno, &bar) || bar.size < 0x40U) {
		shell_error(sh, "%02x:%02x.%x: no memory BAR%lu", b, d, f, barno);
		return -ENODEV;
	}
	device_map(&regs, bar.phys_addr, 0x1000, K_MEM_CACHE_NONE);

	/* both buffers random, the destination gets overwritten */
	fill(src_buf, size);
	fill(dst_buf, size);

	sys_write32(0, regs + REG_STATUS);
	sys_write32(0, regs + REG_FLAGS);
	sys_write32(IRQ_TYPE_INTX, regs + REG_IRQ_TYPE);
	sys_write32(0, regs + REG_IRQ_NUMBER);
	sys_write32(size, regs + REG_SIZE);
	sys_write32((uint32_t)bus_addr(src_buf), regs + REG_SRC_LO);
	sys_write32((uint32_t)(bus_addr(src_buf) >> 32), regs + REG_SRC_HI);
	sys_write32((uint32_t)bus_addr(dst_buf), regs + REG_DST_LO);
	sys_write32((uint32_t)(bus_addr(dst_buf) >> 32), regs + REG_DST_HI);
	if (cmd == CMD_READ) {
		sys_write32(crc_linux(src_buf, size), regs + REG_CHECKSUM);
	}

	t0 = k_uptime_get();
	sys_write32(cmd, regs + REG_COMMAND);
	do {
		sts = sys_read32(regs + REG_STATUS);
		if ((sts & (ok | (ok << 1) | STS_SRC_ADDR_INVALID | STS_DST_ADDR_INVALID)) != 0U) {
			break;
		}
		k_msleep(1);
	} while (k_uptime_get() - t0 < TIMEOUT_MS);
	t1 = k_uptime_get();

	if ((sts & ok) == 0U) {
		shell_error(sh, "%s: status %08x, command %08x after %lld ms%s", argv[2], sts,
			    sys_read32(regs + REG_COMMAND), t1 - t0,
			    sts == 0U ? " (no answer: test function not running?)" : "");
		ret = -EIO;
		goto out;
	}

	sys_cache_data_invd_range(dst_buf, size);
	if (cmd == CMD_WRITE) {
		uint32_t want = sys_read32(regs + REG_CHECKSUM);
		uint32_t got = crc_linux(dst_buf, size);

		if (want != got) {
			shell_error(sh, "write: CRC %08x, endpoint %08x", got, want);
			ret = -EIO;
		}
	} else if (cmd == CMD_COPY) {
		if (memcmp(src_buf, dst_buf, size) != 0) {
			shell_error(sh, "copy: data differ");
			ret = -EIO;
		}
	}
	if (ret == 0) {
		shell_print(sh, "%s %u bytes at 0x%llx: OK (status %08x, %lld ms)", argv[2], size,
			    (unsigned long long)bus_addr(cmd == CMD_READ ? src_buf : dst_buf),
			    sts, t1 - t0);
	}
out:
	device_unmap(regs, 0x1000);
	return ret;
}
