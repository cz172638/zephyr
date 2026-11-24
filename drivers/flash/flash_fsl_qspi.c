/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPI NOR flash on the Freescale QuadSPI controller of the Layerscape
 * LS1021A and LS1012A (big-endian registers). The operation sequence
 * follows the Linux spi-fsl-qspi driver: every command is put into one
 * LUT sequence, the flash address as single bytes; up to 124 bytes are
 * read through the RX FIFO, longer reads through the 1 KiB AHB buffer,
 * writes through the 64 byte TX FIFO. Single wire; parts above 16 MiB
 * use the 4-byte address opcodes.
 */

#define DT_DRV_COMPAT fsl_ls1021a_qspi

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/device_mmio.h>

#include "spi_nor.h"

LOG_MODULE_REGISTER(flash_fsl_qspi, CONFIG_FLASH_LOG_LEVEL);

#define QSPI_MCR           0x000
#define QSPI_MCR_RESERVED  GENMASK(19, 16)
#define QSPI_MCR_MDIS      BIT(14)
#define QSPI_MCR_CLR_TXF   BIT(11)
#define QSPI_MCR_CLR_RXF   BIT(10)
#define QSPI_MCR_END_CFG   GENMASK(3, 2)
#define QSPI_MCR_SWRSTHD   BIT(1)
#define QSPI_MCR_SWRSTSD   BIT(0)
#define QSPI_IPCR          0x008
#define QSPI_IPCR_SEQID(x) ((uint32_t)(x) << 24)
#define QSPI_BUF0CR        0x010
#define QSPI_BUF1CR        0x014
#define QSPI_BUF2CR        0x018
#define QSPI_BUFXCR_INVALID_MSTRID 0xe
#define QSPI_BUF3CR        0x01c
#define QSPI_BUF3CR_ALLMST BIT(31)
#define QSPI_BUF3CR_ADATSZ(x) ((uint32_t)(x) << 8)
#define QSPI_BFGENCR       0x020
#define QSPI_BFGENCR_SEQID(x) ((uint32_t)(x) << 12)
#define QSPI_BUF0IND       0x030
#define QSPI_BUF1IND       0x034
#define QSPI_BUF2IND       0x038
#define QSPI_SFAR          0x100
#define QSPI_SMPR          0x108
#define QSPI_RBCT          0x110
#define QSPI_RBCT_WMRK     GENMASK(4, 0)
#define QSPI_RBCT_RXBRD_IPS BIT(8)
#define QSPI_TBDR          0x154
#define QSPI_SR            0x15c
#define QSPI_SR_BUSY       BIT(0)
#define QSPI_SR_IP_ACC     BIT(1)
#define QSPI_SR_AHB_ACC    BIT(2)
#define QSPI_FR            0x160
#define QSPI_FR_TFF        BIT(0)
#define QSPI_RSER          0x164
#define QSPI_SPTRCLR       0x16c
#define QSPI_SPTRCLR_IPPTRC BIT(8)
#define QSPI_SPTRCLR_BFPTRC BIT(0)
#define QSPI_SFA1AD        0x180
#define QSPI_SFA2AD        0x184
#define QSPI_SFB1AD        0x188
#define QSPI_SFB2AD        0x18c
#define QSPI_RBDR(i)       (0x200 + 4 * (i))
#define QSPI_LUTKEY        0x300
#define QSPI_LUTKEY_VALUE  0x5af05af0
#define QSPI_LCKCR         0x304
#define QSPI_LCKCR_LOCK    BIT(0)
#define QSPI_LCKCR_UNLOCK  BIT(1)
#define QSPI_LUT(i)        (0x310 + 4 * (i))

#define LUT_CMD   1
#define LUT_MODE  4
#define LUT_DUMMY 3
#define LUT_READ  7
#define LUT_WRITE 8
#define LUT_DEF(idx, ins, opr) ((((uint32_t)(ins) << 10) | (opr)) << (((idx) % 2) * 16))

#define SEQID       15
#define RX_FIFO     128
#define TX_FIFO     64
#define AHB_BUF     1024
#define PAGE_SIZE   256

struct fsl_qspi_config {
	uintptr_t regs_phys;
	uintptr_t mem_phys;
	size_t size;
	size_t erase_size;
	struct flash_parameters params;
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	struct flash_pages_layout layout;
#endif
};

struct fsl_qspi_data {
	mm_reg_t regs;
	mm_reg_t mem;
	struct k_mutex lock;
	uint8_t addr_len;
	uint8_t jedec_id[3];
};

struct qspi_op {
	uint8_t cmd;
	uint8_t addr_len;
	uint32_t addr;
	uint8_t dummy;
	bool out;
	void *buf;
	size_t len;
};

static uint32_t rd(const struct device *dev, uint32_t off)
{
	return sys_be32_to_cpu(sys_read32(((struct fsl_qspi_data *)dev->data)->regs + off));
}

static void wr(const struct device *dev, uint32_t off, uint32_t v)
{
	sys_write32(sys_cpu_to_be32(v), ((struct fsl_qspi_data *)dev->data)->regs + off);
}

static int wait_clear(const struct device *dev, uint32_t off, uint32_t mask)
{
	for (int i = 0; i < 100000; i++) {
		if ((rd(dev, off) & mask) == 0) {
			return 0;
		}
		k_busy_wait(1);
	}
	return -ETIMEDOUT;
}

static void invalidate(const struct device *dev)
{
	uint32_t mcr = rd(dev, QSPI_MCR);

	wr(dev, QSPI_MCR, mcr | QSPI_MCR_SWRSTHD | QSPI_MCR_SWRSTSD);
	k_busy_wait(1);
	wr(dev, QSPI_MCR, mcr & ~(QSPI_MCR_SWRSTHD | QSPI_MCR_SWRSTSD));
}

static void prepare_lut(const struct device *dev, const struct qspi_op *op)
{
	uint32_t lut[4] = {0};
	int idx = 1;

	lut[0] = LUT_DEF(0, LUT_CMD, op->cmd);
	/* address bytes one by one as in Linux: LUT_ADDR fails in some cases */
	for (int i = 0; i < op->addr_len; i++) {
		lut[idx / 2] |= LUT_DEF(idx, LUT_MODE,
					(op->addr >> (8 * (op->addr_len - i - 1))) & 0xff);
		idx++;
	}
	if (op->dummy != 0) {
		lut[idx / 2] |= LUT_DEF(idx, LUT_DUMMY, op->dummy * 8);
		idx++;
	}
	if (op->len != 0) {
		lut[idx / 2] |= LUT_DEF(idx, op->out ? LUT_WRITE : LUT_READ, 0);
	}

	wr(dev, QSPI_LUTKEY, QSPI_LUTKEY_VALUE);
	wr(dev, QSPI_LCKCR, QSPI_LCKCR_UNLOCK);
	for (int i = 0; i < 4; i++) {
		wr(dev, QSPI_LUT(SEQID * 4 + i), lut[i]);
	}
	wr(dev, QSPI_LUTKEY, QSPI_LUTKEY_VALUE);
	wr(dev, QSPI_LCKCR, QSPI_LCKCR_LOCK);
}

/* Data words: bytes in memory order, through the big-endian register access */
static void fill_txfifo(const struct device *dev, const uint8_t *buf, size_t len)
{
	uint32_t v;
	size_t i;

	for (i = 0; i < len; i += 4) {
		v = 0;
		memcpy(&v, buf + i, MIN(4U, len - i));
		wr(dev, QSPI_TBDR, v);
	}
}

static void read_rxfifo(const struct device *dev, uint8_t *buf, size_t len)
{
	uint32_t v;

	for (size_t i = 0; i < len; i += 4) {
		v = rd(dev, QSPI_RBDR(i / 4));
		memcpy(buf + i, &v, MIN(4U, len - i));
	}
}

static int exec_op(const struct device *dev, const struct qspi_op *op)
{
	const struct fsl_qspi_config *cfg = dev->config;
	struct fsl_qspi_data *data = dev->data;
	int ret;

	ret = wait_clear(dev, QSPI_SR, QSPI_SR_IP_ACC | QSPI_SR_AHB_ACC);
	if (ret != 0) {
		return ret;
	}
	wr(dev, QSPI_SFAR, cfg->mem_phys);
	wr(dev, QSPI_MCR, rd(dev, QSPI_MCR) | QSPI_MCR_CLR_RXF | QSPI_MCR_CLR_TXF);
	wr(dev, QSPI_SPTRCLR, QSPI_SPTRCLR_BFPTRC | QSPI_SPTRCLR_IPPTRC);
	wr(dev, QSPI_BUF0CR, QSPI_BUFXCR_INVALID_MSTRID);
	wr(dev, QSPI_BUF1CR, QSPI_BUFXCR_INVALID_MSTRID);
	wr(dev, QSPI_BUF2CR, QSPI_BUFXCR_INVALID_MSTRID);
	prepare_lut(dev, op);

	if (!op->out && op->len > RX_FIFO - 4) {
		/* through the AHB buffer: the read sequence is SEQID (BFGENCR) */
		memcpy(op->buf, (const void *)data->mem, op->len);
	} else {
		wr(dev, QSPI_RBCT, QSPI_RBCT_WMRK | QSPI_RBCT_RXBRD_IPS);
		if (op->out && op->len != 0) {
			fill_txfifo(dev, op->buf, op->len);
		}
		wr(dev, QSPI_FR, UINT32_MAX);
		wr(dev, QSPI_IPCR, QSPI_IPCR_SEQID(SEQID) | op->len);
		ret = -ETIMEDOUT;
		for (int i = 0; i < 100000; i++) {
			if ((rd(dev, QSPI_FR) & QSPI_FR_TFF) != 0) {
				ret = 0;
				break;
			}
			k_busy_wait(1);
		}
		if (ret == 0) {
			ret = wait_clear(dev, QSPI_SR, QSPI_SR_BUSY);
		}
		if (ret == 0 && !op->out && op->len != 0) {
			read_rxfifo(dev, op->buf, op->len);
		}
	}
	invalidate(dev);
	return ret;
}

static int cmd(const struct device *dev, uint8_t opcode)
{
	struct qspi_op op = {.cmd = opcode};

	return exec_op(dev, &op);
}

static int wait_ready(const struct device *dev, int timeout_ms)
{
	int64_t end = k_uptime_get() + timeout_ms;
	uint8_t sr;
	struct qspi_op op = {.cmd = SPI_NOR_CMD_RDSR, .buf = &sr, .len = 1};
	int ret;

	do {
		ret = exec_op(dev, &op);
		if (ret != 0) {
			return ret;
		}
		if ((sr & SPI_NOR_WIP_BIT) == 0) {
			return 0;
		}
		k_yield();
	} while (k_uptime_get() < end);
	return -ETIMEDOUT;
}

static bool range_ok(const struct device *dev, off_t off, size_t len)
{
	const struct fsl_qspi_config *cfg = dev->config;

	return off >= 0 && (size_t)off <= cfg->size && len <= cfg->size - (size_t)off;
}

static int fsl_qspi_read(const struct device *dev, off_t off, void *buf, size_t len)
{
	struct fsl_qspi_data *data = dev->data;
	uint8_t *p = buf;
	int ret = 0;

	if (!range_ok(dev, off, len)) {
		return -EINVAL;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	while (len > 0 && ret == 0) {
		size_t n = MIN(len, (size_t)AHB_BUF);
		struct qspi_op op = {
			.cmd = data->addr_len == 4 ? SPI_NOR_CMD_READ_FAST_4B : SPI_NOR_CMD_READ_FAST,
			.addr_len = data->addr_len,
			.addr = off,
			.dummy = 1,
			.buf = p,
		};

		/* the AHB path takes multiples of 8 bytes */
		if (n > RX_FIFO - 4) {
			n = ROUND_DOWN(n, 8);
		}
		op.len = n;
		ret = exec_op(dev, &op);
		p += n;
		off += n;
		len -= n;
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static int fsl_qspi_write(const struct device *dev, off_t off, const void *buf, size_t len)
{
	struct fsl_qspi_data *data = dev->data;
	const uint8_t *p = buf;
	int ret = 0;

	if (!range_ok(dev, off, len)) {
		return -EINVAL;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	while (len > 0 && ret == 0) {
		/* within one page, at most the TX FIFO */
		size_t n = MIN(len, MIN((size_t)TX_FIFO, PAGE_SIZE - ((size_t)off % PAGE_SIZE)));
		struct qspi_op op = {
			.cmd = data->addr_len == 4 ? SPI_NOR_CMD_PP_4B : SPI_NOR_CMD_PP,
			.addr_len = data->addr_len,
			.addr = off,
			.out = true,
			.buf = (void *)p,
			.len = n,
		};

		ret = cmd(dev, SPI_NOR_CMD_WREN);
		if (ret == 0) {
			ret = exec_op(dev, &op);
		}
		if (ret == 0) {
			ret = wait_ready(dev, 10);
		}
		p += n;
		off += n;
		len -= n;
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static int fsl_qspi_erase(const struct device *dev, off_t off, size_t len)
{
	const struct fsl_qspi_config *cfg = dev->config;
	struct fsl_qspi_data *data = dev->data;
	uint8_t opcode;
	int ret = 0;

	if (!range_ok(dev, off, len) || (off % cfg->erase_size) != 0 ||
	    (len % cfg->erase_size) != 0) {
		return -EINVAL;
	}
	if (cfg->erase_size == KB(4)) {
		opcode = data->addr_len == 4 ? SPI_NOR_CMD_SE_4B : SPI_NOR_CMD_SE;
	} else {
		opcode = data->addr_len == 4 ? SPI_NOR_CMD_BE_4B : SPI_NOR_CMD_BE;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	for (; len > 0 && ret == 0; off += cfg->erase_size, len -= cfg->erase_size) {
		struct qspi_op op = {.cmd = opcode, .addr_len = data->addr_len, .addr = off};

		ret = cmd(dev, SPI_NOR_CMD_WREN);
		if (ret == 0) {
			ret = exec_op(dev, &op);
		}
		if (ret == 0) {
			ret = wait_ready(dev, 3000);
		}
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static const struct flash_parameters *fsl_qspi_get_parameters(const struct device *dev)
{
	const struct fsl_qspi_config *cfg = dev->config;

	return &cfg->params;
}

static int fsl_qspi_get_size(const struct device *dev, uint64_t *size)
{
	const struct fsl_qspi_config *cfg = dev->config;

	*size = cfg->size;
	return 0;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void fsl_qspi_pages_layout(const struct device *dev,
				  const struct flash_pages_layout **layout, size_t *layout_size)
{
	const struct fsl_qspi_config *cfg = dev->config;

	*layout = &cfg->layout;
	*layout_size = 1;
}
#endif

#if defined(CONFIG_FLASH_JESD216_API)
static int fsl_qspi_read_jedec_id(const struct device *dev, uint8_t *id)
{
	struct fsl_qspi_data *data = dev->data;

	memcpy(id, data->jedec_id, sizeof(data->jedec_id));
	return 0;
}
#endif

static int fsl_qspi_init(const struct device *dev)
{
	const struct fsl_qspi_config *cfg = dev->config;
	struct fsl_qspi_data *data = dev->data;
	struct qspi_op op = {.cmd = SPI_NOR_CMD_RDID, .buf = data->jedec_id, .len = 3};
	int ret;

	device_map(&data->regs, cfg->regs_phys, KB(4), K_MEM_CACHE_NONE);
	device_map(&data->mem, cfg->mem_phys, AHB_BUF, K_MEM_CACHE_NONE);
	k_mutex_init(&data->lock);
	data->addr_len = cfg->size > MB(16) ? 4 : 3;

	/* reset, then set up while disabled (Linux fsl_qspi_default_setup) */
	wr(dev, QSPI_MCR, QSPI_MCR_SWRSTSD | QSPI_MCR_SWRSTHD);
	k_busy_wait(1);
	wr(dev, QSPI_MCR, QSPI_MCR_MDIS | QSPI_MCR_RESERVED);
	wr(dev, QSPI_SMPR, rd(dev, QSPI_SMPR) & ~(GENMASK(18, 16) | BIT(6) | BIT(5) | BIT(0)));
	wr(dev, QSPI_BUF0IND, 0);
	wr(dev, QSPI_BUF1IND, 0);
	wr(dev, QSPI_BUF2IND, 0);
	wr(dev, QSPI_BFGENCR, QSPI_BFGENCR_SEQID(SEQID));
	wr(dev, QSPI_RBCT, QSPI_RBCT_WMRK);
	wr(dev, QSPI_BUF3CR, QSPI_BUF3CR_ALLMST | QSPI_BUF3CR_ADATSZ(AHB_BUF / 8));
	/* chip select A1 only: the 1 KiB AHB buffer window at the start */
	wr(dev, QSPI_SFA1AD, cfg->mem_phys + AHB_BUF);
	wr(dev, QSPI_SFA2AD, cfg->mem_phys + 2 * AHB_BUF);
	wr(dev, QSPI_SFB1AD, cfg->mem_phys + 3 * AHB_BUF);
	wr(dev, QSPI_SFB2AD, cfg->mem_phys + 4 * AHB_BUF);
	wr(dev, QSPI_MCR, QSPI_MCR_RESERVED | QSPI_MCR_END_CFG);
	wr(dev, QSPI_FR, UINT32_MAX);
	wr(dev, QSPI_RSER, 0);

	ret = exec_op(dev, &op);
	if (ret != 0) {
		LOG_ERR("RDID failed (%d)", ret);
		return ret;
	}
	LOG_INF("%s: JEDEC %02x %02x %02x, %zu KiB, erase %zu KiB, %u byte addresses", dev->name,
		data->jedec_id[0], data->jedec_id[1], data->jedec_id[2], cfg->size / 1024,
		cfg->erase_size / 1024, data->addr_len);
	return ret;
}

static DEVICE_API(flash, fsl_qspi_api) = {
	.read = fsl_qspi_read,
	.write = fsl_qspi_write,
	.erase = fsl_qspi_erase,
	.get_parameters = fsl_qspi_get_parameters,
	.get_size = fsl_qspi_get_size,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = fsl_qspi_pages_layout,
#endif
#if defined(CONFIG_FLASH_JESD216_API)
	.read_jedec_id = fsl_qspi_read_jedec_id,
#endif
};

#define FLASH_NODE(n) DT_INST_CHILD(n, flash_0)

#define FSL_QSPI_DEFINE(n)                                                                         \
	static const struct fsl_qspi_config fsl_qspi_cfg_##n = {                                   \
		.regs_phys = DT_INST_REG_ADDR_BY_NAME(n, ctrl),                                    \
		.mem_phys = DT_INST_REG_ADDR_BY_NAME(n, mem),                                      \
		.size = DT_REG_SIZE(FLASH_NODE(n)),                                                \
		.erase_size = DT_PROP(FLASH_NODE(n), erase_block_size),                            \
		.params = {                                                                        \
			.write_block_size = DT_PROP(FLASH_NODE(n), write_block_size),              \
			.erase_value = 0xff,                                                       \
		},                                                                                 \
		IF_ENABLED(CONFIG_FLASH_PAGE_LAYOUT,                                               \
			   (.layout = {                                                            \
				.pages_count = DT_REG_SIZE(FLASH_NODE(n)) /                        \
					       DT_PROP(FLASH_NODE(n), erase_block_size),           \
				.pages_size = DT_PROP(FLASH_NODE(n), erase_block_size),            \
			   },))                                                                    \
	};                                                                                         \
	static struct fsl_qspi_data fsl_qspi_data_##n;                                             \
	DEVICE_DT_INST_DEFINE(n, fsl_qspi_init, NULL, &fsl_qspi_data_##n, &fsl_qspi_cfg_##n,       \
			      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY, &fsl_qspi_api);

DT_INST_FOREACH_STATUS_OKAY(FSL_QSPI_DEFINE)
