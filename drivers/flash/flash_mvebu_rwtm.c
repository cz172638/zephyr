/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Armada 3700 SPI NOR through the Cortex-M3. The CM3 owns the SPI
 * controller; every flash call here is a request over the rWTM mailbox,
 * with the data in a shared, uncached DDR buffer.
 */

#define DT_DRV_COMPAT marvell_armada_3700_rwtm_flash

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/drivers/mbox/mbox_mvebu_rwtm.h>
#include <zephyr/sys/device_mmio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_mvebu_rwtm, CONFIG_FLASH_LOG_LEVEL);

/* erase in pieces, each one request with its own timeout */
#define RFLASH_ERASE_CHUNK 0x10000

struct rflash_config {
	DEVICE_MMIO_NAMED_ROM(buf);
	size_t buf_size;
	struct mbox_dt_spec tx;
	struct mbox_dt_spec rx;
	size_t size;
	struct flash_parameters params;
	struct flash_pages_layout layout;
};

struct rflash_data {
	DEVICE_MMIO_NAMED_RAM(buf);
	struct k_mutex lock;
	struct k_sem done;
	uint32_t seq;
	struct mvebu_rflash_rsp rsp;
};

#define DEV_CFG(dev)  ((const struct rflash_config *)(dev)->config)
#define DEV_DATA(dev) ((struct rflash_data *)(dev)->data)

static void rflash_rx(const struct device *mbox, mbox_channel_id_t ch, void *user_data,
		      struct mbox_msg *msg)
{
	struct rflash_data *data = user_data;

	ARG_UNUSED(mbox);
	ARG_UNUSED(ch);

	if (msg == NULL || msg->size < sizeof(data->rsp)) {
		return;
	}
	memcpy(&data->rsp, msg->data, sizeof(data->rsp));
	k_sem_give(&data->done);
}

/* one request, lock held */
static int rflash_call_timeout(const struct device *dev, uint32_t op, uint32_t offset,
			       uint32_t len, k_timeout_t timeout)
{
	const struct rflash_config *cfg = dev->config;
	struct rflash_data *data = dev->data;
	struct mvebu_rflash_req req = {
		.seq = ++data->seq,
		.op = op,
		.offset = offset,
		.len = len,
	};
	struct mbox_msg msg = {
		.data = &req,
		.size = sizeof(req),
	};
	int ret;

	k_sem_reset(&data->done);
	ret = mbox_send_dt(&cfg->tx, &msg);
	if (ret < 0) {
		return ret;
	}

	/* a late answer to a timed out request is dropped by its seq */
	do {
		ret = k_sem_take(&data->done, timeout);
		if (ret < 0) {
			LOG_ERR("op %u @0x%x+0x%x: no answer from the CM3", op, offset, len);
			return -ETIMEDOUT;
		}
	} while (data->rsp.seq != req.seq);

	return data->rsp.ret;
}

static int rflash_call(const struct device *dev, uint32_t op, uint32_t offset, uint32_t len)
{
	return rflash_call_timeout(dev, op, offset, len,
				   K_MSEC(CONFIG_FLASH_MVEBU_RWTM_TIMEOUT_MS));
}

static bool rflash_range_ok(const struct device *dev, off_t offset, size_t len)
{
	const struct rflash_config *cfg = dev->config;

	return offset >= 0 && (size_t)offset <= cfg->size && len <= cfg->size - offset;
}

static int rflash_read(const struct device *dev, off_t offset, void *buf, size_t len)
{
	const struct rflash_config *cfg = dev->config;
	struct rflash_data *data = dev->data;
	uint8_t *shm = (uint8_t *)DEVICE_MMIO_NAMED_GET(dev, buf);
	uint8_t *p = buf;
	int ret = 0;

	if (!rflash_range_ok(dev, offset, len)) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	while (len > 0) {
		size_t n = MIN(len, cfg->buf_size);

		ret = rflash_call(dev, MVEBU_RFLASH_OP_READ, offset, n);
		if (ret < 0) {
			break;
		}
		memcpy(p, shm, n);
		p += n;
		offset += n;
		len -= n;
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

static int rflash_write(const struct device *dev, off_t offset, const void *buf, size_t len)
{
	const struct rflash_config *cfg = dev->config;
	struct rflash_data *data = dev->data;
	uint8_t *shm = (uint8_t *)DEVICE_MMIO_NAMED_GET(dev, buf);
	const uint8_t *p = buf;
	int ret = 0;

	if (!rflash_range_ok(dev, offset, len)) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	while (len > 0) {
		size_t n = MIN(len, cfg->buf_size);

		memcpy(shm, p, n);
		/* the buffer is uncached; order it before the doorbell */
		barrier_dsync_fence_full();
		ret = rflash_call(dev, MVEBU_RFLASH_OP_WRITE, offset, n);
		if (ret < 0) {
			break;
		}
		p += n;
		offset += n;
		len -= n;
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

static int rflash_erase(const struct device *dev, off_t offset, size_t len)
{
	const struct rflash_config *cfg = dev->config;
	struct rflash_data *data = dev->data;
	int ret = 0;

	if (!rflash_range_ok(dev, offset, len) || (offset % cfg->layout.pages_size) != 0 ||
	    (len % cfg->layout.pages_size) != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	while (len > 0) {
		/* whole 64 KiB blocks where aligned, so the CM3 can use them */
		size_t n = MIN(len, RFLASH_ERASE_CHUNK - (offset % RFLASH_ERASE_CHUNK));

		ret = rflash_call(dev, MVEBU_RFLASH_OP_ERASE, offset, n);
		if (ret < 0) {
			break;
		}
		offset += n;
		len -= n;
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

#ifdef CONFIG_FWUPDATE_PROGRAM_EXTERNAL
#include <zephyr/cache.h>
#include <zephyr/kernel/internal/mm.h>

/* fwupdate: the CM3 programs the image straight from our RAM */
int fwupdate_program_external(const void *img, size_t len, uint32_t *crc)
{
	const struct device *dev = DEVICE_DT_INST_GET(0);
	struct rflash_data *data = dev->data;
	uintptr_t phys = k_mem_phys_addr((void *)img);
	int ret;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	/* the CM3 reads DDR behind the caches */
	sys_cache_data_flush_range((void *)img, len);

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = rflash_call_timeout(dev, MVEBU_RFLASH_OP_FWUPDATE, phys, len,
				  K_SECONDS(CONFIG_FLASH_MVEBU_RWTM_FWUPDATE_TIMEOUT));
	if (ret == 0) {
		*crc = data->rsp.val[0];
	}
	k_mutex_unlock(&data->lock);

	return ret;
}
#endif

static const struct flash_parameters *rflash_get_parameters(const struct device *dev)
{
	return &DEV_CFG(dev)->params;
}

static int rflash_get_size(const struct device *dev, uint64_t *size)
{
	*size = DEV_CFG(dev)->size;
	return 0;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static void rflash_page_layout(const struct device *dev, const struct flash_pages_layout **layout,
			       size_t *layout_size)
{
	*layout = &DEV_CFG(dev)->layout;
	*layout_size = 1;
}
#endif

static DEVICE_API(flash, rflash_api) = {
	.read = rflash_read,
	.write = rflash_write,
	.erase = rflash_erase,
	.get_parameters = rflash_get_parameters,
	.get_size = rflash_get_size,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = rflash_page_layout,
#endif
};

static int rflash_init(const struct device *dev)
{
	const struct rflash_config *cfg = dev->config;
	struct rflash_data *data = dev->data;
	int ret;

	DEVICE_MMIO_NAMED_MAP(dev, buf, K_MEM_CACHE_NONE);
	k_mutex_init(&data->lock);
	k_sem_init(&data->done, 0, 1);

	ret = mbox_register_callback_dt(&cfg->rx, rflash_rx, data);
	if (ret < 0) {
		return ret;
	}
	return mbox_set_enabled_dt(&cfg->rx, true);
}

#define RFLASH_CHILD(n) DT_INST_CHILD(n, flash_0)

#define RFLASH_DEFINE(n)                                                                           \
	static struct rflash_data rflash_data_##n;                                                 \
	static const struct rflash_config rflash_config_##n = {                                    \
		DEVICE_MMIO_NAMED_ROM_INIT(buf, DT_DRV_INST(n)),                                   \
		.buf_size = DT_INST_REG_SIZE(n),                                                   \
		.tx = MBOX_DT_SPEC_INST_GET(n, tx),                                                \
		.rx = MBOX_DT_SPEC_INST_GET(n, rx),                                                \
		.size = DT_REG_SIZE(RFLASH_CHILD(n)),                                              \
		.params = {                                                                        \
			.write_block_size = DT_PROP(RFLASH_CHILD(n), write_block_size),            \
			.erase_value = 0xff,                                                       \
		},                                                                                 \
		.layout = {                                                                        \
			.pages_count = DT_REG_SIZE(RFLASH_CHILD(n)) /                              \
				       DT_PROP(RFLASH_CHILD(n), erase_block_size),                 \
			.pages_size = DT_PROP(RFLASH_CHILD(n), erase_block_size),                  \
		},                                                                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, rflash_init, NULL, &rflash_data_##n, &rflash_config_##n,          \
			      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY, &rflash_api);

DT_INST_FOREACH_STATUS_OKAY(RFLASH_DEFINE)
