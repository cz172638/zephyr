/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The CM3 owns the SPI NOR. The A53 (flash_mvebu_rwtm) asks for reads,
 * writes and erases outside the firmware area through the rWTM mailbox;
 * the firmware area itself only changes through a whole-image update,
 * which the CM3 checks, programs (TIM header last) and verifies here.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/drivers/mbox/mbox_mvebu_rwtm.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#include <soc.h>

#define RFLASH_NODE DT_NODELABEL(rflash)

#define FW_SIZE     CONFIG_SOC_ARMADA3720_CM3_FLASH_FW_SIZE
#define ERASE_UNIT  4096
#define DDR_WINDOW  0x40000000UL

/* Marvell TIM header: version, then "TIMH" */
#define TIM_MAGIC_OFFSET 4
#define TIM_MAGIC        0x54494d48

static const struct device *const flash = DEVICE_DT_GET(DT_PHANDLE(RFLASH_NODE,
								   zephyr_flash_server));
static const struct mbox_dt_spec tx = MBOX_DT_SPEC_GET(RFLASH_NODE, tx);
static const struct mbox_dt_spec rx = MBOX_DT_SPEC_GET(RFLASH_NODE, rx);
static uint8_t *const shm = (uint8_t *)DT_REG_ADDR(RFLASH_NODE);
#define SHM_SIZE DT_REG_SIZE(RFLASH_NODE)

K_MSGQ_DEFINE(rflash_q, sizeof(struct mvebu_rflash_req), 4, 4);

static void rflash_rx(const struct device *dev, mbox_channel_id_t ch, void *user_data,
		      struct mbox_msg *msg)
{
	struct mvebu_rflash_req req;

	ARG_UNUSED(dev);
	ARG_UNUSED(ch);
	ARG_UNUSED(user_data);

	if (msg == NULL || msg->size < sizeof(req)) {
		return;
	}
	memcpy(&req, msg->data, sizeof(req));
	(void)k_msgq_put(&rflash_q, &req, K_NO_WAIT);
}

static int fw_update(uint32_t addr, uint32_t len, uint32_t *crc)
{
	static uint8_t vbuf[256];
	const uint8_t *img = (const uint8_t *)(A3700_CM3_DDR + addr);
	size_t erase_len = ROUND_UP(len, ERASE_UNIT);
	uint32_t c = 0;
	int ret;

	if (len < TIM_MAGIC_OFFSET + 4 || len > FW_SIZE || addr >= DDR_WINDOW ||
	    len > DDR_WINDOW - addr) {
		return -EINVAL;
	}
	if (sys_get_le32(&img[TIM_MAGIC_OFFSET]) != TIM_MAGIC) {
		printk("CM3 fwupdate: no TIM header\n");
		return -ENOEXEC;
	}

	printk("CM3 fwupdate: %u bytes from 0x%08x, erasing\n", len, addr);
	/* header first: from here on the old image no longer boots */
	ret = flash_erase(flash, 0, ERASE_UNIT);
	if (ret == 0 && erase_len > ERASE_UNIT) {
		ret = flash_erase(flash, ERASE_UNIT, erase_len - ERASE_UNIT);
	}
	if (ret < 0) {
		printk("CM3 fwupdate: erase %d\n", ret);
		return ret;
	}

	printk("CM3 fwupdate: writing\n");
	if (len > ERASE_UNIT) {
		ret = flash_write(flash, ERASE_UNIT, &img[ERASE_UNIT], len - ERASE_UNIT);
	}
	if (ret == 0) {
		/* header last */
		ret = flash_write(flash, 0, img, MIN(len, ERASE_UNIT));
	}
	if (ret < 0) {
		printk("CM3 fwupdate: write %d\n", ret);
		return ret;
	}

	printk("CM3 fwupdate: verifying\n");
	for (uint32_t off = 0; off < len; off += sizeof(vbuf)) {
		size_t n = MIN(sizeof(vbuf), len - off);

		ret = flash_read(flash, off, vbuf, n);
		if (ret < 0 || memcmp(vbuf, &img[off], n) != 0) {
			printk("CM3 fwupdate: verify failed at 0x%x\n", off);
			return ret < 0 ? ret : -EIO;
		}
		c = crc32_ieee_update(c, vbuf, n);
	}

	printk("CM3 fwupdate: done, crc32 %08x\n", c);
	*crc = c;
	return 0;
}

static int rflash_do(const struct mvebu_rflash_req *req, struct mvebu_rflash_rsp *rsp)
{
	uint64_t size;

	switch (req->op) {
	case MVEBU_RFLASH_OP_READ:
		if (req->len > SHM_SIZE) {
			return -EINVAL;
		}
		return flash_read(flash, req->offset, shm, req->len);
	case MVEBU_RFLASH_OP_WRITE:
		if (req->len > SHM_SIZE) {
			return -EINVAL;
		}
		if (req->offset < FW_SIZE) {
			return -EACCES;
		}
		return flash_write(flash, req->offset, shm, req->len);
	case MVEBU_RFLASH_OP_ERASE:
		if (req->offset < FW_SIZE) {
			return -EACCES;
		}
		return flash_erase(flash, req->offset, req->len);
	case MVEBU_RFLASH_OP_INFO:
		(void)flash_get_size(flash, &size);
		rsp->val[0] = (uint32_t)size;
		rsp->val[1] = ERASE_UNIT;
		return 0;
	case MVEBU_RFLASH_OP_FWUPDATE:
		return fw_update(req->offset, req->len, &rsp->val[0]);
	default:
		return -ENOTSUP;
	}
}

static void rflash_thread(void *a, void *b, void *c)
{
	struct mvebu_rflash_req req;
	struct mvebu_rflash_rsp rsp;
	struct mbox_msg msg = {
		.data = &rsp,
		.size = sizeof(rsp),
	};

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	if (!device_is_ready(flash)) {
		printk("CM3 flash: %s not ready\n", flash->name);
		return;
	}
	if (mbox_register_callback_dt(&rx, rflash_rx, NULL) < 0 ||
	    mbox_set_enabled_dt(&rx, true) < 0) {
		printk("CM3 flash: mailbox\n");
		return;
	}

	for (;;) {
		k_msgq_get(&rflash_q, &req, K_FOREVER);
		memset(&rsp, 0, sizeof(rsp));
		rsp.seq = req.seq;
		rsp.ret = rflash_do(&req, &rsp);
		(void)mbox_send_dt(&tx, &msg);
	}
}

K_THREAD_DEFINE(cm3_flash, 1024, rflash_thread, NULL, NULL, NULL, 4, 0, 0);
