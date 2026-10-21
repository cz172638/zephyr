/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Marvell Armada 3700 rWTM mailbox, both ends.
 *
 * Host (A53): 16 argument registers and a command register, a 4 deep
 * command FIFO; the secure processor answers with a status and 16
 * arguments and raises the host interrupt (Linux
 * armada-37xx-rwtm-mailbox.c). Secure processor (CM3): the same block at
 * 0x40000000, command interrupt on NVIC line 2 (CZ.NIC mox-boot-builder
 * wtmi/mbox.c).
 *
 * Channel n travels as command 0x200 + n, with the length in argument 0;
 * a NULL message is a doorbell (length 0).
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/drivers/mbox/mbox_mvebu_rwtm.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mbox_mvebu_rwtm, CONFIG_MBOX_LOG_LEVEL);

/* Common to both views */
#define RWTM_IN_ARG(i)    (0x00 + 4 * (i))
#define RWTM_IN_CMD       0x40
#define RWTM_OUT_STATUS   0x80
#define RWTM_OUT_ARG(i)   (0x84 + 4 * (i))
#define RWTM_FIFO_STATUS  0xc4
#define FIFO_STS_RDY      BIT(8)
#define FIFO_STS_CNTR     GENMASK(2, 0)
#define FIFO_STS_CNTR_MAX 4

/* Host view */
#define RWTM_HOST_INT_RESET 0xc8
#define RWTM_HOST_INT_MASK  0xcc

/* Secure processor view */
#define RWTM_SP_INT_RESET  0x218
#define RWTM_SP_INT_MASK   0x21c
#define RWTM_SP_CONTROL    0x220
#define RWTM_HOST_INT_SET  0x234
#define SP_CMD_OCCUPIED       BIT(0)
#define SP_CMD_OCCUPIED_RESET BIT(1)

/* Host interrupt bits */
#define HOST_CMD_COMPLETE          BIT(0)
#define HOST_CMD_QUEUE_FULL_ACCESS BIT(17)
#define HOST_CMD_QUEUE_FULL        BIT(18)
#define HOST_INT_ALL (HOST_CMD_COMPLETE | HOST_CMD_QUEUE_FULL_ACCESS | HOST_CMD_QUEUE_FULL)

#define RWTM_TIMEOUT_US 20000

struct rwtm_config {
	DEVICE_MMIO_NAMED_ROM(base);
	bool sp;
	void (*irq_config)(void);
};

struct rwtm_data {
	DEVICE_MMIO_NAMED_RAM(base);
	struct k_spinlock lock;
	mbox_callback_t cb[MVEBU_RWTM_CHANNELS];
	void *user_data[MVEBU_RWTM_CHANNELS];
	uint32_t enabled;
	uint32_t rx_count;
	uint32_t tx_count;
	uint32_t errors;
};

#define DEV_CFG(dev)  ((const struct rwtm_config *)(dev)->config)
#define DEV_DATA(dev) ((struct rwtm_data *)(dev)->data)

static inline uint32_t rwtm_rd(const struct device *dev, uint32_t off)
{
	return sys_read32(DEVICE_MMIO_NAMED_GET(dev, base) + off);
}

static inline void rwtm_wr(const struct device *dev, uint32_t off, uint32_t val)
{
	sys_write32(val, DEVICE_MMIO_NAMED_GET(dev, base) + off);
}

__weak uint32_t mvebu_rwtm_sp_legacy_cmd(uint16_t cmd, const uint32_t args[MVEBU_RWTM_NARGS],
					 uint32_t out[MVEBU_RWTM_NARGS])
{
	ARG_UNUSED(args);
	ARG_UNUSED(out);

	return MVEBU_RWTM_STS(cmd, 0, BADCMD);
}

static void rwtm_deliver(const struct device *dev, uint32_t cmd, const uint32_t *args)
{
	struct rwtm_data *data = dev->data;
	uint32_t ch = cmd - MVEBU_RWTM_CMD_CHAN;
	uint32_t len = MIN(args[0], MVEBU_RWTM_MTU);
	struct mbox_msg msg = {
		.data = &args[1],
		.size = len,
	};

	if (ch >= MVEBU_RWTM_CHANNELS) {
		data->errors++;
		return;
	}

	data->rx_count++;
	if ((data->enabled & BIT(ch)) != 0 && data->cb[ch] != NULL) {
		data->cb[ch](dev, ch, data->user_data[ch], &msg);
	}
}

/* Secure processor -> host: status and arguments, then the host interrupt */
static int rwtm_sp_reply(const struct device *dev, uint32_t status, const uint32_t *args,
			 size_t nargs)
{
	struct rwtm_data *data = dev->data;
	k_spinlock_key_t key;
	int wait = RWTM_TIMEOUT_US;

	/* the host has not taken the previous one yet */
	for (;;) {
		key = k_spin_lock(&data->lock);
		if ((rwtm_rd(dev, RWTM_HOST_INT_SET) & HOST_CMD_COMPLETE) == 0) {
			break;
		}
		k_spin_unlock(&data->lock, key);
		if (--wait <= 0) {
			data->errors++;
			return -EBUSY;
		}
		k_busy_wait(1);
	}

	for (size_t i = 0; i < MVEBU_RWTM_NARGS; i++) {
		rwtm_wr(dev, RWTM_OUT_ARG(i), i < nargs ? args[i] : 0);
	}
	rwtm_wr(dev, RWTM_OUT_STATUS, status);
	rwtm_wr(dev, RWTM_HOST_INT_SET, rwtm_rd(dev, RWTM_HOST_INT_SET) | HOST_CMD_COMPLETE);
	data->tx_count++;
	k_spin_unlock(&data->lock, key);

	return 0;
}

/* Host -> secure processor: arguments, then the command (into the FIFO) */
static int rwtm_host_cmd(const struct device *dev, uint32_t cmd, const uint32_t *args,
			 size_t nargs)
{
	struct rwtm_data *data = dev->data;
	k_spinlock_key_t key;
	int wait = RWTM_TIMEOUT_US;

	for (;;) {
		key = k_spin_lock(&data->lock);
		if ((rwtm_rd(dev, RWTM_FIFO_STATUS) & FIFO_STS_CNTR) < FIFO_STS_CNTR_MAX) {
			break;
		}
		k_spin_unlock(&data->lock, key);
		if (--wait <= 0) {
			data->errors++;
			return -EBUSY;
		}
		k_busy_wait(1);
	}

	for (size_t i = 0; i < MVEBU_RWTM_NARGS; i++) {
		rwtm_wr(dev, RWTM_IN_ARG(i), i < nargs ? args[i] : 0);
	}
	rwtm_wr(dev, RWTM_IN_CMD, cmd);
	data->tx_count++;
	k_spin_unlock(&data->lock, key);

	return 0;
}

static void rwtm_sp_isr(const struct device *dev)
{
	uint32_t args[MVEBU_RWTM_NARGS];
	uint32_t out[MVEBU_RWTM_NARGS];
	uint32_t cmd, status;
	int budget = FIFO_STS_CNTR_MAX + 1;

	while (budget-- > 0 && (rwtm_rd(dev, RWTM_SP_CONTROL) & SP_CMD_OCCUPIED) != 0) {
		cmd = rwtm_rd(dev, RWTM_IN_CMD) & 0xffff;
		for (int i = 0; i < MVEBU_RWTM_NARGS; i++) {
			args[i] = rwtm_rd(dev, RWTM_IN_ARG(i));
		}

		/* release the command register before the handlers run */
		rwtm_wr(dev, RWTM_SP_INT_RESET,
			rwtm_rd(dev, RWTM_SP_INT_RESET) | SP_CMD_OCCUPIED_RESET);
		rwtm_wr(dev, RWTM_SP_CONTROL, rwtm_rd(dev, RWTM_SP_CONTROL) & ~SP_CMD_OCCUPIED);

		if (cmd >= MVEBU_RWTM_CMD_CHAN) {
			rwtm_deliver(dev, cmd, args);
			continue;
		}

		memset(out, 0, sizeof(out));
		status = mvebu_rwtm_sp_legacy_cmd(cmd, args, out);
		if (MVEBU_RWTM_STS_CMD(status) == 0) {
			status |= cmd;
		}
		if (MVEBU_RWTM_STS_ERROR(status) != MVEBU_RWTM_STS_LATER) {
			(void)rwtm_sp_reply(dev, status, out, MVEBU_RWTM_NARGS);
		}
	}
}

static void rwtm_host_isr(const struct device *dev)
{
	struct rwtm_data *data = dev->data;
	uint32_t args[MVEBU_RWTM_NARGS];
	uint32_t reg = rwtm_rd(dev, RWTM_HOST_INT_RESET);
	uint32_t status;

	if ((reg & HOST_CMD_COMPLETE) != 0) {
		status = rwtm_rd(dev, RWTM_OUT_STATUS);
		for (int i = 0; i < MVEBU_RWTM_NARGS; i++) {
			args[i] = rwtm_rd(dev, RWTM_OUT_ARG(i));
		}
		/* acknowledge first: the secure processor may send the next */
		rwtm_wr(dev, RWTM_HOST_INT_RESET, reg);

		if (MVEBU_RWTM_STS_CMD(status) >= MVEBU_RWTM_CMD_CHAN) {
			rwtm_deliver(dev, MVEBU_RWTM_STS_CMD(status), args);
		} else {
			LOG_DBG("WTMI reply 0x%08x", status);
		}
		reg &= ~HOST_CMD_COMPLETE;
	}

	if ((reg & (HOST_CMD_QUEUE_FULL_ACCESS | HOST_CMD_QUEUE_FULL)) != 0) {
		data->errors++;
		rwtm_wr(dev, RWTM_HOST_INT_RESET, reg);
	}
}

static void rwtm_isr(const struct device *dev)
{
	const struct rwtm_config *cfg = dev->config;

	if (cfg->sp) {
		rwtm_sp_isr(dev);
	} else {
		rwtm_host_isr(dev);
	}
}

static int rwtm_send(const struct device *dev, mbox_channel_id_t ch, const struct mbox_msg *msg)
{
	const struct rwtm_config *cfg = dev->config;
	uint32_t args[MVEBU_RWTM_NARGS] = {0};
	size_t len = msg != NULL ? msg->size : 0;

	if (ch >= MVEBU_RWTM_CHANNELS) {
		return -EINVAL;
	}
	if (len > MVEBU_RWTM_MTU) {
		return -EMSGSIZE;
	}

	args[0] = len;
	if (len != 0) {
		memcpy(&args[1], msg->data, len);
	}

	if (cfg->sp) {
		return rwtm_sp_reply(dev,
				     MVEBU_RWTM_STS(MVEBU_RWTM_CMD_CHAN + ch, len, SUCCESS),
				     args, 1 + DIV_ROUND_UP(len, 4));
	}
	return rwtm_host_cmd(dev, MVEBU_RWTM_CMD_CHAN + ch, args, 1 + DIV_ROUND_UP(len, 4));
}

static int rwtm_register_callback(const struct device *dev, mbox_channel_id_t ch,
				  mbox_callback_t cb, void *user_data)
{
	struct rwtm_data *data = dev->data;
	k_spinlock_key_t key;

	if (ch >= MVEBU_RWTM_CHANNELS) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	data->cb[ch] = cb;
	data->user_data[ch] = user_data;
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int rwtm_mtu_get(const struct device *dev)
{
	ARG_UNUSED(dev);

	return MVEBU_RWTM_MTU;
}

static uint32_t rwtm_max_channels_get(const struct device *dev)
{
	ARG_UNUSED(dev);

	return MVEBU_RWTM_CHANNELS;
}

static int rwtm_set_enabled(const struct device *dev, mbox_channel_id_t ch, bool enable)
{
	struct rwtm_data *data = dev->data;
	k_spinlock_key_t key;

	if (ch >= MVEBU_RWTM_CHANNELS) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	if (enable) {
		data->enabled |= BIT(ch);
	} else {
		data->enabled &= ~BIT(ch);
	}
	k_spin_unlock(&data->lock, key);

	return 0;
}

static DEVICE_API(mbox, rwtm_api) = {
	.send = rwtm_send,
	.register_callback = rwtm_register_callback,
	.mtu_get = rwtm_mtu_get,
	.max_channels_get = rwtm_max_channels_get,
	.set_enabled = rwtm_set_enabled,
};

static int rwtm_init(const struct device *dev)
{
	const struct rwtm_config *cfg = dev->config;

	DEVICE_MMIO_NAMED_MAP(dev, base, K_MEM_CACHE_NONE);

	if (cfg->sp) {
		/* as WTMI mbox_init(): unmask the command, mark the FIFO ready */
		rwtm_wr(dev, RWTM_SP_INT_MASK,
			rwtm_rd(dev, RWTM_SP_INT_MASK) & ~SP_CMD_OCCUPIED_RESET);
		rwtm_wr(dev, RWTM_FIFO_STATUS, FIFO_STS_RDY);
	} else {
		/* the BootROM start message or a stale reply */
		rwtm_wr(dev, RWTM_HOST_INT_RESET, rwtm_rd(dev, RWTM_HOST_INT_RESET));
		rwtm_wr(dev, RWTM_HOST_INT_MASK, rwtm_rd(dev, RWTM_HOST_INT_MASK) & ~HOST_INT_ALL);
	}

	cfg->irq_config();

	return 0;
}

#define RWTM_DEFINE(inst, compat, is_sp)                                                           \
	static void rwtm_irq_config_##compat##inst(void)                                           \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQN(DT_INST(inst, compat)),                                        \
			    DT_IRQ_BY_IDX(DT_INST(inst, compat), 0, priority), rwtm_isr,           \
			    DEVICE_DT_GET(DT_INST(inst, compat)),                                  \
			    COND_CODE_1(DT_IRQ_HAS_CELL(DT_INST(inst, compat), flags),             \
					(DT_IRQ(DT_INST(inst, compat), flags)), (0)));             \
		irq_enable(DT_IRQN(DT_INST(inst, compat)));                                        \
	}                                                                                          \
	static struct rwtm_data rwtm_data_##compat##inst;                                          \
	static const struct rwtm_config rwtm_config_##compat##inst = {                            \
		DEVICE_MMIO_NAMED_ROM_INIT(base, DT_INST(inst, compat)),                           \
		.sp = is_sp,                                                                       \
		.irq_config = rwtm_irq_config_##compat##inst,                                      \
	};                                                                                         \
	DEVICE_DT_DEFINE(DT_INST(inst, compat), rwtm_init, NULL, &rwtm_data_##compat##inst,        \
			 &rwtm_config_##compat##inst, PRE_KERNEL_1, CONFIG_MBOX_INIT_PRIORITY,     \
			 &rwtm_api);

#define RWTM_HOST(inst) RWTM_DEFINE(inst, marvell_armada_3700_rwtm_mailbox, false)
#define RWTM_SP(inst)   RWTM_DEFINE(inst, marvell_armada_3700_rwtm_sp, true)

#define DT_DRV_COMPAT marvell_armada_3700_rwtm_mailbox
DT_INST_FOREACH_STATUS_OKAY(RWTM_HOST)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT marvell_armada_3700_rwtm_sp
DT_INST_FOREACH_STATUS_OKAY(RWTM_SP)
#undef DT_DRV_COMPAT
