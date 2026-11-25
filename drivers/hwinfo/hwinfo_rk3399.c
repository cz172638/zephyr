/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * RK3399 device ID: the CPU ID in the non-secure eFuse (bytes 7..22), read
 * word by word in the read mode of the eFuse macro (as U-Boot's
 * rockchip-efuse driver does).
 */

#define DT_DRV_COMPAT rockchip_rk3399_efuse

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#define EFUSE_CTRL        0x00U
#define  CTRL_A_SHIFT     16U
#define  CTRL_A_MASK      0x3ffU
#define  CTRL_STROBSFTSEL BIT(9)
#define  CTRL_RSB         BIT(7)
#define  CTRL_PD          BIT(5)
#define  CTRL_PGENB       BIT(3)
#define  CTRL_LOAD        BIT(2)
#define  CTRL_STROBE      BIT(1)
#define  CTRL_CSB         BIT(0)
#define EFUSE_DOUT        0x04U

#define CPUID_OFFSET      7U
#define CPUID_LEN         16U
#define EFUSE_WORDS       32U

static struct k_spinlock lock;

static int efuse_read(uint8_t *buf, size_t offset, size_t len)
{
	const struct device *clk = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0));
	uint32_t first = offset / 4U;
	uint32_t last = (offset + len + 3U) / 4U;
	uint32_t words[EFUSE_WORDS];
	k_spinlock_key_t key;
	mm_reg_t base;
	int ret;

	if (last > EFUSE_WORDS) {
		return -EINVAL;
	}
	if (!device_is_ready(clk)) {
		return -ENODEV;
	}
	ret = clock_control_on(clk, (clock_control_subsys_t)DT_INST_CLOCKS_CELL(0, clkid));
	if (ret != 0) {
		return ret;
	}
	device_map(&base, DT_INST_REG_ADDR(0), DT_INST_REG_SIZE(0), K_MEM_CACHE_NONE);

	key = k_spin_lock(&lock);
	sys_write32(CTRL_LOAD | CTRL_PGENB | CTRL_STROBSFTSEL | CTRL_RSB, base + EFUSE_CTRL);
	k_busy_wait(1);
	for (uint32_t a = first; a < last; a++) {
		uint32_t v = sys_read32(base + EFUSE_CTRL);

		v &= ~(CTRL_A_MASK << CTRL_A_SHIFT);
		sys_write32(v | (a << CTRL_A_SHIFT), base + EFUSE_CTRL);
		k_busy_wait(1);
		sys_write32(v | (a << CTRL_A_SHIFT) | CTRL_STROBE, base + EFUSE_CTRL);
		k_busy_wait(1);
		words[a] = sys_read32(base + EFUSE_DOUT);
		sys_write32(v | (a << CTRL_A_SHIFT), base + EFUSE_CTRL);
		k_busy_wait(1);
	}
	sys_write32(CTRL_PD | CTRL_CSB, base + EFUSE_CTRL);
	k_spin_unlock(&lock, key);

	memcpy(buf, (uint8_t *)words + offset, len);
	return 0;
}

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	uint8_t id[CPUID_LEN];
	bool blank = true;
	int ret;

	ret = efuse_read(id, CPUID_OFFSET, sizeof(id));
	if (ret != 0) {
		return ret;
	}
	for (size_t i = 0; i < sizeof(id); i++) {
		if (id[i] != 0U && id[i] != 0xffU) {
			blank = false;
		}
	}
	if (blank) {
		return -EIO;
	}
	length = MIN(length, sizeof(id));
	memcpy(buffer, id, length);
	return length;
}
