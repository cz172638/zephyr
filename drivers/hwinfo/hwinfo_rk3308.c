/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * RK3308 device ID: the 16-byte CPU ID at byte 7 of the OTP, read byte by
 * byte through the user port with ECC off (as U-Boot's rockchip-otp driver
 * does for px30/rk3308). The OTP clocks are on after reset and left so by
 * the boot loaders.
 */

#define DT_DRV_COMPAT rockchip_rk3308_otp

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#define OTPC_SBPI_CTRL          0x0020U
#define OTPC_SBPI_CMD_VALID_PRE 0x0024U
#define OTPC_USER_CTRL          0x0100U
#define OTPC_USER_ADDR          0x0104U
#define OTPC_USER_ENABLE        0x0108U
#define OTPC_USER_Q             0x0124U
#define OTPC_INT_STATUS         0x0304U
#define OTPC_SBPI_CMD0          0x1000U
#define OTPC_SBPI_CMD1          0x1004U

/* upper halfword: write enable of the lower one */
#define WMSK(bits) ((uint32_t)(bits) << 16)

#define OTPC_USE_USER        BIT(0)
#define OTPC_USER_FSM_ENABLE BIT(0)
#define OTPC_SBPI_DONE       BIT(1)
#define OTPC_USER_DONE       BIT(2)

#define SBPI_DAP_ADDR      0x02U
#define SBPI_DAP_ADDR_SHIFT 8U
#define SBPI_DAP_CMD_WRF   0xc0U
#define SBPI_DAP_REG_ECC   0x3aU
#define SBPI_ECC_DISABLE   0x09U
#define SBPI_ENABLE        BIT(0)

#define OTPC_TIMEOUT_US 10000U

#define CPUID_OFFSET 7U
#define CPUID_LEN    16U

static struct k_spinlock lock;

static int otp_wait(mm_reg_t base, uint32_t flag)
{
	for (uint32_t us = 0U; us < OTPC_TIMEOUT_US; us++) {
		if ((sys_read32(base + OTPC_INT_STATUS) & flag) != 0U) {
			sys_write32(flag, base + OTPC_INT_STATUS);
			return 0;
		}
		k_busy_wait(1);
	}
	return -ETIMEDOUT;
}

static int otp_read(uint8_t *buf, uint32_t offset, size_t len)
{
	k_spinlock_key_t key;
	mm_reg_t base;
	int ret;

	device_map(&base, DT_INST_REG_ADDR(0), DT_INST_REG_SIZE(0), K_MEM_CACHE_NONE);

	key = k_spin_lock(&lock);

	/* ECC off: one SBPI write of the ECC register of the OTP macro */
	sys_write32(GENMASK(31, 24) | (SBPI_DAP_ADDR << SBPI_DAP_ADDR_SHIFT),
		    base + OTPC_SBPI_CTRL);
	sys_write32(GENMASK(31, 16) | 0x1U, base + OTPC_SBPI_CMD_VALID_PRE);
	sys_write32(SBPI_DAP_CMD_WRF | SBPI_DAP_REG_ECC, base + OTPC_SBPI_CMD0);
	sys_write32(SBPI_ECC_DISABLE, base + OTPC_SBPI_CMD1);
	sys_write32(WMSK(SBPI_ENABLE) | SBPI_ENABLE, base + OTPC_SBPI_CTRL);
	ret = otp_wait(base, OTPC_SBPI_DONE);
	if (ret != 0) {
		goto out;
	}

	sys_write32(WMSK(OTPC_USE_USER) | OTPC_USE_USER, base + OTPC_USER_CTRL);
	k_busy_wait(5);
	for (size_t i = 0; i < len; i++) {
		sys_write32(GENMASK(31, 16) | (offset + i), base + OTPC_USER_ADDR);
		sys_write32(WMSK(OTPC_USER_FSM_ENABLE) | OTPC_USER_FSM_ENABLE,
			    base + OTPC_USER_ENABLE);
		ret = otp_wait(base, OTPC_USER_DONE);
		if (ret != 0) {
			break;
		}
		buf[i] = (uint8_t)sys_read32(base + OTPC_USER_Q);
	}
	sys_write32(WMSK(OTPC_USE_USER), base + OTPC_USER_CTRL);
out:
	k_spin_unlock(&lock, key);
	device_unmap(base, DT_INST_REG_SIZE(0));
	return ret;
}

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	uint8_t id[CPUID_LEN];
	bool blank = true;
	int ret;

	ret = otp_read(id, CPUID_OFFSET, sizeof(id));
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
