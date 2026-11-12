/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * RK3399 PCIe in either role, driven from the shell ("pcie ..."), with the
 * micro SD mounted at /SD: and "spiflash update <file>" to put an image
 * from the card into the SPI flash.
 */

#include <stdlib.h>

#include <ff.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/pcie/role.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>

#define SPI_FLASH DT_NODELABEL(spi_flash)

static FATFS fat;
static struct fs_mount_t sd_mnt = {
	.type = FS_FATFS,
	.mnt_point = "/SD:",
	.fs_data = &fat,
};

#if DT_NODE_HAS_STATUS_OKAY(SPI_FLASH)
static uint8_t blk[4096];
static uint8_t chk[4096];

static int cmd_update(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *fl = DEVICE_DT_GET(SPI_FLASH);
	off_t off = argc > 2 ? strtoul(argv[2], NULL, 0) : 0;
	struct fs_file_t f;
	struct fs_dirent st;
	size_t done = 0;
	int ret;

	if (!device_is_ready(fl)) {
		shell_error(sh, "no SPI flash on this board");
		return -ENODEV;
	}
	if ((off % sizeof(blk)) != 0) {
		shell_error(sh, "offset must be 4 KiB aligned");
		return -EINVAL;
	}
	ret = fs_stat(argv[1], &st);
	if (ret != 0) {
		shell_error(sh, "%s: %d", argv[1], ret);
		return ret;
	}
	fs_file_t_init(&f);
	ret = fs_open(&f, argv[1], FS_O_READ);
	if (ret != 0) {
		return ret;
	}
	shell_print(sh, "%s: %zu bytes to flash offset 0x%lx", argv[1], st.size, (long)off);
	while (done < st.size) {
		ssize_t n = fs_read(&f, blk, sizeof(blk));

		if (n <= 0) {
			ret = n < 0 ? (int)n : -EIO;
			break;
		}
		memset(blk + n, 0xff, sizeof(blk) - n);
		ret = flash_erase(fl, off + done, sizeof(blk));
		if (ret == 0) {
			ret = flash_write(fl, off + done, blk, sizeof(blk));
		}
		if (ret == 0) {
			ret = flash_read(fl, off + done, chk, sizeof(blk));
		}
		if (ret == 0 && memcmp(blk, chk, sizeof(blk)) != 0) {
			shell_error(sh, "verify failed at 0x%lx", (long)(off + done));
			ret = -EIO;
		}
		if (ret != 0) {
			break;
		}
		done += n;
		if ((done % (64 * 1024)) == 0) {
			shell_fprintf(sh, SHELL_NORMAL, ".");
		}
	}
	fs_close(&f);
	shell_print(sh, "\n%s: %zu bytes written and verified", ret ? "FAILED" : "done", done);
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_spiflash,
	SHELL_CMD_ARG(update, NULL,
		      "<file> [offset]: erase, write and verify (e.g. /SD:/151-rp4-pcie-spi.bin)",
		      cmd_update, 2, 1),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(spiflash, &sub_spiflash, "SPI flash from SD", NULL);
#endif

int main(void)
{
	enum pcie_role saved;
	bool have = pcie_role_saved(&saved);
	int ret;

	printk("RK3399 PCIe %s (saved: %s, in %s); \"pcie controller mode rc|ep\", "
	       "then a warm reset (\"kernel reboot warm\")\n",
	       pcie_role_get() == PCIE_ROLE_RC ? "root complex" : "endpoint",
	       have ? (saved == PCIE_ROLE_RC ? "rc" : "ep") : "-", pcie_role_storage());

	ret = fs_mount(&sd_mnt);
	printk("SD: %s (%d)\n", ret == 0 ? "mounted at /SD:" : "no FAT", ret);
	return 0;
}
