/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * RK3399 PCIe in either role, driven from the shell ("pcie ..."). The eMMC
 * is the boot medium and the storage: its FAT partition is mounted at
 * /EMMC: before the PCIe role is read from settings. The micro SD is only
 * for installing: boot the same image from a card and
 * "emmc update /SD:/<image>-emmc.bin" writes it to the eMMC.
 */

#include <stdlib.h>
#include <string.h>

#include <ff.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pcie/role.h>
#include <zephyr/fs/fs.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/printk.h>

static FATFS fat_emmc, fat_sd;
static struct fs_mount_t emmc_mnt = {
	.type = FS_FATFS,
	.mnt_point = "/EMMC:",
	.fs_data = &fat_emmc,
};
static struct fs_mount_t sd_mnt = {
	.type = FS_FATFS,
	.mnt_point = "/SD:",
	.fs_data = &fat_sd,
};
static int emmc_mount_ret = -ENODEV;

/* after the SDHC hosts (70), the disks (72) and FAT (74); before the PCIe role (81) */
static int emmc_mount(void)
{
	emmc_mount_ret = fs_mount(&emmc_mnt);
	return 0;
}
SYS_INIT(emmc_mount, POST_KERNEL, 76);

#define SECT    512U
#define CHUNK   (64U * SECT)
static uint8_t buf[CHUNK] __aligned(64);
static uint8_t chk[CHUNK] __aligned(64);

/* raw image (MBR, idbloader, FIT, FAT) from a file to the eMMC from sector 0 */
static int cmd_update(const struct shell *sh, size_t argc, char **argv)
{
	struct fs_dirent st;
	struct fs_file_t f;
	uint32_t count = 0, sz = 0;
	size_t done = 0;
	int ret;

	ret = fs_stat(argv[1], &st);
	if (ret != 0) {
		shell_error(sh, "%s: %d", argv[1], ret);
		return ret;
	}
	ret = disk_access_init("EMMC");
	if (ret == 0) {
		ret = disk_access_ioctl("EMMC", DISK_IOCTL_GET_SECTOR_COUNT, &count);
	}
	if (ret == 0) {
		ret = disk_access_ioctl("EMMC", DISK_IOCTL_GET_SECTOR_SIZE, &sz);
	}
	if (ret != 0 || sz != SECT) {
		shell_error(sh, "eMMC not usable (%d, sector %u)", ret, sz);
		return ret ? ret : -EIO;
	}
	if (DIV_ROUND_UP(st.size, SECT) > count) {
		shell_error(sh, "image %zu B > eMMC %u sectors", st.size, count);
		return -EFBIG;
	}
	/* the file system on the eMMC is about to be overwritten */
	if (emmc_mount_ret == 0) {
		(void)fs_unmount(&emmc_mnt);
		emmc_mount_ret = -ENODEV;
	}
	fs_file_t_init(&f);
	ret = fs_open(&f, argv[1], FS_O_READ);
	if (ret != 0) {
		return ret;
	}
	shell_print(sh, "%s: %zu bytes to the eMMC (%u sectors)", argv[1], st.size, count);
	while (done < st.size) {
		ssize_t n = fs_read(&f, buf, CHUNK);
		uint32_t sect, ns;

		if (n <= 0) {
			ret = n < 0 ? (int)n : -EIO;
			break;
		}
		memset(buf + n, 0, CHUNK - n);
		sect = done / SECT;
		ns = DIV_ROUND_UP(n, SECT);
		ret = disk_access_write("EMMC", buf, sect, ns);
		if (ret == 0) {
			ret = disk_access_read("EMMC", chk, sect, ns);
		}
		if (ret == 0 && memcmp(buf, chk, ns * SECT) != 0) {
			shell_error(sh, "verify failed at sector %u", sect);
			ret = -EIO;
		}
		if (ret != 0) {
			break;
		}
		done += n;
		if ((done % MB(1)) == 0) {
			shell_fprintf(sh, SHELL_NORMAL, ".");
		}
	}
	fs_close(&f);
	(void)disk_access_ioctl("EMMC", DISK_IOCTL_CTRL_SYNC, NULL);
	shell_print(sh, "\n%s: %zu bytes written and verified%s", ret ? "FAILED" : "done", done,
		    ret ? "" : "; remove the card (or put the JTAG breakout back) and reset");
	if (ret == 0) {
		emmc_mount_ret = fs_mount(&emmc_mnt);
	}
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_emmc,
	SHELL_CMD_ARG(update, NULL,
		      "<file>: write a raw image to the eMMC from sector 0 and verify "
		      "(e.g. /SD:/153-rp4-emmc.bin)",
		      cmd_update, 2, 0),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(emmc, &sub_emmc, "eMMC install", NULL);

int main(void)
{
	enum pcie_role saved;
	bool have = pcie_role_saved(&saved);
	int ret;

	printk("eMMC: %s (%d)\n", emmc_mount_ret == 0 ? "mounted at /EMMC:" : "no FAT",
	       emmc_mount_ret);
	printk("RK3399 PCIe %s (saved: %s, in %s); \"pcie controller mode rc|ep\", "
	       "then a warm reset (\"kernel reboot warm\")\n",
	       pcie_role_get() == PCIE_ROLE_RC ? "root complex" : "endpoint",
	       have ? (saved == PCIE_ROLE_RC ? "rc" : "ep") : "-", pcie_role_storage());

	ret = fs_mount(&sd_mnt);
	printk("SD: %s (%d)\n", ret == 0 ? "mounted at /SD:" : "no FAT (or JTAG breakout)", ret);
	return 0;
}
