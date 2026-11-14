/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * RK3399 PCIe in either role, driven from the shell ("pcie ..."). The eMMC
 * is the boot medium and the storage: its FAT partition is mounted at
 * /EMMC: before the PCIe role is read from settings. The micro SD slot is
 * left to the JTAG breakout (no SD card disk); "emmc update <file>" rewrites
 * the eMMC boot area (MBR, idbloader, FIT) from a file, e.g. on /EMMC:.
 * The BootROM tries the SPI NOR first: "fwupdate ymodem" writes
 * <img>-rock_pi_4-spi.bin there over the shell UART, "flash erase" of its
 * first sector falls back to the eMMC.
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
#include <zephyr/sys/sys_io.h>

static FATFS fat_emmc;
static struct fs_mount_t emmc_mnt = {
	.type = FS_FATFS,
	.mnt_point = "/EMMC:",
	.fs_data = &fat_emmc,
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

/*
 * Boot area of a raw image (MBR, idbloader at 64, FIT at 2048) from a file
 * to the eMMC from sector 0. The FAT from BOOT_AREA on, where the file may
 * live, is left alone: a full -emmc.bin is cut there.
 */
#define BOOT_AREA (32768U * SECT)
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
	if (st.size > BOOT_AREA) {
		st.size = BOOT_AREA;
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
	fs_file_t_init(&f);
	ret = fs_open(&f, argv[1], FS_O_READ);
	if (ret != 0) {
		return ret;
	}
	shell_print(sh, "%s: %zu bytes to the eMMC (%u sectors)", argv[1], st.size, count);
	while (done < st.size) {
		ssize_t n = fs_read(&f, buf, MIN(CHUNK, st.size - done));
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
		    ret ? "" : "; reset to boot it");
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_emmc,
	SHELL_CMD_ARG(update, NULL,
		      "<file>: write the boot area (first 16 MiB) of a raw image to the "
		      "eMMC and verify (e.g. /EMMC:/154-rock_pi_4-boot.bin)",
		      cmd_update, 2, 0),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(emmc, &sub_emmc, "eMMC install", NULL);

/*
 * The BAR carveout (pcie_ep_mem) is identity mapped by its DT memory
 * attribute, so devmem, which maps again, panics on it: read and write it
 * here through that mapping. Offsets: PF at 0, VF n at n * 4 MiB.
 */
#define EP_MEM_BASE DT_REG_ADDR(DT_NODELABEL(pcie_ep_mem))
#define EP_MEM_SIZE DT_REG_SIZE(DT_NODELABEL(pcie_ep_mem))

static int cmd_epmem(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long off = strtoul(argv[1], NULL, 0);
	unsigned long n = argc > 2 ? strtoul(argv[2], NULL, 0) : 4;

	if (off >= EP_MEM_SIZE || (off & 3U) != 0U) {
		shell_error(sh, "offset 0x%lx: word aligned, below 0x%lx", off,
			    (unsigned long)EP_MEM_SIZE);
		return -EINVAL;
	}
	if (argc > 3) {
		sys_write32((uint32_t)strtoul(argv[3], NULL, 0), EP_MEM_BASE + off);
	}
	for (unsigned long i = 0; i < n && off + i * 4U < EP_MEM_SIZE; i++) {
		if ((i % 4U) == 0U) {
			shell_fprintf(sh, SHELL_NORMAL, "%s%08lx:", i ? "\n" : "", off + i * 4U);
		}
		shell_fprintf(sh, SHELL_NORMAL, " %08x", sys_read32(EP_MEM_BASE + off + i * 4U));
	}
	shell_print(sh, "");
	return 0;
}
#ifdef CONFIG_RK3399_NOC_ERRLOG
#include <rk3399_noc.h>
#include <zephyr/fatal.h>
#include <zephyr/logging/log_ctrl.h>

/* show what the interconnect logged (SErrors through the PCIe window) */
void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	ARG_UNUSED(esf);
	LOG_PANIC();
	rk3399_noc_errlog_dump();
	printk("Halting system (reason %u)\n", reason);
	(void)arch_irq_lock();
	for (;;) {
	}
}
#endif

SHELL_CMD_ARG_REGISTER(epmem, NULL,
		       "<offset> [words] [value]: BAR carveout (PF 0, VF n at n*4M); value writes "
		       "the first word",
		       cmd_epmem, 2, 2);

int main(void)
{
	enum pcie_role saved;
	bool have = pcie_role_saved(&saved);

	printk("eMMC: %s (%d)\n", emmc_mount_ret == 0 ? "mounted at /EMMC:" : "no FAT",
	       emmc_mount_ret);
	printk("RK3399 PCIe %s (saved: %s, in %s); \"pcie controller mode rc|ep\", "
	       "then a warm reset (\"kernel reboot warm\")\n",
	       pcie_role_get() == PCIE_ROLE_RC ? "root complex" : "endpoint",
	       have ? (saved == PCIE_ROLE_RC ? "rc" : "ep") : "-", pcie_role_storage());
	return 0;
}
