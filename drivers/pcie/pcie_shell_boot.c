/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * "pcie boot <file>...": boot an endpoint over PCIe from the root complex.
 * The endpoint side is U-Boot SPL with CONFIG_SPL_PCI_DFU: it offers a
 * memory BAR0 that ends with the name of the image it waits for (63
 * bytes before the last dword) and goes on once the root complex has
 * written the image to the start of BAR0 and the magic word to its last
 * dword. Each stage stops the endpoint and the next one starts it again,
 * so the command enumerates until the endpoint is back and sends the
 * file named by its boot phase, or the next one in order.
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/pcie/pcie.h>
#include <zephyr/drivers/pcie/pcie_shell.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "pcie_shell_priv.h"

#define BOOT_MAGIC      0xdeadbeefU /* U-Boot CONFIG_SPL_PCI_DFU_MAGIC_WORD default */
#define BOOT_PHASE_LEN  63U
#define BOOT_MAX_FILES  4
#define BOOT_BUSES      16U
#define BOOT_POLL_MS    200
#define BOOT_CHUNK      4096U

static uint8_t boot_buf[BOOT_CHUNK] __aligned(4);

/* first function with this vendor:device below the root port, or -ENODEV */
static int boot_find(const struct pcie_shell_ctrl *c, uint32_t id, pcie_bdf_t *bdf)
{
	for (unsigned int b = 1; b < BOOT_BUSES; b++) {
		for (unsigned int d = 0; d < 32U; d++) {
			if (c->cfg_read(c->dev, PCIE_BDF(b, d, 0), 0) == id) {
				*bdf = PCIE_BDF(b, d, 0);
				return 0;
			}
		}
	}
	return -ENODEV;
}

struct boot_ep {
	pcie_bdf_t bdf;
	struct pcie_bar bar;
	char phase[BOOT_PHASE_LEN + 1];
};

/* the boot phase string of the endpoint: 63 bytes before the last dword of BAR0 */
static void boot_phase(const struct boot_ep *ep, char *phase)
{
	mm_reg_t bar;

	device_map(&bar, ep->bar.phys_addr, ep->bar.size, K_MEM_CACHE_NONE);
	for (size_t i = 0; i < BOOT_PHASE_LEN; i++) {
		phase[i] = (char)sys_read8(bar + ep->bar.size - 4U - BOOT_PHASE_LEN - 1U + i);
	}
	phase[BOOT_PHASE_LEN] = '\0';
	device_unmap(bar, ep->bar.size);
}

/*
 * Enumerate (retraining the link when it is down) until the endpoint
 * answers with a BAR0 and a boot phase other than the one just served:
 * a stage leaves the endpoint up until the next one sets it up again.
 */
static int boot_wait(const struct shell *sh, const struct pcie_shell_ctrl *c, uint32_t id,
		     int32_t timeout_ms, const char *served, struct boot_ep *ep)
{
	int64_t t0 = k_uptime_get();

	while (k_uptime_get() - t0 < timeout_ms) {
		if (c->scan != NULL && c->scan(c->dev, sh) == -ENOTCONN && c->link != NULL) {
			(void)c->link(c->dev, sh, true);
		}
		if (boot_find(c, id, &ep->bdf) == 0) {
			pcie_set_cmd(ep->bdf, PCIE_CONF_CMDSTAT_MEM, true);
			if (pcie_get_mbar(ep->bdf, 0, &ep->bar) && ep->bar.size >= 128U) {
				boot_phase(ep, ep->phase);
				if (strcmp(ep->phase, served) != 0) {
					return 0;
				}
			}
		}
		k_msleep(BOOT_POLL_MS);
	}
	return -ETIMEDOUT;
}

static int boot_copy(const struct shell *sh, const char *path, mm_reg_t bar, size_t room)
{
	struct fs_dirent st;
	struct fs_file_t f;
	size_t done = 0;
	ssize_t n;
	int ret;

	ret = fs_stat(path, &st);
	if (ret != 0) {
		shell_error(sh, "%s: %d", path, ret);
		return ret;
	}
	if (st.size > room) {
		shell_error(sh, "%s: %zu bytes, BAR0 takes %zu", path, st.size, room);
		return -EFBIG;
	}
	fs_file_t_init(&f);
	ret = fs_open(&f, path, FS_O_READ);
	if (ret != 0) {
		shell_error(sh, "%s: %d", path, ret);
		return ret;
	}
	/* 32-bit stores: the BAR is mapped as device memory */
	while ((n = fs_read(&f, boot_buf, sizeof(boot_buf))) > 0) {
		memset(boot_buf + n, 0, ROUND_UP(n, 4) - n);
		for (size_t i = 0; i < (size_t)n; i += 4U) {
			sys_write32(sys_get_le32(&boot_buf[i]), bar + done + i);
		}
		done += (size_t)n;
	}
	(void)fs_close(&f);
	if (n < 0) {
		shell_error(sh, "%s: read %d", path, (int)n);
		return (int)n;
	}
	shell_print(sh, "  %s: %zu bytes", path, done);
	return 0;
}

static const char *base_name(const char *path)
{
	const char *s = strrchr(path, '/');
	const char *t = strrchr(path, ':');

	s = MAX(s, t);
	return s != NULL ? s + 1 : path;
}

int pcie_shell_cmd_boot(const struct shell *sh, size_t argc, char **argv)
{
	const struct pcie_shell_ctrl *c = pcie_shell_active(sh);
	uint32_t id = 0xb010104cU; /* TI J722S/AM64 */
	int32_t timeout_ms = 30000;
	const char *files[BOOT_MAX_FILES];
	bool sent[BOOT_MAX_FILES] = {false};
	char served[BOOT_PHASE_LEN + 1] = "";
	int nfiles = 0, left;

	if (c == NULL) {
		return -ENODEV;
	}
	if (c->role != PCIE_SHELL_ROLE_RC) {
		shell_error(sh, "boot needs the controller in root complex mode");
		return -ENOTSUP;
	}
	for (size_t i = 1; i < argc; i++) {
		uint16_t vid, did;

		if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
			if (pcie_shell_parse_pair(argv[++i], &vid, &did) != 0) {
				shell_error(sh, "-i <vid:did>");
				return -EINVAL;
			}
			id = ((uint32_t)did << 16) | vid;
		} else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
			timeout_ms = (int32_t)strtol(argv[++i], NULL, 0) * 1000;
		} else if (nfiles < BOOT_MAX_FILES) {
			files[nfiles++] = argv[i];
		}
	}
	if (nfiles == 0) {
		shell_error(sh, "pcie boot [-i vid:did] [-t s] <file>...");
		return -EINVAL;
	}

	for (left = nfiles; left > 0; left--) {
		struct boot_ep ep;
		mm_reg_t bar;
		int pick = -1, ret;

		shell_print(sh, "waiting for %04x:%04x (%d s)", id & 0xffffU, id >> 16,
			    timeout_ms / 1000);
		ret = boot_wait(sh, c, id, timeout_ms, served, &ep);
		if (ret != 0) {
			shell_error(sh, "no endpoint waiting for an image");
			return ret;
		}
		for (int i = 0; i < nfiles && pick < 0; i++) {
			if (!sent[i] && strcmp(base_name(files[i]), ep.phase) == 0) {
				pick = i;
			}
		}
		for (int i = 0; i < nfiles && pick < 0; i++) {
			if (!sent[i]) {
				pick = i;
			}
		}
		shell_print(sh, "%x:%x.%x BAR0 0x%lx (%zu KiB) wants \"%s\"",
			    PCIE_BDF_TO_BUS(ep.bdf), PCIE_BDF_TO_DEV(ep.bdf),
			    PCIE_BDF_TO_FUNC(ep.bdf), (unsigned long)ep.bar.phys_addr,
			    ep.bar.size / 1024U, ep.phase);
		device_map(&bar, ep.bar.phys_addr, ep.bar.size, K_MEM_CACHE_NONE);
		ret = boot_copy(sh, files[pick], bar, ep.bar.size - 4U - BOOT_PHASE_LEN - 1U);
		if (ret == 0) {
			sys_write32(BOOT_MAGIC, bar + ep.bar.size - 4U);
			(void)sys_read32(bar + ep.bar.size - 4U); /* the writes have arrived */
			sent[pick] = true;
			strcpy(served, ep.phase);
		}
		device_unmap(bar, ep.bar.size);
		if (ret != 0) {
			return ret;
		}
	}
	shell_print(sh, "done");
	return 0;
}
