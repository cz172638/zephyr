/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * "fwupdate ymodem [dry]": receive a firmware image over the shell UART
 * with YMODEM (e.g. tio ctrl-t y, or sb), check it and write it into the
 * firmware flash partition.
 *
 * "fwupdate xmodem [dry]": the same with XMODEM / XMODEM-1K (e.g. sx);
 * the image arrives padded to whole blocks and is written with the padding.
 *
 * "fwupdate tcp [port] [dry]": the same over one TCP connection, e.g.
 * "nc -N <board> 6969 < flash-image.bin"; the result is sent back on the
 * connection.
 *
 * The whole image is received into RAM first, so an aborted transfer does
 * not touch the flash. Programming erases the first erase unit (the boot
 * header) first and writes it last: if power fails in between, the boot
 * ROM finds no valid header and falls back to its recovery path instead of
 * starting a half-written image.
 */

#include <stdlib.h>
#include <string.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_log_backend.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/ring_buffer.h>

#include "ymodem.h"

#if defined(CONFIG_FWUPDATE_PROGRAM_EXTERNAL)
/*
 * Provided by the platform: programs the image somewhere this core has
 * no direct access to, verifies it and returns the crc32 it read back.
 */
int fwupdate_program_external(const void *img, size_t len, uint32_t *crc);
#endif

#if defined(CONFIG_FWUPDATE_TCP)
#include <zephyr/net/socket.h>
#endif

#if PARTITION_EXISTS(fwupdate_partition)
#define FW_LABEL fwupdate_partition
#elif PARTITION_EXISTS(boot_partition)
#define FW_LABEL boot_partition
#else
#error "fwupdate needs a fwupdate_partition or boot_partition node label"
#endif

#define FW_ID   PARTITION_ID(FW_LABEL)
#define FW_SIZE PARTITION_SIZE(FW_LABEL)

/* Marvell TIM header: version, then "TIMH" (little endian) */
#define TIM_MAGIC_OFFSET 4
#define TIM_MAGIC        0x54494d48

static uint8_t fw_buf[FW_SIZE] __noinit __aligned(8);

static struct fwupdate {
	const struct shell *sh;
	struct ring_buf rx;
	uint8_t rx_mem[CONFIG_FWUPDATE_YMODEM_RX_BUF_SIZE];
	struct k_sem rx_sem;
	bool log_was_on;
	bool dry;
	bool xmodem;
	char name[64];
	size_t size;     /* announced by the sender, 0 = unknown */
	size_t received; /* actual file length */
	atomic_t busy;
} fw;

static K_THREAD_STACK_DEFINE(fw_stack, CONFIG_FWUPDATE_THREAD_STACK_SIZE);
static struct k_thread fw_thread;

#if defined(CONFIG_FWUPDATE_YMODEM)
/* ---------------------------------------------------------------------- */
/* Transport: raw bytes on the shell UART                                  */

static void fw_bypass(const struct shell *sh, uint8_t *data, size_t len, void *user_data)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(user_data);

	(void)ring_buf_put(&fw.rx, data, len);
	k_sem_give(&fw.rx_sem);
}

static int fw_getc(void *ctx, uint8_t *c, int timeout_ms)
{
	k_timepoint_t end = sys_timepoint_calc(K_MSEC(timeout_ms));

	ARG_UNUSED(ctx);

	while (ring_buf_get(&fw.rx, c, 1) == 0) {
		if (k_sem_take(&fw.rx_sem, sys_timepoint_timeout(end)) != 0) {
			return -EAGAIN;
		}
	}
	return 0;
}

static void fw_send(void *ctx, const uint8_t *buf, size_t len)
{
	const struct shell_transport *t = fw.sh->iface;

	ARG_UNUSED(ctx);

	while (len > 0) {
		size_t cnt = 0;

		if (t->api->write(t, buf, len, &cnt) < 0) {
			return;
		}
		buf += cnt;
		len -= cnt;
		if (cnt == 0) {
			k_yield();
		}
	}
}

static int fw_open(void *ctx, const char *name, size_t size)
{
	ARG_UNUSED(ctx);

	if (fw.received != 0) {
		/* One image per transfer */
		return -E2BIG;
	}
	if (size > FW_SIZE) {
		return -EFBIG;
	}
	strncpy(fw.name, name, sizeof(fw.name) - 1);
	fw.name[sizeof(fw.name) - 1] = '\0';
	fw.size = size;
	return 0;
}

static int fw_data(void *ctx, size_t offset, const uint8_t *buf, size_t len)
{
	ARG_UNUSED(ctx);

	if (offset + len > FW_SIZE) {
		return -EFBIG;
	}
	memcpy(&fw_buf[offset], buf, len);
	return 0;
}

static int fw_close(void *ctx, size_t size)
{
	ARG_UNUSED(ctx);

	fw.received = size;
	return 0;
}

static const struct ymodem_ops fw_ymodem_ops = {
	.getc = fw_getc,
	.send = fw_send,
	.open = fw_open,
	.data = fw_data,
	.close = fw_close,
};

#endif /* CONFIG_FWUPDATE_YMODEM */

/* ---------------------------------------------------------------------- */
/* Image check and flash programming                                       */

/* Without a shell (scp) the messages go to the console */
#define FW_ERR(sh, fmt, ...)                                                                     \
	do {                                                                                     \
		if ((sh) != NULL) {                                                              \
			shell_error(sh, fmt, ##__VA_ARGS__);                                     \
		} else {                                                                         \
			printk("fwupdate: " fmt "\n", ##__VA_ARGS__);                            \
		}                                                                                \
	} while (0)
#define FW_PRINT(sh, fmt, ...)                                                                   \
	do {                                                                                     \
		if ((sh) != NULL) {                                                              \
			shell_print(sh, fmt, ##__VA_ARGS__);                                     \
		} else {                                                                         \
			printk("fwupdate: " fmt "\n", ##__VA_ARGS__);                            \
		}                                                                                \
	} while (0)

static int fw_check(const struct shell *sh, size_t len)
{
	if (len == 0) {
		FW_ERR(sh, "empty image");
		return -ENODATA;
	}
	if (fw.size != 0 && fw.size != len) {
		FW_ERR(sh, "size mismatch: announced %zu, received %zu", fw.size, len);
		return -EIO;
	}
	if (IS_ENABLED(CONFIG_FWUPDATE_TIM_HEADER) &&
	    (len < TIM_MAGIC_OFFSET + 4 || sys_get_le32(&fw_buf[TIM_MAGIC_OFFSET]) != TIM_MAGIC)) {
		FW_ERR(sh, "no Marvell TIM header (TIMH) at offset 4, refusing to flash");
		return -ENOEXEC;
	}
	return 0;
}

#if defined(CONFIG_FWUPDATE_PROGRAM_EXTERNAL)
static int fw_program_flash(const struct shell *sh, size_t len)
{
	uint32_t crc = 0;
	int ret;

	FW_PRINT(sh, "handing %zu bytes to the firmware programmer...", len);
	ret = fwupdate_program_external(fw_buf, len, &crc);
	if (ret < 0) {
		FW_ERR(sh, "programming: %d", ret);
		return ret;
	}
	if (crc != crc32_ieee(fw_buf, len)) {
		FW_ERR(sh, "programmer read back crc32 %08x", crc);
		return -EIO;
	}
	return 0;
}
#else
static int fw_program_flash(const struct shell *sh, size_t len)
{
	static uint8_t vbuf[256];
	const struct flash_area *fa;
	struct flash_pages_info info;
	size_t unit, erase_len;
	int ret;

	ret = flash_area_open(FW_ID, &fa);
	if (ret < 0) {
		FW_ERR(sh, "flash_area_open: %d", ret);
		return ret;
	}

	ret = flash_get_page_info_by_offs(flash_area_get_device(fa), fa->fa_off, &info);
	if (ret < 0) {
		goto out;
	}
	unit = info.size;
	erase_len = ROUND_UP(len, unit);

	FW_PRINT(sh, "erasing %zu bytes...", erase_len);
	/* Header first: from here on the old image no longer boots */
	ret = flash_area_erase(fa, 0, unit);
	if (ret == 0 && erase_len > unit) {
		ret = flash_area_erase(fa, unit, erase_len - unit);
	}
	if (ret < 0) {
		FW_ERR(sh, "erase: %d", ret);
		goto out;
	}

	FW_PRINT(sh, "writing...");
	if (len > unit) {
		ret = flash_area_write(fa, unit, &fw_buf[unit], len - unit);
	}
	if (ret == 0) {
		/* Header last */
		ret = flash_area_write(fa, 0, fw_buf, MIN(len, unit));
	}
	if (ret < 0) {
		FW_ERR(sh, "write: %d", ret);
		goto out;
	}

	FW_PRINT(sh, "verifying...");
	for (size_t off = 0; off < len; off += sizeof(vbuf)) {
		size_t n = MIN(sizeof(vbuf), len - off);

		ret = flash_area_read(fa, off, vbuf, n);
		if (ret < 0 || memcmp(vbuf, &fw_buf[off], n) != 0) {
			FW_ERR(sh, "verify failed at 0x%zx", off);
			ret = ret < 0 ? ret : -EIO;
			goto out;
		}
	}

out:
	flash_area_close(fa);
	return ret;
}
#endif /* CONFIG_FWUPDATE_PROGRAM_EXTERNAL */

/*
 * The TIM names the boot device at 0x28: "SPI" + id for SPI NOR, "SAT"
 * + id for SATA. A SATA image goes to the disk, into the GPT partition
 * of the Marvell boot type the A3720 BootROM loads from (the TIM flash
 * offsets are relative to its start), e.g. as sata-install.sh makes it.
 */
#define TIM_BOOT_DEV_OFFSET 0x28
#define TIM_BOOT_DEV_SATA   0x534154 /* "SAT", upper three bytes */

static bool fw_is_sata(size_t len)
{
	return IS_ENABLED(CONFIG_FWUPDATE_TIM_HEADER) && len >= TIM_BOOT_DEV_OFFSET + 4 &&
	       (sys_get_le32(&fw_buf[TIM_BOOT_DEV_OFFSET]) >> 8) == TIM_BOOT_DEV_SATA;
}

#if defined(CONFIG_FWUPDATE_DISK)
#include <zephyr/storage/disk_access.h>

#define FW_DISK CONFIG_FWUPDATE_DISK_NAME

/* 6828311A-BA55-42A4-BCDE-A89BB5EDECAE as stored in a GPT entry */
static const uint8_t mrvl_boot_guid[16] = {0x1a, 0x31, 0x28, 0x68, 0x55, 0xba, 0xa4, 0x42,
					   0xbc, 0xde, 0xa8, 0x9b, 0xb5, 0xed, 0xec, 0xae};

static uint8_t fw_sec[4096] __aligned(64);

/* The first Marvell boot partition: first and last LBA */
static int fw_disk_find(const struct shell *sh, uint32_t ss, uint64_t *first, uint64_t *last,
			uint32_t *index)
{
	uint64_t ent_lba;
	uint32_t n, esz, cur = UINT32_MAX;
	int ret;

	ret = disk_access_read(FW_DISK, fw_sec, 1, 1);
	if (ret != 0) {
		FW_ERR(sh, "%s: read of the GPT header: %d", FW_DISK, ret);
		return ret;
	}
	if (memcmp(fw_sec, "EFI PART", 8) != 0) {
		FW_ERR(sh, "%s: no GPT (make the boot partition with sata-install.sh)", FW_DISK);
		return -ENOENT;
	}
	ent_lba = sys_get_le64(&fw_sec[72]);
	n = sys_get_le32(&fw_sec[80]);
	esz = sys_get_le32(&fw_sec[84]);
	if (esz < 128 || esz > ss || (ss % esz) != 0 || n > 1024) {
		FW_ERR(sh, "%s: odd GPT (%u entries of %u B)", FW_DISK, n, esz);
		return -EINVAL;
	}
	for (uint32_t i = 0; i < n; i++) {
		uint32_t sec = (uint32_t)(ent_lba + (uint64_t)i * esz / ss);
		const uint8_t *e;

		if (sec != cur) {
			ret = disk_access_read(FW_DISK, fw_sec, sec, 1);
			if (ret != 0) {
				FW_ERR(sh, "%s: read of GPT entries: %d", FW_DISK, ret);
				return ret;
			}
			cur = sec;
		}
		e = &fw_sec[(i * esz) % ss];
		if (memcmp(e, mrvl_boot_guid, 16) == 0) {
			*first = sys_get_le64(&e[32]);
			*last = sys_get_le64(&e[40]);
			*index = i + 1;
			return 0;
		}
	}
	FW_ERR(sh, "%s: no Marvell boot partition (type 6828311A-BA55-42A4-BCDE-A89BB5EDECAE)",
	       FW_DISK);
	return -ENOENT;
}

static int fw_program_disk(const struct shell *sh, size_t len)
{
	uint64_t first, last;
	uint32_t ss = 0, nsec, full, part;
	int ret;

	ret = disk_access_init(FW_DISK);
	if (ret == 0) {
		ret = disk_access_ioctl(FW_DISK, DISK_IOCTL_GET_SECTOR_SIZE, &ss);
	}
	if (ret != 0 || ss == 0 || ss > sizeof(fw_sec)) {
		FW_ERR(sh, "%s: disk not ready (%d, sector %u B)", FW_DISK, ret, ss);
		return ret != 0 ? ret : -EIO;
	}
	ret = fw_disk_find(sh, ss, &first, &last, &part);
	if (ret != 0) {
		return ret;
	}
	nsec = DIV_ROUND_UP(len, ss);
	if (last < first || last - first + 1 < nsec) {
		FW_ERR(sh, "%s: boot partition %u too small (%llu sectors, image %u)", FW_DISK,
		       part, (unsigned long long)(last - first + 1), nsec);
		return -EFBIG;
	}
	full = len / ss;
	FW_PRINT(sh, "%s: partition %u at LBA %llu, writing %u sectors...", FW_DISK, part,
		 (unsigned long long)first, nsec);

	/* Header first goes away, so a half-written image does not boot */
	memset(fw_sec, 0, ss);
	ret = disk_access_write(FW_DISK, fw_sec, (uint32_t)first, 1);
	if (ret == 0 && full > 1) {
		ret = disk_access_write(FW_DISK, &fw_buf[ss], (uint32_t)first + 1, full - 1);
	}
	if (ret == 0 && full < nsec && full > 0) {
		memset(fw_sec, 0, ss);
		memcpy(fw_sec, &fw_buf[(size_t)full * ss], len - (size_t)full * ss);
		ret = disk_access_write(FW_DISK, fw_sec, (uint32_t)(first + full), 1);
	}
	if (ret == 0) {
		/* Header last */
		memset(fw_sec, 0, ss);
		memcpy(fw_sec, fw_buf, MIN(len, ss));
		ret = disk_access_write(FW_DISK, fw_sec, (uint32_t)first, 1);
	}
	if (ret == 0) {
		ret = disk_access_ioctl(FW_DISK, DISK_IOCTL_CTRL_SYNC, NULL);
	}
	if (ret != 0) {
		FW_ERR(sh, "%s: write: %d", FW_DISK, ret);
		return ret;
	}

	FW_PRINT(sh, "verifying...");
	for (uint32_t s = 0; s < nsec; s++) {
		size_t off = (size_t)s * ss, n = MIN((size_t)ss, len - off);

		ret = disk_access_read(FW_DISK, fw_sec, (uint32_t)(first + s), 1);
		if (ret != 0 || memcmp(fw_sec, &fw_buf[off], n) != 0) {
			FW_ERR(sh, "verify failed at 0x%zx", off);
			return ret != 0 ? ret : -EIO;
		}
	}
	return 0;
}
#endif /* CONFIG_FWUPDATE_DISK */

static int fw_program(const struct shell *sh, size_t len)
{
	if (fw_is_sata(len)) {
#if defined(CONFIG_FWUPDATE_DISK)
		return fw_program_disk(sh, len);
#else
		FW_ERR(sh, "SATA boot image (TIM boot device SAT): no disk support "
			   "(CONFIG_FWUPDATE_DISK), the SPI NOR would not boot it");
		return -ENOTSUP;
#endif
	}
	return fw_program_flash(sh, len);
}

static const char *fw_where(void)
{
	return fw_is_sata(fw.received) ? "SATA boot partition (boot jumpers on SATA)" : "flash";
}


#if defined(CONFIG_FWUPDATE_SCP)
/*
 * scp -O flash-image.bin <board>:/fw      (or :/fw-dry for a dry run)
 *
 * The file is collected in the same RAM buffer, then checked and
 * programmed as above; the result comes back on the scp's stderr.
 */
#include <zephyr/shell/shell_ssh.h>

static int fw_scp_open(const char *path, const char *name, size_t size, void **h)
{
	const char *p = path[0] == '/' ? &path[1] : path;
	bool dry;

	if (strcmp(p, "fw") == 0) {
		dry = false;
	} else if (strcmp(p, "fw-dry") == 0) {
		dry = true;
	} else {
		return -ENOENT;
	}
	if (size == 0) {
		return -ENODATA;
	}
	if (size > FW_SIZE) {
		return -EFBIG;
	}
	if (!atomic_cas(&fw.busy, 0, 1)) {
		return -EBUSY;
	}

	fw.sh = NULL;
	fw.dry = dry;
	fw.xmodem = false;
	fw.size = size;
	fw.received = 0;
	strncpy(fw.name, name, sizeof(fw.name) - 1);
	fw.name[sizeof(fw.name) - 1] = '\0';
	*h = &fw;
	printk("fwupdate: scp \"%s\", %zu bytes%s\n", fw.name, size, dry ? " (dry run)" : "");
	return 0;
}

static int fw_scp_write(void *h, const uint8_t *data, size_t len)
{
	ARG_UNUSED(h);

	if (fw.received + len > fw.size) {
		return -EFBIG;
	}
	memcpy(&fw_buf[fw.received], data, len);
	fw.received += len;
	return 0;
}

static int fw_scp_close(void *h, bool ok, char *msg, size_t msg_len)
{
	uint32_t crc;
	int ret;

	ARG_UNUSED(h);

	if (!ok) {
		printk("fwupdate: scp transfer incomplete, flash not touched\n");
		ret = -EIO;
		goto done;
	}

	crc = crc32_ieee(fw_buf, fw.received);
	ret = fw_check(NULL, fw.received);
	if (ret < 0) {
		snprintk(msg, msg_len, "image rejected (empty, size or no TIMH header)");
		goto done;
	}
	if (fw.dry) {
		snprintk(msg, msg_len, "fwupdate: %zu bytes, crc32 %08x, dry run, flash not touched",
			 fw.received, crc);
		goto done;
	}

	ret = fw_program(NULL, fw.received);
	if (ret < 0) {
		snprintk(msg, msg_len, "programming failed, see the console");
		goto done;
	}
	snprintk(msg, msg_len, "fwupdate: %zu bytes, crc32 %08x written to the %s and verified, start it with: %s",
		 fw.received, crc, fw_where(), IS_ENABLED(CONFIG_REBOOT) ? "kernel reboot cold" : "a power cycle");
	printk("%s\n", msg);

done:
	atomic_clear(&fw.busy);
	return ret;
}

static const struct shell_ssh_scp_sink fw_scp_sink = {
	.open = fw_scp_open,
	.write = fw_scp_write,
	.close = fw_scp_close,
};

static int fw_scp_init(void)
{
	shell_ssh_scp_sink_set(&fw_scp_sink);
	return 0;
}
SYS_INIT(fw_scp_init, APPLICATION, 99);
#endif /* CONFIG_FWUPDATE_SCP */

/* ---------------------------------------------------------------------- */
/* Shell                                                                   */

#if defined(CONFIG_FWUPDATE_YMODEM)
static void fw_log(const struct shell *sh, bool on)
{
#if defined(CONFIG_SHELL_LOG_BACKEND)
	const struct shell_log_backend *lb = sh->log_backend;

	if (lb == NULL) {
		return;
	}
	if (!on) {
		fw.log_was_on = lb->control_block->state == SHELL_LOG_BACKEND_ENABLED;
		if (fw.log_was_on) {
			z_shell_log_backend_disable(lb);
		}
	} else if (fw.log_was_on) {
		z_shell_log_backend_enable(lb, (void *)sh, sh->ctx->log_level);
	}
#else
	ARG_UNUSED(sh);
	ARG_UNUSED(on);
#endif
}

static void fw_worker(void *p1, void *p2, void *p3)
{
	const struct shell *sh = fw.sh;
	int ret;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	ret = (fw.xmodem ? xmodem_receive : ymodem_receive)(&fw_ymodem_ops, NULL,
							    CONFIG_FWUPDATE_YMODEM_START_TIMEOUT);

	/* Let the sender finish and the terminal settle before talking again */
	k_msleep(500);
	shell_set_bypass(sh, NULL, NULL);
	fw_log(sh, true);

	if (ret <= 0) {
		shell_error(sh, "%s: %s (%d)", fw.xmodem ? "xmodem" : "ymodem",
			    ret == -ECANCELED   ? "cancelled"
			    : ret == -ETIMEDOUT ? "timeout"
			    : ret == 0          ? "no file"
						: "error",
			    ret);
		goto done;
	}

	shell_print(sh, "received \"%s\", %zu bytes, crc32 %08x", fw.name, fw.received,
		    crc32_ieee(fw_buf, fw.received));
	if (fw.xmodem) {
		shell_print(sh, "(XMODEM: padded by the sender to whole blocks, "
				"the padding is written too)");
	}

	if (fw_check(sh, fw.received) < 0) {
		goto done;
	}
	if (fw.dry) {
		shell_print(sh, "dry run, flash not touched");
		goto done;
	}

	ret = fw_program(sh, fw.received);
	if (ret == 0) {
		shell_print(sh, "firmware updated (%s), start it with: %s", fw_where(),
			    IS_ENABLED(CONFIG_REBOOT) ? "kernel reboot cold" : "a power cycle");
	}

done:
	atomic_clear(&fw.busy);
}

#endif /* CONFIG_FWUPDATE_YMODEM */

#if defined(CONFIG_FWUPDATE_TCP)
static void fw_reply(int c, const char *msg)
{
	if (c >= 0) {
		(void)zsock_send(c, msg, strlen(msg), 0);
	}
}

static int fw_tcp_receive(const struct shell *sh, uint16_t port, int *conn)
{
	struct net_sockaddr_in addr = {
		.sin_family = NET_AF_INET,
		.sin_port = net_htons(port),
	};
	struct zsock_pollfd pfd;
	int one = 1;
	int s, c, ret;
	size_t len = 0;

	*conn = -1;
	s = zsock_socket(NET_AF_INET, NET_SOCK_STREAM, NET_IPPROTO_TCP);
	if (s < 0) {
		return -errno;
	}
	(void)zsock_setsockopt(s, ZSOCK_SOL_SOCKET, ZSOCK_SO_REUSEADDR, &one, sizeof(one));
	if (zsock_bind(s, (struct net_sockaddr *)&addr, sizeof(addr)) < 0 ||
	    zsock_listen(s, 1) < 0) {
		ret = -errno;
		goto out;
	}

	pfd.fd = s;
	pfd.events = ZSOCK_POLLIN;
	ret = zsock_poll(&pfd, 1, CONFIG_FWUPDATE_TCP_ACCEPT_TIMEOUT * MSEC_PER_SEC);
	if (ret <= 0) {
		ret = ret == 0 ? -ETIMEDOUT : -errno;
		goto out;
	}
	c = zsock_accept(s, NULL, NULL);
	if (c < 0) {
		ret = -errno;
		goto out;
	}
	*conn = c;

	pfd.fd = c;
	while (true) {
		ssize_t n;

		ret = zsock_poll(&pfd, 1, 10 * MSEC_PER_SEC);
		if (ret <= 0) {
			ret = ret == 0 ? -ETIMEDOUT : -errno;
			break;
		}
		if (len >= FW_SIZE) {
			ret = -EFBIG;
			break;
		}
		n = zsock_recv(c, &fw_buf[len], FW_SIZE - len, 0);
		if (n < 0) {
			ret = -errno;
			break;
		}
		if (n == 0) {
			/* EOF: the sender is done */
			ret = 0;
			break;
		}
		len += n;
	}

	fw.received = len;
out:
	zsock_close(s);
	return ret;
}

static void fw_tcp_worker(void *p1, void *p2, void *p3)
{
	const struct shell *sh = fw.sh;
	uint16_t port = (uintptr_t)p1;
	char msg[96];
	int conn, ret;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	ret = fw_tcp_receive(sh, port, &conn);
	if (ret < 0) {
		shell_error(sh, "tcp: receive failed (%d)", ret);
		fw_reply(conn, "fwupdate: receive failed\n");
		goto done;
	}

	snprintk(msg, sizeof(msg), "received %zu bytes, crc32 %08x\n", fw.received,
		 crc32_ieee(fw_buf, fw.received));
	shell_print(sh, "%s", msg);
	fw_reply(conn, msg);

	if (fw_check(sh, fw.received) < 0) {
		fw_reply(conn, "fwupdate: image rejected\n");
		goto done;
	}
	if (fw.dry) {
		fw_reply(conn, "fwupdate: dry run, flash not touched\n");
		goto done;
	}

	ret = fw_program(sh, fw.received);
	fw_reply(conn, ret == 0 ? "fwupdate: firmware updated\n" : "fwupdate: FAILED\n");
	if (ret == 0) {
		shell_print(sh, "firmware updated (%s), start it with: %s", fw_where(),
			    IS_ENABLED(CONFIG_REBOOT) ? "kernel reboot cold" : "a power cycle");
	}

done:
	if (conn >= 0) {
		zsock_close(conn);
	}
	atomic_clear(&fw.busy);
}

static int cmd_tcp(const struct shell *sh, size_t argc, char **argv)
{
	uintptr_t port = CONFIG_FWUPDATE_TCP_PORT;

	if (!atomic_cas(&fw.busy, 0, 1)) {
		shell_error(sh, "update already running");
		return -EBUSY;
	}

	fw.sh = sh;
	fw.dry = false;
	fw.size = 0;
	fw.received = 0;
	for (size_t i = 1; i < argc; i++) {
		if (strcmp(argv[i], "dry") == 0) {
			fw.dry = true;
		} else {
			port = strtoul(argv[i], NULL, 0);
		}
	}

	shell_print(sh, "partition %s: %u KiB. Waiting %d s on TCP port %u, e.g.",
		    STRINGIFY(FW_LABEL), (unsigned int)(FW_SIZE / 1024),
		    CONFIG_FWUPDATE_TCP_ACCEPT_TIMEOUT, (unsigned int)port);
	shell_print(sh, "  nc -N <board> %u < flash-image.bin%s", (unsigned int)port,
		    fw.dry ? "  (dry run)" : "");

	k_thread_create(&fw_thread, fw_stack, K_THREAD_STACK_SIZEOF(fw_stack), fw_tcp_worker,
			(void *)port, NULL, NULL, CONFIG_FWUPDATE_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&fw_thread, "fwupdate-tcp");

	return 0;
}
#endif /* CONFIG_FWUPDATE_TCP */

#if defined(CONFIG_FWUPDATE_YMODEM)
static int fw_serial_start(const struct shell *sh, size_t argc, char **argv, bool xmodem)
{
	if (!atomic_cas(&fw.busy, 0, 1)) {
		shell_error(sh, "update already running");
		return -EBUSY;
	}

	fw.sh = sh;
	fw.dry = (argc > 1 && strcmp(argv[1], "dry") == 0);
	fw.xmodem = xmodem;
	fw.size = 0;
	fw.received = 0;
	fw.name[0] = '\0';
	ring_buf_init(&fw.rx, sizeof(fw.rx_mem), fw.rx_mem);
	k_sem_init(&fw.rx_sem, 0, 1);

	shell_print(sh, "partition %s: %u KiB. Start the %s upload now (tio: ctrl-t %c),",
		    STRINGIFY(FW_LABEL), (unsigned int)(FW_SIZE / 1024),
		    xmodem ? "XMODEM" : "YMODEM", xmodem ? 'x' : 'y');
	shell_print(sh, "Ctrl-C to abort.%s", fw.dry ? " Dry run." : "");

	fw_log(sh, false);
	shell_set_bypass(sh, fw_bypass, NULL);

	k_thread_create(&fw_thread, fw_stack, K_THREAD_STACK_SIZEOF(fw_stack), fw_worker, NULL,
			NULL, NULL, CONFIG_FWUPDATE_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&fw_thread, "fwupdate");

	return 0;
}

static int cmd_ymodem(const struct shell *sh, size_t argc, char **argv)
{
	return fw_serial_start(sh, argc, argv, false);
}

static int cmd_xmodem(const struct shell *sh, size_t argc, char **argv)
{
	return fw_serial_start(sh, argc, argv, true);
}
#endif /* CONFIG_FWUPDATE_YMODEM */

SHELL_STATIC_SUBCMD_SET_CREATE(
	fwupdate_cmds,
	IF_ENABLED(CONFIG_FWUPDATE_YMODEM,
		   (SHELL_CMD_ARG(ymodem, NULL,
				  "[dry]: receive an image with YMODEM and write it to the "
				  "firmware partition",
				  cmd_ymodem, 1, 1),
		    SHELL_CMD_ARG(xmodem, NULL,
				  "[dry]: the same with XMODEM / XMODEM-1K (CRC or checksum)",
				  cmd_xmodem, 1, 1),))
	IF_ENABLED(CONFIG_FWUPDATE_TCP,
		   (SHELL_CMD_ARG(tcp, NULL, "[port] [dry]: receive an image over one TCP connection",
				  cmd_tcp, 1, 2),))
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(fwupdate, &fwupdate_cmds, "Firmware update", NULL);
