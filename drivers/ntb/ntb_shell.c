/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ntb shell: the Zephyr side of the Linux ntb_tool. Works on any device
 * with the NTB API; the first one is selected at the first command,
 * "ntb dev <name>" picks another.
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/ntb.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

#define CHUNK 4096U

static const struct device *cur;
static uint8_t buf[CHUNK] __aligned(64);

static struct {
	bool on;
	bool echo;
	uint32_t links;
	uint32_t dbs;
	uint32_t echoes;
	uint32_t echo_errors;
	uint64_t seen;
} ev;

static const struct device *first_ntb(void)
{
	STRUCT_SECTION_FOREACH(device, dev) {
		if (DEVICE_API_IS(ntb, dev) && device_is_ready(dev)) {
			return dev;
		}
	}
	return NULL;
}

static const struct device *get_dev(const struct shell *sh)
{
	if (cur == NULL) {
		cur = first_ntb();
	}
	if (cur == NULL) {
		shell_error(sh, "no NTB device");
	}
	return cur;
}

static int parse_ul(const struct shell *sh, const char *s, unsigned long long *v)
{
	char *end;

	*v = strtoull(s, &end, 0);
	if (end == s || *end != '\0') {
		shell_error(sh, "bad number '%s'", s);
		return -EINVAL;
	}
	return 0;
}

static void on_link(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	ev.links++;
	if (ev.on) {
		printk("ntb: %s link %s\n", dev->name, ntb_link_is_up(dev) ? "up" : "down");
	}
}

/*
 * Echo: the peer rings bit n with a value in our scratchpad 0; we answer
 * with value + 1 in its scratchpad 0 and ring bit n back.
 */
static void on_db(const struct device *dev, int vector, void *user_data)
{
	uint64_t bit = BIT64(vector);

	ARG_UNUSED(user_data);

	ev.dbs++;
	ev.seen |= bit;
	if (ev.on) {
		printk("ntb: %s doorbell %d, spad0 0x%08x\n", dev->name, vector,
		       ntb_spad_read(dev, 0));
	}
	if (ev.echo) {
		(void)ntb_db_clear(dev, bit);
		(void)ntb_peer_spad_write(dev, 0, ntb_spad_read(dev, 0) + 1U);
		if (ntb_peer_db_set(dev, bit) != 0) {
			ev.echo_errors++;
		} else {
			ev.echoes++;
		}
	}
}

static const struct ntb_callbacks shell_cb = {
	.link_event = on_link,
	.db_event = on_db,
};

static int claim(const struct shell *sh, const struct device *dev)
{
	int ret = ntb_set_callbacks(dev, &shell_cb, NULL);

	if (ret != 0) {
		shell_error(sh, "%s: callbacks taken by another client (%d)", dev->name, ret);
	}
	return ret;
}

static int cmd_dev(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1) {
		const struct device *dev = device_get_binding(argv[1]);

		if (dev == NULL || !DEVICE_API_IS(ntb, dev)) {
			shell_error(sh, "%s: not an NTB device", argv[1]);
			return -ENODEV;
		}
		if (cur != NULL && cur != dev && (ev.on || ev.echo)) {
			(void)ntb_set_callbacks(cur, NULL, NULL);
			ev.on = false;
			ev.echo = false;
		}
		cur = dev;
	}
	STRUCT_SECTION_FOREACH(device, dev) {
		if (DEVICE_API_IS(ntb, dev)) {
			shell_print(sh, "%c %s%s", dev == get_dev(sh) ? '*' : ' ', dev->name,
				    device_is_ready(dev) ? "" : " (not ready)");
		}
	}
	return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_dev(sh);
	int n;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (dev == NULL) {
		return -ENODEV;
	}
	shell_print(sh, "%s: link %s, %d scratchpads, doorbells 0x%llx (pending 0x%llx)",
		    dev->name, ntb_link_is_up(dev) ? "up" : "down", ntb_spad_count(dev),
		    (unsigned long long)ntb_db_valid_mask(dev),
		    (unsigned long long)ntb_db_read(dev));

	n = ntb_mw_count(dev);
	for (int i = 0; i < n; i++) {
		struct ntb_mw_align al = {0};
		uint64_t phys = 0;
		size_t size = 0;
		void *va = NULL;

		(void)ntb_mw_get_align(dev, i, &al);
		if (ntb_mw_get_local(dev, i, &va, &phys, &size) == 0) {
			shell_print(sh, "  mw %d (peer writes here): %zu KiB at 0x%llx (cpu %p), "
				    "align 0x%llx, max 0x%llx", i, size / 1024U,
				    (unsigned long long)phys, va,
				    (unsigned long long)al.addr_align,
				    (unsigned long long)al.size_max);
		} else {
			shell_print(sh, "  mw %d: align 0x%llx, max 0x%llx", i,
				    (unsigned long long)al.addr_align,
				    (unsigned long long)al.size_max);
		}
	}
	n = ntb_peer_mw_count(dev);
	for (int i = 0; i < n; i++) {
		volatile void *va = NULL;
		size_t size = 0;

		if (ntb_peer_mw_get_addr(dev, i, &va, &size) == 0) {
			shell_print(sh, "  peer mw %d (we write there): %zu KiB, cpu %p", i,
				    size / 1024U, (void *)va);
		} else {
			shell_print(sh, "  peer mw %d: not set up by the peer", i);
		}
	}
	shell_print(sh, "  shell: events %s, echo %s; link events %u, doorbells %u (bits 0x%llx), "
		    "echoes %u (failed %u)", ev.on ? "on" : "off", ev.echo ? "on" : "off",
		    ev.links, ev.dbs, (unsigned long long)ev.seen, ev.echoes, ev.echo_errors);
	return 0;
}

static int cmd_link(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_dev(sh);
	bool on;
	int ret;

	if (dev == NULL) {
		return -ENODEV;
	}
	if (argc < 2) {
		shell_print(sh, "%s", ntb_link_is_up(dev) ? "up" : "down");
		return 0;
	}
	on = strcmp(argv[1], "up") == 0 || strcmp(argv[1], "on") == 0;
	if (!on && strcmp(argv[1], "down") != 0 && strcmp(argv[1], "off") != 0) {
		shell_error(sh, "up | down");
		return -EINVAL;
	}
	ret = ntb_link_enable(dev, on);
	if (ret != 0) {
		shell_error(sh, "failed (%d)", ret);
	}
	return ret;
}

static int spad_cmd(const struct shell *sh, size_t argc, char **argv, bool peer)
{
	const struct device *dev = get_dev(sh);
	unsigned long long idx, val;
	int n, ret;

	if (dev == NULL) {
		return -ENODEV;
	}
	n = ntb_spad_count(dev);
	if (argc < 2) {
		for (int i = 0; i < n; i++) {
			uint32_t own = ntb_spad_read(dev, i), p = ntb_peer_spad_read(dev, i);

			if (own != 0U || p != 0U) {
				shell_print(sh, "%2d: own 0x%08x  peer 0x%08x", i, own, p);
			}
		}
		return 0;
	}
	if (parse_ul(sh, argv[1], &idx) != 0) {
		return -EINVAL;
	}
	if (idx >= (unsigned long long)n) {
		shell_error(sh, "index 0..%d", n - 1);
		return -EINVAL;
	}
	if (argc < 3) {
		shell_print(sh, "0x%08x", peer ? ntb_peer_spad_read(dev, (int)idx)
					       : ntb_spad_read(dev, (int)idx));
		return 0;
	}
	if (parse_ul(sh, argv[2], &val) != 0) {
		return -EINVAL;
	}
	ret = peer ? ntb_peer_spad_write(dev, (int)idx, (uint32_t)val)
		   : ntb_spad_write(dev, (int)idx, (uint32_t)val);
	if (ret != 0) {
		shell_error(sh, "failed (%d)", ret);
	}
	return ret;
}

static int cmd_spad(const struct shell *sh, size_t argc, char **argv)
{
	return spad_cmd(sh, argc, argv, false);
}

static int cmd_peer_spad(const struct shell *sh, size_t argc, char **argv)
{
	return spad_cmd(sh, argc, argv, true);
}

static int cmd_db(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_dev(sh);
	unsigned long long bits;

	if (dev == NULL) {
		return -ENODEV;
	}
	if (argc >= 3 && strcmp(argv[1], "clear") == 0) {
		if (parse_ul(sh, argv[2], &bits) != 0) {
			return -EINVAL;
		}
		return ntb_db_clear(dev, bits);
	}
	shell_print(sh, "0x%llx (valid 0x%llx)", (unsigned long long)ntb_db_read(dev),
		    (unsigned long long)ntb_db_valid_mask(dev));
	return 0;
}

static int cmd_peer_db(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_dev(sh);
	unsigned long long bits;
	int ret;

	ARG_UNUSED(argc);

	if (dev == NULL) {
		return -ENODEV;
	}
	if (parse_ul(sh, argv[1], &bits) != 0) {
		return -EINVAL;
	}
	ret = ntb_peer_db_set(dev, bits);
	if (ret != 0) {
		shell_error(sh, "failed (%d)", ret);
	}
	return ret;
}

static int cmd_events(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_dev(sh);
	bool *flag = strcmp(argv[0], "echo") == 0 ? &ev.echo : &ev.on;
	bool on;

	if (dev == NULL) {
		return -ENODEV;
	}
	if (argc < 2) {
		shell_print(sh, "%s", *flag ? "on" : "off");
		return 0;
	}
	on = strcmp(argv[1], "on") == 0;
	if (on && !ev.on && !ev.echo && claim(sh, dev) != 0) {
		return -EBUSY;
	}
	*flag = on;
	if (!ev.on && !ev.echo) {
		(void)ntb_set_callbacks(dev, NULL, NULL);
	}
	return 0;
}

/*
 * Memory windows: local (the peer writes it, we read the CPU view) and
 * peer (we write it through ntb_peer_mw_xfer()).
 */
struct win {
	volatile uint8_t *va;
	size_t size;
	bool peer;
	int idx;
};

static int get_win(const struct shell *sh, const struct device *dev, const char *arg, bool peer,
		   struct win *w)
{
	unsigned long long idx;
	int ret;

	if (parse_ul(sh, arg, &idx) != 0) {
		return -EINVAL;
	}
	w->idx = (int)idx;
	w->peer = peer;
	if (peer) {
		volatile void *va;

		ret = ntb_peer_mw_get_addr(dev, w->idx, &va, &w->size);
		w->va = (volatile uint8_t *)va;
	} else {
		void *va;

		ret = ntb_mw_get_local(dev, w->idx, &va, NULL, &w->size);
		w->va = (volatile uint8_t *)va;
	}
	if (ret != 0) {
		shell_error(sh, "%smw %d: %d", peer ? "peer " : "", w->idx, ret);
	}
	return ret;
}

/* copy [off, off + len) of the window into buf */
static int win_get(const struct device *dev, const struct win *w, size_t off, size_t len)
{
	if (w->peer) {
		return ntb_peer_mw_xfer(dev, w->idx, off, buf, len, false);
	}
	for (size_t i = 0; i < len; i++) {
		buf[i] = w->va[off + i];
	}
	return 0;
}

static int win_put(const struct device *dev, const struct win *w, size_t off, size_t len)
{
	if (w->peer) {
		return ntb_peer_mw_xfer(dev, w->idx, off, buf, len, true);
	}
	for (size_t i = 0; i < len; i++) {
		w->va[off + i] = buf[i];
	}
	return 0;
}

static int mw_cmd(const struct shell *sh, size_t argc, char **argv, bool peer)
{
	const struct device *dev = get_dev(sh);
	unsigned long long off = 0, len, v;
	struct win w;
	int ret;

	if (dev == NULL) {
		return -ENODEV;
	}
	if (get_win(sh, dev, argv[1], peer, &w) != 0) {
		return -EINVAL;
	}
	if (argc < 3) {
		shell_print(sh, "%smw %d: %zu bytes, cpu %p", peer ? "peer " : "", w.idx, w.size,
			    (void *)w.va);
		return 0;
	}
	len = w.size;
	if (argc > 3 && parse_ul(sh, argv[3], &off) != 0) {
		return -EINVAL;
	}

	if (strcmp(argv[2], "rd") == 0) {
		len = 64;
		if (argc > 4 && parse_ul(sh, argv[4], &len) != 0) {
			return -EINVAL;
		}
		if (off >= w.size || len > CHUNK || len > w.size - off) {
			shell_error(sh, "offset + length beyond the window or above %u", CHUNK);
			return -EINVAL;
		}
		ret = win_get(dev, &w, off, len);
		if (ret == 0) {
			shell_hexdump_line(sh, (unsigned int)off, buf, MIN(len, 16U));
			for (size_t i = 16; i < len; i += 16) {
				shell_hexdump_line(sh, (unsigned int)(off + i), &buf[i],
						   MIN(len - i, 16U));
			}
		}
	} else if (strcmp(argv[2], "wr") == 0) {
		size_t n = argc - 4;

		if (argc < 5 || off + n * 4U > w.size || n * 4U > CHUNK) {
			shell_error(sh, "wr <off> <u32>...");
			return -EINVAL;
		}
		for (size_t i = 0; i < n; i++) {
			if (parse_ul(sh, argv[4 + i], &v) != 0) {
				return -EINVAL;
			}
			sys_put_le32((uint32_t)v, &buf[i * 4U]);
		}
		ret = win_put(dev, &w, off, n * 4U);
	} else if (strcmp(argv[2], "fill") == 0 || strcmp(argv[2], "crc") == 0) {
		bool fill = argv[2][0] == 'f';
		uint32_t crc = 0;
		int64_t t0;

		/* fill <pattern> [len] / crc [off] [len] */
		if (fill) {
			if (argc < 4 || parse_ul(sh, argv[3], &v) != 0) {
				shell_error(sh, "fill <u32 pattern> [len]");
				return -EINVAL;
			}
			off = 0;
			if (argc > 4 && parse_ul(sh, argv[4], &len) != 0) {
				return -EINVAL;
			}
		} else if (argc > 4 && parse_ul(sh, argv[4], &len) != 0) {
			return -EINVAL;
		} else if (argc <= 4) {
			len = w.size - MIN(off, w.size);
		}
		if (off > w.size || len > w.size - off) {
			shell_error(sh, "beyond the window (%zu bytes)", w.size);
			return -EINVAL;
		}
		t0 = k_uptime_get();
		ret = 0;
		for (size_t done = 0; done < len && ret == 0;) {
			size_t n = MIN((size_t)(len - done), CHUNK);

			if (fill) {
				/* the pattern plus the word offset: shifted copies show */
				for (size_t i = 0; i + 4U <= n; i += 4U) {
					sys_put_le32((uint32_t)v + (uint32_t)((done + i) / 4U),
						     &buf[i]);
				}
				ret = win_put(dev, &w, off + done, n);
				crc = crc32_ieee_update(crc, buf, n);
			} else {
				ret = win_get(dev, &w, off + done, n);
				crc = crc32_ieee_update(crc, buf, n);
			}
			done += n;
		}
		if (ret == 0) {
			int64_t ms = MAX(k_uptime_get() - t0, 1);

			shell_print(sh, "%llu bytes at 0x%llx, crc32 0x%08x, %lld ms (%llu KiB/s)",
				    len, off, crc, ms, (len * 1000ULL / (uint64_t)ms) / 1024U);
		}
	} else {
		shell_error(sh, "rd | wr | fill | crc");
		return -EINVAL;
	}
	if (ret != 0) {
		shell_error(sh, "failed (%d)", ret);
	}
	return ret;
}

static int cmd_mw(const struct shell *sh, size_t argc, char **argv)
{
	return mw_cmd(sh, argc, argv, false);
}

static int cmd_peer_mw(const struct shell *sh, size_t argc, char **argv)
{
	return mw_cmd(sh, argc, argv, true);
}

#define MW_HELP                                                                                    \
	"<idx> [rd <off> [len] | wr <off> <u32>... | fill <pattern> [len] | crc [off] [len]]"

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ntb,
	SHELL_CMD_ARG(dev, NULL, "[name]: list NTB devices, select one", cmd_dev, 1, 1),
	SHELL_CMD(status, NULL, "Link, windows, doorbells", cmd_status),
	SHELL_CMD_ARG(link, NULL, "[up|down]: this side of the link", cmd_link, 1, 1),
	SHELL_CMD_ARG(spad, NULL, "[idx [val]]: own scratchpads (all when no index)",
		      cmd_spad, 1, 2),
	SHELL_CMD_ARG(peer_spad, NULL, "<idx> [val]: scratchpads of the peer", cmd_peer_spad,
		      2, 1),
	SHELL_CMD_ARG(db, NULL, "[clear <bits>]: doorbells rung by the peer", cmd_db, 1, 2),
	SHELL_CMD_ARG(peer_db, NULL, "<bits>: ring doorbells at the peer", cmd_peer_db, 2, 0),
	SHELL_CMD_ARG(events, NULL, "[on|off]: print link and doorbell events", cmd_events, 1,
		      1),
	SHELL_CMD_ARG(echo, NULL,
		      "[on|off]: answer doorbell n with peer spad 0 = spad 0 + 1 and "
		      "doorbell n", cmd_events, 1, 1),
	SHELL_CMD_ARG(mw, NULL, "Window the peer writes: " MW_HELP, cmd_mw, 2, 16),
	SHELL_CMD_ARG(peer_mw, NULL, "Window of the peer: " MW_HELP, cmd_peer_mw, 2, 16),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(ntb, &sub_ntb, "Non-Transparent Bridge", cmd_status);
