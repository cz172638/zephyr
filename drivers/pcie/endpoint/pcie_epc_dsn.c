/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Device Serial Number kept in settings, as the netshell keeps its MAC
 * addresses: made on the first boot (locally administered EUI-64 from the
 * SPI NOR unique ID, else random), saved under pcie/ep/dsn<n> (the root
 * port keeps its own, pcie/rc/dsn0; an old pcie/dsn<n> moves over), programmed
 * into every EPC with a DSN capability before link autostart. A value the
 * host writes into the capability (setpci ECAP_DSN+4.l / +8.l, where the
 * controller lets it) is picked up by a poll and saved too.
 * Shell: pcie controller dsn [<hex> | uid | new | reset].
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/flash/spi_nor_ex.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(pcie_epc_dsn, CONFIG_PCIE_EP_LOG_LEVEL);

BUILD_ASSERT(CONFIG_PCIE_EPC_DSN_INIT_PRIORITY < CONFIG_PCIE_EPC_AUTOSTART_PRIORITY,
	     "the DSN has to be in place before link autostart");
#ifdef CONFIG_SPI_NOR
BUILD_ASSERT(CONFIG_PCIE_EPC_DSN_INIT_PRIORITY > CONFIG_SPI_NOR_INIT_PRIORITY,
	     "settings on SPI NOR: the flash has to be ready");
#endif

#define MAX_EPC 4

#define PCI_EXT_CAP_ID_DSN 0x0003

static const struct device *epc[MAX_EPC];
static uint64_t dsn[MAX_EPC];
static uint16_t dsn_cap[MAX_EPC];
static int n_epc;
static bool settings_ok;
#if CONFIG_PCIE_EPC_DSN_POLL_MS > 0
static struct k_work_delayable dsn_work;
#endif

struct dsn_read {
	uint64_t v;
	bool found;
};

static int dsn_read_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
		       void *param)
{
	struct dsn_read *r = param;

	ARG_UNUSED(key);
	if (len == sizeof(r->v) && read_cb(cb_arg, &r->v, len) == len) {
		r->found = true;
	}
	return 0;
}

static void dsn_key(char *key, size_t len, int i)
{
	snprintf(key, len, "pcie/ep/dsn%d", i);
}

/* Before the role split the endpoint kept pcie/dsn<n>: move it over */
static void dsn_migrate(int i, struct dsn_read *r)
{
	char old[16];

	snprintf(old, sizeof(old), "pcie/dsn%d", i);
	(void)settings_load_subtree_direct(old, dsn_read_cb, r);
	if (r->found) {
		(void)settings_delete(old);
	}
}

/* The DSN capability in the extended config space, 0 if there is none */
static uint16_t find_dsn_cap(const struct device *dev)
{
	uint32_t off = 0x100, hdr;

	for (int n = 0; n < 64 && off >= 0x100 && off < 0x1000; n++) {
		if (pcie_epc_cfg_read(dev, 0, off, &hdr) != 0 || hdr == 0 || hdr == 0xffffffff) {
			return 0;
		}
		if ((hdr & 0xffff) == PCI_EXT_CAP_ID_DSN) {
			return (uint16_t)off;
		}
		off = (hdr >> 20) & 0xffc;
	}
	return 0;
}

static uint64_t dsn_read_hw(int i)
{
	uint32_t lo = 0, hi = 0;

	(void)pcie_epc_cfg_read(epc[i], 0, dsn_cap[i] + 4, &lo);
	(void)pcie_epc_cfg_read(epc[i], 0, dsn_cap[i] + 8, &hi);
	return ((uint64_t)hi << 32) | lo;
}

/* Locally administered, unicast EUI-64 */
static uint64_t dsn_local(uint64_t v)
{
	return (v & ~(0x01ULL << 56)) | (0x02ULL << 56);
}

static bool dsn_from_uid(uint64_t *v)
{
#if defined(CONFIG_PCIE_EPC_DSN_SPI_NOR_UID) && DT_HAS_COMPAT_STATUS_OKAY(jedec_spi_nor)
	const struct device *nor = DEVICE_DT_GET_ANY(jedec_spi_nor);
	uint8_t uid[SPI_NOR_UID_LEN];
	uint64_t u = 0;

	if (!device_is_ready(nor) ||
	    flash_ex_op(nor, FLASH_EX_OP_SPI_NOR_READ_UID, 0, uid) != 0) {
		return false;
	}
	for (int b = 0; b < SPI_NOR_UID_LEN; b++) {
		u = (u << 8) | uid[b];
	}
	if (u == 0 || u == UINT64_MAX) {
		return false;
	}
	*v = dsn_local(u);
	return true;
#else
	ARG_UNUSED(v);
	return false;
#endif
}

/* The cycle counter mixes in boot timing: the test generator is not seeded */
static uint64_t dsn_random(void)
{
	uint64_t v;

	sys_rand_get(&v, sizeof(v));
	v ^= k_cycle_get_64() * 0x9e3779b97f4a7c15ULL;
	return dsn_local(v);
}

static uint64_t dsn_generate(const char **src)
{
	uint64_t v;

	if (dsn_from_uid(&v)) {
		*src = "SPI NOR unique ID";
		return v;
	}
	*src = "random";
	return dsn_random();
}

static int dsn_apply(int i, uint64_t v, bool save)
{
	char key[16];
	int ret = pcie_epc_set_dsn(epc[i], 0, v);

	if (ret != 0) {
		return ret;
	}
	dsn[i] = v;
	if (save) {
		dsn_key(key, sizeof(key), i);
		ret = settings_save_one(key, &v, sizeof(v));
		if (ret != 0) {
			LOG_WRN("%s: DSN not saved (%d)", epc[i]->name, ret);
		}
	}
	return 0;
}

#if CONFIG_PCIE_EPC_DSN_POLL_MS > 0
/* The host wrote the capability (setpci): keep its value */
static void dsn_poll(struct k_work *work)
{
	ARG_UNUSED(work);
	for (int i = 0; i < n_epc; i++) {
		uint64_t v = dsn_read_hw(i);

		if (v != dsn[i]) {
			LOG_INF("%s: DSN %016llx written by the host", epc[i]->name,
				(unsigned long long)v);
			(void)dsn_apply(i, v, settings_ok);
		}
	}
	k_work_schedule(&dsn_work, K_MSEC(CONFIG_PCIE_EPC_DSN_POLL_MS));
}
#endif

static int dsn_init(void)
{
	const struct device *devs;
	size_t n = z_device_get_all_static(&devs);
	int ret = settings_subsys_init();

	if (ret != 0) {
		LOG_WRN("settings unavailable (%d), DSN will not persist", ret);
	}
	settings_ok = ret == 0;

	for (size_t d = 0; d < n && n_epc < MAX_EPC; d++) {
		const struct device *dev = &devs[d];
		struct dsn_read r = {0};
		const char *src = "saved";
		uint16_t cap;
		char key[16];
		bool save;

		if (!DEVICE_API_IS(pcie_epc, dev) || !device_is_ready(dev)) {
			continue;
		}
		cap = find_dsn_cap(dev);
		if (cap == 0) {
			continue;
		}
		epc[n_epc] = dev;
		dsn_cap[n_epc] = cap;
		dsn_key(key, sizeof(key), n_epc);
		if (settings_ok) {
			(void)settings_load_subtree_direct(key, dsn_read_cb, &r);
			if (!r.found) {
				dsn_migrate(n_epc, &r);
			}
		}
		save = !r.found;
		if (!r.found) {
			r.v = dsn_generate(&src);
		}
		if (dsn_apply(n_epc, r.v, save && settings_ok) == 0) {
			LOG_INF("%s: DSN %016llx (%s, cap 0x%x)", dev->name, (unsigned long long)r.v,
				src, cap);
			n_epc++;
		}
	}
#if CONFIG_PCIE_EPC_DSN_POLL_MS > 0
	if (n_epc != 0) {
		k_work_init_delayable(&dsn_work, dsn_poll);
		k_work_schedule(&dsn_work, K_MSEC(CONFIG_PCIE_EPC_DSN_POLL_MS));
	}
#endif
	return 0;
}

SYS_INIT(dsn_init, POST_KERNEL, CONFIG_PCIE_EPC_DSN_INIT_PRIORITY);

int pcie_epc_dsn_cmd(const struct device *dev, const char *arg, uint64_t *v, const char **src)
{
	for (int i = 0; i < n_epc; i++) {
		if (epc[i] != dev) {
			continue;
		}
		if (arg == NULL) {
			/* show */
		} else if (strcmp(arg, "new") == 0) {
			/* random, so it differs from the one the UID gives */
			(void)dsn_apply(i, dsn_random(), settings_ok);
		} else if (strcmp(arg, "uid") == 0 || strcmp(arg, "reset") == 0) {
			/* reset: back to the start-up default, saved as on the first start */
			(void)dsn_apply(i, dsn_generate(src), settings_ok);
		} else {
			char *end;
			uint64_t n = strtoull(arg, &end, 16);

			if (*end != '\0') {
				return -EINVAL;
			}
			(void)dsn_apply(i, n, settings_ok);
		}
		*v = dsn[i];
		return 0;
	}
	return -ENODEV;
}
