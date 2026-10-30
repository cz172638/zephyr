/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Device Serial Number kept in settings, as the netshell keeps its MAC
 * addresses: generated on the first boot (locally administered EUI-64),
 * saved under pcie/dsn<n>, programmed into every EPC with a DSN
 * capability before link autostart. Shell: pcie_dsn [show | set <hex> |
 * new].
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
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

static const struct device *epc[MAX_EPC];
static uint64_t dsn[MAX_EPC];
static int n_epc;

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
	snprintf(key, len, "pcie/dsn%d", i);
}

/* Locally administered, unicast EUI-64; the cycle counter mixes in boot timing */
static uint64_t dsn_generate(void)
{
	uint64_t v;

	sys_rand_get(&v, sizeof(v));
	v ^= k_cycle_get_64() * 0x9e3779b97f4a7c15ULL;
	v = (v & ~(0x01ULL << 56)) | (0x02ULL << 56);
	return v;
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

static int dsn_init(void)
{
	const struct device *devs;
	size_t n = z_device_get_all_static(&devs);
	int ret = settings_subsys_init();

	if (ret != 0) {
		LOG_WRN("settings unavailable (%d), DSN will not persist", ret);
	}

	for (size_t d = 0; d < n && n_epc < MAX_EPC; d++) {
		const struct device *dev = &devs[d];
		struct dsn_read r = {0};
		char key[16];
		bool save;

		if (!DEVICE_API_IS(pcie_epc, dev) || !device_is_ready(dev)) {
			continue;
		}
		epc[n_epc] = dev;
		dsn_key(key, sizeof(key), n_epc);
		if (ret == 0) {
			(void)settings_load_subtree_direct(key, dsn_read_cb, &r);
		}
		save = !r.found;
		if (!r.found) {
			r.v = dsn_generate();
		}
		if (dsn_apply(n_epc, r.v, save && ret == 0) == 0) {
			LOG_INF("%s: DSN %016llx (%s)", dev->name, (unsigned long long)r.v,
				save ? "generated" : "saved");
			n_epc++;
		}
	}
	return 0;
}

SYS_INIT(dsn_init, POST_KERNEL, CONFIG_PCIE_EPC_DSN_INIT_PRIORITY);

#ifdef CONFIG_SHELL
static int cmd_dsn(const struct shell *sh, size_t argc, char **argv)
{
	for (int i = 0; i < n_epc; i++) {
		if (argc >= 2 && strcmp(argv[1], "new") == 0) {
			(void)dsn_apply(i, dsn_generate(), true);
		} else if (argc >= 3 && strcmp(argv[1], "set") == 0) {
			char *end;
			uint64_t v = strtoull(argv[2], &end, 16);

			if (*end != '\0') {
				shell_error(sh, "hex EUI-64, e.g. 02aabbfffe010203");
				return -EINVAL;
			}
			(void)dsn_apply(i, v, true);
		} else if (argc >= 2 && strcmp(argv[1], "show") != 0) {
			return -EINVAL;
		}
		shell_print(sh, "%s: DSN %016llx  (lspci: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x)",
			    epc[i]->name, (unsigned long long)dsn[i],
			    (uint8_t)(dsn[i] >> 56), (uint8_t)(dsn[i] >> 48), (uint8_t)(dsn[i] >> 40),
			    (uint8_t)(dsn[i] >> 32), (uint8_t)(dsn[i] >> 24), (uint8_t)(dsn[i] >> 16),
			    (uint8_t)(dsn[i] >> 8), (uint8_t)dsn[i]);
	}
	if (n_epc == 0) {
		shell_print(sh, "no EPC with a DSN capability");
	}
	return 0;
}

SHELL_CMD_ARG_REGISTER(pcie_dsn, NULL, "[show | set <hex> | new] PCIe Device Serial Number",
		       cmd_dsn, 1, 2);
#endif
