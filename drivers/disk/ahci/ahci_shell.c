/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * ata shell: controller and disk state, identify, link reset, raw reads,
 * power state, registers and the SATA PHY (lane 2) registers.
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "ahci.h"

#define P0(off) (AHCI_P0 + (off))

static const struct device *get_dev(const struct shell *sh)
{
	const struct device *dev = ahci_first();

	if (dev == NULL) {
		shell_error(sh, "no AHCI controller");
	}
	return dev;
}

static int parse_u64(const struct shell *sh, const char *s, uint64_t *v)
{
	char *end;

	*v = strtoull(s, &end, 0);
	if (*end != '\0') {
		shell_error(sh, "bad number: %s", s);
		return -EINVAL;
	}
	return 0;
}

static const char *spd(uint32_t s)
{
	static const char *const t[] = {"none", "1.5 Gbps", "3.0 Gbps", "6.0 Gbps"};

	return s < ARRAY_SIZE(t) ? t[s] : "?";
}

static const char *det(uint32_t d)
{
	switch (d) {
	case 0:
		return "no device";
	case 1:
		return "device, no PHY comm";
	case 3:
		return "device, PHY comm";
	case 4:
		return "PHY offline";
	default:
		return "?";
	}
}

static const char *ipm(uint32_t i)
{
	switch (i) {
	case 1:
		return "active";
	case 2:
		return "partial";
	case 6:
		return "slumber";
	case 8:
		return "devsleep";
	default:
		return "-";
	}
}

/* The device the commands work on: "ata dev <n>" (PM port), 0 by default */
static uint8_t cur;

static struct ahci_disk *cur_dk(const struct shell *sh, const struct device *dev, bool need_up)
{
	struct ahci_data *data = dev->data;
	struct ahci_disk *dk = &data->d[cur];

	if (need_up && !dk->up) {
		shell_error(sh, "%s not ready (ata link up, ata dev)", dk->name);
		return NULL;
	}
	return dk;
}

static const char *zoned_str(uint8_t z)
{
	static const char *const t[] = {"not zoned", "host-aware", "device-managed", "reserved"};

	return t[z & 3];
}

static void print_ident(const struct shell *sh, struct ahci_disk *dk)
{
	const struct ahci_ident *id = &dk->id;
	uint16_t c = id->sata_cap;

	shell_print(sh, "%s (PM port %u)", dk->name, dk->pmp);
	shell_print(sh, "model   %s", id->model);
	shell_print(sh, "serial  %s  fw %s", id->serial, id->fw);
	shell_print(sh, "size    %llu sectors x %u B = %llu MB (phys sector %u B)%s",
		    (unsigned long long)id->sectors, id->sector_size,
		    (unsigned long long)(id->sectors * id->sector_size / 1000000),
		    id->phys_sector_size, id->lba48 ? ", LBA48" : "");
	shell_print(sh, "SATA    %s%s%s%s%s%s", (c & BIT(1)) ? "Gen1 " : "",
		    (c & BIT(2)) ? "Gen2 " : "", (c & BIT(3)) ? "Gen3 " : "",
		    id->ncq ? "NCQ " : "", id->smart ? "SMART " : "", id->trim ? "TRIM" : "");
	if (id->ncq) {
		shell_print(sh, "NCQ     depth %u%s%s%s%s", id->qdepth, (c & BIT(12)) ? ", priority" : "",
			    (id->ncq_ext & BIT(4)) ? ", streaming" : "",
			    (id->ncq_ext & BIT(5)) ? ", queue management" : "",
			    (id->ncq_ext & BIT(6)) ? ", send/receive queued" : "");
	}
	shell_print(sh, "zoned   %s", zoned_str(id->zoned));
	if (id->rpm == 1) {
		shell_print(sh, "media   solid state");
	} else if (id->rpm >= 0x401 && id->rpm != 0xffff) {
		shell_print(sh, "media   %u rpm", id->rpm);
	}
}

static int cmd_info(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_dev(sh);
	struct ahci_data *data;
	uint32_t ssts, vs, cap;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (dev == NULL) {
		return -ENODEV;
	}
	data = dev->data;
	vs = ahci_rd(dev, AHCI_VS);
	cap = ahci_rd(dev, AHCI_CAP);
	shell_print(sh, "%s: AHCI %x.%x CAP %08x (%u slots%s%s%s) GHC %08x PI %08x, COMPHY %ld",
		    dev->name, vs >> 16, (vs >> 4) & 0xff, cap,
		    (unsigned int)FIELD_GET(AHCI_CAP_NCS, cap) + 1,
		    (cap & AHCI_CAP_SNCQ) ? ", NCQ" : "", (cap & AHCI_CAP_SPM) ? ", SPM" : "",
		    (cap & AHCI_CAP_FBS) ? ", FBS" : "", ahci_rd(dev, AHCI_GHC),
		    ahci_rd(dev, AHCI_PI), data->comphy_ret);
	ssts = ahci_rd(dev, P0(AHCI_PxSSTS));
	shell_print(sh, "port 0: SSTS %08x (%s, %s, %s) SCTL %08x", ssts, det(SSTS_DET(ssts)),
		    spd(SSTS_SPD(ssts)), ipm(SSTS_IPM(ssts)), ahci_rd(dev, P0(AHCI_PxSCTL)));
	shell_print(sh, "        CMD %08x TFD %08x SIG %08x SERR %08x IS %08x IE %08x CI %08x",
		    ahci_rd(dev, P0(AHCI_PxCMD)), ahci_rd(dev, P0(AHCI_PxTFD)),
		    ahci_rd(dev, P0(AHCI_PxSIG)), ahci_rd(dev, P0(AHCI_PxSERR)),
		    ahci_rd(dev, P0(AHCI_PxIS)), ahci_rd(dev, P0(AHCI_PxIE)),
		    ahci_rd(dev, P0(AHCI_PxCI)));
	shell_print(sh, "        link %s, %u commands, %u errors, %u interrupts",
		    data->link ? "up" : "down", data->cmds, data->errors, data->irqs);
	if (data->pm) {
		shell_print(sh, "port multiplier %04x:%04x rev %08x, %u ports",
			    data->pm_id & 0xffff, data->pm_id >> 16, data->pm_rev, data->pm_ports);
	}
	for (int i = 0; i < AHCI_MAX_DEV; i++) {
		struct ahci_disk *dk = &data->d[i];

		if (!dk->present) {
			continue;
		}
		shell_print(sh, "%c %-6s pmp %u sig %08x %s%s%s", i == cur ? '*' : ' ', dk->name,
			    dk->pmp, dk->sig, dk->up ? dk->id.model : "not ready",
			    dk->up ? ", " : "", dk->up ? (dk->registered ? "disk" : "unregistered")
						   : "");
	}
	return 0;
}

static int cmd_dev(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_dev(sh);
	uint64_t n;

	if (dev == NULL) {
		return -ENODEV;
	}
	if (argc > 1) {
		if (parse_u64(sh, argv[1], &n) != 0 || n >= AHCI_MAX_DEV) {
			shell_error(sh, "device 0..%u", AHCI_MAX_DEV - 1);
			return -EINVAL;
		}
		cur = (uint8_t)n;
	}
	shell_print(sh, "device %u: %s", cur, ((struct ahci_data *)dev->data)->d[cur].name);
	return 0;
}

/* IDENTIFY every device found (or the one given): fresh data, printed */
static int cmd_identify(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_dev(sh);
	struct ahci_data *data;
	uint64_t only = UINT64_MAX;
	int ret = 0, n = 0;

	if (dev == NULL) {
		return -ENODEV;
	}
	data = dev->data;
	if (argc > 1 && (parse_u64(sh, argv[1], &only) != 0 || only >= AHCI_MAX_DEV)) {
		shell_error(sh, "device 0..%u", AHCI_MAX_DEV - 1);
		return -EINVAL;
	}
	if (!data->link) {
		shell_error(sh, "link down (ata link up)");
		return -ENODEV;
	}
	for (int i = 0; i < AHCI_MAX_DEV; i++) {
		struct ahci_disk *dk = &data->d[i];
		int r;

		if ((only != UINT64_MAX && i != (int)only) || (only == UINT64_MAX && !dk->present)) {
			continue;
		}
		if (dk->sig != SIG_ATA && dk->present) {
			shell_print(sh, "%s (PM port %u): signature %08x, %s", dk->name, dk->pmp,
				    dk->sig, dk->sig == SIG_ATAPI ? "ATAPI, not handled" : "unknown");
			continue;
		}
		k_mutex_lock(&data->lock, K_FOREVER);
		r = ahci_identify(dev, dk);
		k_mutex_unlock(&data->lock);
		if (n++ != 0) {
			shell_print(sh, "");
		}
		if (r != 0) {
			shell_error(sh, "%s (PM port %u): identify: %d", dk->name, dk->pmp, r);
			ret = r;
			continue;
		}
		print_ident(sh, dk);
	}
	if (n == 0) {
		shell_error(sh, "no device%s", only != UINT64_MAX ? " there" : " found");
		return -ENODEV;
	}
	return ret;
}

static int cmd_link(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_dev(sh);
	struct ahci_data *data;
	int ret;

	if (dev == NULL) {
		return -ENODEV;
	}
	data = dev->data;
	if (argc > 1 && strcmp(argv[1], "reset") != 0 && strcmp(argv[1], "up") != 0) {
		shell_error(sh, "link [up|reset]");
		return -EINVAL;
	}
	if (argc > 1) {
		k_mutex_lock(&data->lock, K_FOREVER);
		ret = ahci_port_up(dev, strcmp(argv[1], "reset") == 0);
		k_mutex_unlock(&data->lock);
		if (ret != 0) {
			shell_error(sh, "link: %d", ret);
		}
	}
	uint32_t ssts = ahci_rd(dev, P0(AHCI_PxSSTS));

	shell_print(sh, "%s, %s, %s, link %s%s", det(SSTS_DET(ssts)), spd(SSTS_SPD(ssts)),
		    ipm(SSTS_IPM(ssts)), data->link ? "up" : "down",
		    data->pm ? ", port multiplier" : "");
	return 0;
}

static int cmd_read(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_dev(sh);
	struct ahci_data *data;
	struct ahci_disk *dk;
	uint64_t lba, n = 1;
	int64_t t0, dt;
	int ret = 0;

	if (dev == NULL) {
		return -ENODEV;
	}
	data = dev->data;
	if (parse_u64(sh, argv[1], &lba) != 0 || (argc > 2 && parse_u64(sh, argv[2], &n) != 0)) {
		return -EINVAL;
	}
	dk = cur_dk(sh, dev, true);
	if (dk == NULL) {
		return -ENODEV;
	}
	uint32_t ss = dk->id.sector_size;
	uint32_t chunk = data->buf_size / ss;

	k_mutex_lock(&data->lock, K_FOREVER);
	t0 = k_uptime_get();
	for (uint64_t done = 0; done < n && ret == 0;) {
		uint32_t c = (uint32_t)MIN(n - done, (uint64_t)chunk);

		ret = ahci_exec(dev, dk->pmp, ATA_READ_DMA_EXT, 0, lba + done, (uint16_t)c,
				data->buf, (size_t)c * ss, false, NULL);
		if (done == 0 && ret == 0) {
			shell_hexdump(sh, data->buf, MIN(ss, 512U));
		}
		done += c;
	}
	dt = k_uptime_get() - t0;
	k_mutex_unlock(&data->lock);
	if (ret != 0) {
		shell_error(sh, "read: %d", ret);
		return ret;
	}
	if (n > 1) {
		uint64_t bytes = n * ss;

		shell_print(sh, "%llu sectors in %lld ms = %llu kB/s", (unsigned long long)n, dt,
			    dt > 0 ? (unsigned long long)(bytes / (uint64_t)dt) : 0ULL);
	}
	return 0;
}

static int cmd_power(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_dev(sh);
	struct ahci_data *data;
	struct ahci_disk *dk;
	uint8_t d2h[20];
	uint8_t cmd = ATA_CHECK_POWER_MODE;
	int ret;

	if (dev == NULL || (dk = cur_dk(sh, dev, true)) == NULL) {
		return -ENODEV;
	}
	data = dev->data;
	if (argc > 1) {
		if (strcmp(argv[1], "standby") == 0) {
			cmd = ATA_STANDBY_IMMEDIATE;
		} else if (strcmp(argv[1], "idle") == 0) {
			cmd = ATA_IDLE_IMMEDIATE;
		} else {
			shell_error(sh, "power [standby|idle]");
			return -EINVAL;
		}
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	ret = ahci_exec(dev, dk->pmp, cmd, 0, 0, 0, NULL, 0, false, d2h);
	if (ret == 0 && cmd != ATA_CHECK_POWER_MODE) {
		ret = ahci_exec(dev, dk->pmp, ATA_CHECK_POWER_MODE, 0, 0, 0, NULL, 0, false, d2h);
	}
	k_mutex_unlock(&data->lock);
	if (ret != 0) {
		shell_error(sh, "power: %d", ret);
		return ret;
	}
	shell_print(sh, "%s power mode %02x: %s", dk->name, d2h[12],
		    d2h[12] == 0x00 ? "standby" : d2h[12] == 0x80 ? "idle" :
		    d2h[12] == 0xff ? "active/idle" : "?");
	return 0;
}

/* Port multiplier registers: GSCR and every port's SStatus/SError/SControl */
static int cmd_pm(const struct shell *sh, size_t argc, char **argv)
{
	static const uint16_t gscr[] = {GSCR_PROD_ID, GSCR_REV, GSCR_INFO, GSCR_ERROR,
					GSCR_ERROR_EN, GSCR_FEAT, GSCR_FEAT_EN};
	const struct device *dev = get_dev(sh);
	struct ahci_data *data;
	uint32_t v[3];
	int ret = 0;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (dev == NULL) {
		return -ENODEV;
	}
	data = dev->data;
	if (!data->pm) {
		shell_error(sh, "no port multiplier (ata link reset probes again)");
		return -ENODEV;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	for (size_t i = 0; i < ARRAY_SIZE(gscr) && ret == 0; i++) {
		ret = ahci_pm_read(dev, PMP_CTRL, gscr[i], &v[0]);
		if (ret == 0) {
			shell_print(sh, "GSCR[%3u] %08x", gscr[i], v[0]);
		}
	}
	for (uint8_t p = 0; p < data->pm_ports && ret == 0; p++) {
		for (int r = 0; r < 3 && ret == 0; r++) {
			ret = ahci_pm_read(dev, p, r, &v[r]);
		}
		if (ret == 0) {
			shell_print(sh, "port %u: SStatus %08x (%s, %s) SError %08x SControl %08x", p,
				    v[0], det(SSTS_DET(v[0])), spd(SSTS_SPD(v[0])), v[1], v[2]);
		}
	}
	k_mutex_unlock(&data->lock);
	if (ret != 0) {
		shell_error(sh, "PM register read: %d", ret);
	}
	return ret;
}

static int cmd_regs(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_dev(sh);

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (dev == NULL) {
		return -ENODEV;
	}
	for (uint32_t o = 0; o < 0x180; o += 16) {
		if (o == A3700_PHY_ADDR - 8) {
			/* the PHY window: reading data would touch a PHY register */
			shell_print(sh, "%03x: %08x %08x  (PHY window at 0x178 not read)", o, ahci_rd(dev, o),
				    ahci_rd(dev, o + 4));
			break;
		}
		shell_print(sh, "%03x: %08x %08x %08x %08x", o, ahci_rd(dev, o),
			    ahci_rd(dev, o + 4), ahci_rd(dev, o + 8), ahci_rd(dev, o + 12));
	}
	ahci_wr(dev, A3700_VENDOR_ADDR, 0);
	shell_print(sh, "vendor 0xa0=0 -> 0xa4 %08x", ahci_rd(dev, A3700_VENDOR_DATA));
	return 0;
}

static int cmd_phy(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_dev(sh);
	uint64_t reg, n = 1, v;

	if (dev == NULL) {
		return -ENODEV;
	}
	if (strcmp(argv[1], "dump") == 0) {
		/* the SATA PHY block: 16-bit registers 0x00..0x7f at +0x200 */
		for (uint16_t r = 0; r < 0x80; r += 8) {
			uint16_t w[8];

			for (int i = 0; i < 8; i++) {
				w[i] = ahci_phy_rd(dev, A3700_PHY_SATA + r + i);
			}
			shell_print(sh, "%02x: %04x %04x %04x %04x %04x %04x %04x %04x", r, w[0],
				    w[1], w[2], w[3], w[4], w[5], w[6], w[7]);
		}
		return 0;
	}
	if (parse_u64(sh, argv[1], &reg) != 0) {
		return -EINVAL;
	}
	if (argc > 2) {
		if (parse_u64(sh, argv[2], &v) != 0) {
			return -EINVAL;
		}
		ahci_phy_wr(dev, (uint16_t)reg, (uint16_t)v);
	}
	for (uint64_t i = 0; i < n; i++) {
		shell_print(sh, "phy %03llx: %04x", (unsigned long long)(reg + i),
			    ahci_phy_rd(dev, (uint16_t)(reg + i)));
	}
	return 0;
}

static int cmd_comphy(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_dev(sh);
	struct ahci_data *data;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (dev == NULL) {
		return -ENODEV;
	}
	data = dev->data;
	data->comphy_ret = ahci_comphy_power_on(dev);
	shell_print(sh, "SiP COMPHY power on: %ld", data->comphy_ret);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(ata_debug_cmds,
	SHELL_CMD_ARG(regs, NULL, "Controller registers 0x000-0x17f", cmd_regs, 1, 0),
	SHELL_CMD_ARG(phy, NULL,
		      "<reg>|dump [<val>] SATA PHY (COMPHY lane 2) register, 16-bit; "
		      "SATA block at 0x200", cmd_phy, 2, 1),
	SHELL_CMD_ARG(comphy, NULL, "COMPHY lane 2 SATA power on again (SiP)", cmd_comphy, 1,
		      0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(ata_cmds,
	SHELL_CMD_ARG(info, NULL, "Controller, link and disk state", cmd_info, 1, 0),
	SHELL_CMD_ARG(dev, NULL, "[<n>] Device the commands work on (PM port), default 0",
		      cmd_dev, 1, 1),
	SHELL_CMD_ARG(identify, NULL, "[<n>] IDENTIFY every device found, or device n",
		      cmd_identify, 1, 1),
	SHELL_CMD_ARG(link, NULL, "[up|reset] Link state; up: bring up, reset: COMRESET",
		      cmd_link, 1, 1),
	SHELL_CMD_ARG(read, NULL, "<lba> [<count>] Read, dump the first sector, rate if count>1",
		      cmd_read, 2, 1),
	SHELL_CMD_ARG(power, NULL, "[standby|idle] Check or set the power mode", cmd_power, 1,
		      1),
	SHELL_CMD_ARG(pm, NULL, "Port multiplier: GSCR and per-port SStatus/SError/SControl",
		      cmd_pm, 1, 0),
	SHELL_CMD(debug, &ata_debug_cmds, "Registers, PHY", NULL),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(ata, &ata_cmds, "SATA (AHCI) disk", NULL);
