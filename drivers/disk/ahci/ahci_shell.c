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

static void print_ident(const struct shell *sh, struct ahci_data *data)
{
	const struct ahci_ident *id = &data->id;
	uint16_t c = id->sata_cap;

	shell_print(sh, "model   %s", id->model);
	shell_print(sh, "serial  %s  fw %s", id->serial, id->fw);
	shell_print(sh, "size    %llu sectors x %u B = %llu MB (phys sector %u B)%s",
		    (unsigned long long)id->sectors, id->sector_size,
		    (unsigned long long)(id->sectors * id->sector_size / 1000000),
		    id->phys_sector_size, id->lba48 ? ", LBA48" : "");
	shell_print(sh, "SATA    %s%s%s%s%s%s", (c & BIT(1)) ? "Gen1 " : "",
		    (c & BIT(2)) ? "Gen2 " : "", (c & BIT(3)) ? "Gen3 " : "",
		    id->ncq ? "NCQ " : "", id->smart ? "SMART " : "", id->trim ? "TRIM" : "");
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
	uint32_t ssts, vs;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (dev == NULL) {
		return -ENODEV;
	}
	data = dev->data;
	vs = ahci_rd(dev, AHCI_VS);
	shell_print(sh, "%s: AHCI %x.%x CAP %08x GHC %08x IS %08x PI %08x, COMPHY %ld",
		    dev->name, vs >> 16, (vs >> 4) & 0xff, ahci_rd(dev, AHCI_CAP),
		    ahci_rd(dev, AHCI_GHC), ahci_rd(dev, AHCI_IS), ahci_rd(dev, AHCI_PI),
		    data->comphy_ret);
	ssts = ahci_rd(dev, P0(AHCI_PxSSTS));
	shell_print(sh, "port 0: SSTS %08x (%s, %s, %s) SCTL %08x", ssts, det(SSTS_DET(ssts)),
		    spd(SSTS_SPD(ssts)), ipm(SSTS_IPM(ssts)), ahci_rd(dev, P0(AHCI_PxSCTL)));
	shell_print(sh, "        CMD %08x TFD %08x SIG %08x SERR %08x IS %08x IE %08x CI %08x",
		    ahci_rd(dev, P0(AHCI_PxCMD)), ahci_rd(dev, P0(AHCI_PxTFD)),
		    ahci_rd(dev, P0(AHCI_PxSIG)), ahci_rd(dev, P0(AHCI_PxSERR)),
		    ahci_rd(dev, P0(AHCI_PxIS)), ahci_rd(dev, P0(AHCI_PxIE)),
		    ahci_rd(dev, P0(AHCI_PxCI)));
	shell_print(sh, "        %s, %u commands, %u errors, %u interrupts",
		    data->up ? "up" : "down", data->cmds, data->errors, data->irqs);
	if (data->up) {
		print_ident(sh, data);
	}
	return 0;
}

static int cmd_identify(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_dev(sh);
	struct ahci_data *data;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (dev == NULL) {
		return -ENODEV;
	}
	data = dev->data;
	k_mutex_lock(&data->lock, K_FOREVER);
	ret = ahci_identify(dev);
	k_mutex_unlock(&data->lock);
	if (ret != 0) {
		shell_error(sh, "identify: %d", ret);
		return ret;
	}
	print_ident(sh, data);
	return 0;
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

	shell_print(sh, "%s, %s, %s, disk %s", det(SSTS_DET(ssts)), spd(SSTS_SPD(ssts)),
		    ipm(SSTS_IPM(ssts)), data->up ? "ready" : "not ready");
	return 0;
}

static int cmd_read(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = get_dev(sh);
	struct ahci_data *data;
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
	if (!data->up) {
		shell_error(sh, "disk not ready (ata link up)");
		return -ENODEV;
	}
	uint32_t ss = data->id.sector_size;
	uint32_t chunk = data->buf_size / ss;

	k_mutex_lock(&data->lock, K_FOREVER);
	t0 = k_uptime_get();
	for (uint64_t done = 0; done < n && ret == 0;) {
		uint32_t c = (uint32_t)MIN(n - done, (uint64_t)chunk);

		ret = ahci_exec(dev, ATA_READ_DMA_EXT, 0, lba + done, (uint16_t)c, data->buf,
				(size_t)c * ss, false, NULL);
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
	uint8_t d2h[20];
	uint8_t cmd = ATA_CHECK_POWER_MODE;
	int ret;

	if (dev == NULL) {
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
	ret = ahci_exec(dev, cmd, 0, 0, 0, NULL, 0, false, d2h);
	if (ret == 0 && cmd != ATA_CHECK_POWER_MODE) {
		ret = ahci_exec(dev, ATA_CHECK_POWER_MODE, 0, 0, 0, NULL, 0, false, d2h);
	}
	k_mutex_unlock(&data->lock);
	if (ret != 0) {
		shell_error(sh, "power: %d", ret);
		return ret;
	}
	shell_print(sh, "power mode %02x: %s", d2h[12],
		    d2h[12] == 0x00 ? "standby" : d2h[12] == 0x80 ? "idle" :
		    d2h[12] == 0xff ? "active/idle" : "?");
	return 0;
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
	SHELL_CMD_ARG(identify, NULL, "IDENTIFY DEVICE again", cmd_identify, 1, 0),
	SHELL_CMD_ARG(link, NULL, "[up|reset] Link state; up: bring up, reset: COMRESET",
		      cmd_link, 1, 1),
	SHELL_CMD_ARG(read, NULL, "<lba> [<count>] Read, dump the first sector, rate if count>1",
		      cmd_read, 2, 1),
	SHELL_CMD_ARG(power, NULL, "[standby|idle] Check or set the power mode", cmd_power, 1,
		      1),
	SHELL_CMD(debug, &ata_debug_cmds, "Registers, PHY", NULL),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(ata, &ata_cmds, "SATA (AHCI) disk", NULL);
