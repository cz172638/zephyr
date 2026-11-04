/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * AHCI SATA disk, port 0, one command slot, through a bounce buffer;
 * devices behind a port multiplier (command-based switching, one command
 * at a time) each become a disk of their own.
 *
 * Armada 3700 glue (Linux ahci_mvebu, TF-A phy-comphy-3700): vendor
 * register 0xa0/0xa4 bit 6, then the SATA PHY on COMPHY lane 2 through
 * the TF-A SiP service. The lane 2 PHY registers are reachable through
 * the controller window at +0x178/+0x17c (ahci_phy_rd/wr, ata debug phy).
 *
 * The link and the disk come up in a background thread at boot (a
 * rotating disk takes seconds to spin up) or on the first disk access.
 */

#define DT_DRV_COMPAT marvell_armada_3700_ahci

#include <string.h>

#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/byteorder.h>

#include "ahci.h"

LOG_MODULE_REGISTER(ahci, CONFIG_AHCI_LOG_LEVEL);

#define SIP_COMPHY_POWER_ON 0x82000001
#define COMPHY_MODE_SATA    1

#define P0(off) (AHCI_P0 + (off))

#define PxIE_DEFAULT (PxIS_ERRORS | PxIS_DHRS | PxIS_PSS | PxIS_DSS | PxIS_SDBS | PxIS_PCS | \
		      PxIS_PRCS)

uint32_t ahci_rd(const struct device *dev, uint32_t off)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + off);
}

void ahci_wr(const struct device *dev, uint32_t off, uint32_t v)
{
	sys_write32(v, DEVICE_MMIO_GET(dev) + off);
}

static void ahci_rmw(const struct device *dev, uint32_t off, uint32_t clr, uint32_t set)
{
	ahci_wr(dev, off, (ahci_rd(dev, off) & ~clr) | set);
}

uint16_t ahci_phy_rd(const struct device *dev, uint16_t reg)
{
	ahci_wr(dev, A3700_PHY_ADDR, reg);
	return (uint16_t)ahci_rd(dev, A3700_PHY_DATA);
}

void ahci_phy_wr(const struct device *dev, uint16_t reg, uint16_t v)
{
	ahci_wr(dev, A3700_PHY_ADDR, reg);
	ahci_wr(dev, A3700_PHY_DATA, v);
}

/* Wait until (reg & mask) == val, true on success */
static bool wait_reg(const struct device *dev, uint32_t off, uint32_t mask, uint32_t val,
		     uint32_t ms)
{
	int64_t end = k_uptime_get() + ms;

	do {
		if ((ahci_rd(dev, off) & mask) == val) {
			return true;
		}
		k_msleep(1);
	} while (k_uptime_get() < end);
	return (ahci_rd(dev, off) & mask) == val;
}

static uint64_t phys(const void *p)
{
	return (uint64_t)k_mem_phys_addr((void *)p);
}

long ahci_comphy_power_on(const struct device *dev)
{
	const struct ahci_config *cfg = dev->config;
	struct arm_smccc_res res;

	arm_smccc_smc(SIP_COMPHY_POWER_ON, cfg->comphy_lane,
		      ((uint32_t)COMPHY_MODE_SATA << 12) | (cfg->comphy_invert & 0x3), 0, 0, 0, 0,
		      0, &res);
	return (long)res.a0;
}

static void port_stop(const struct device *dev)
{
	ahci_rmw(dev, P0(AHCI_PxCMD), PxCMD_ST, 0);
	if (!wait_reg(dev, P0(AHCI_PxCMD), PxCMD_CR, 0, 500)) {
		LOG_WRN("command list engine does not stop");
	}
}

static void port_start(const struct device *dev)
{
	ahci_rmw(dev, P0(AHCI_PxCMD), 0, PxCMD_ST);
}

static void ata_string(char *dst, const uint16_t *w, int words)
{
	int n = 0;

	for (int i = 0; i < words; i++) {
		dst[n++] = (char)(w[i] >> 8);
		dst[n++] = (char)(w[i] & 0xff);
	}
	dst[n] = '\0';
	while (n > 0 && (dst[n - 1] == ' ' || dst[n - 1] == '\0')) {
		dst[--n] = '\0';
	}
	/* leading blanks too */
	int s = 0;

	while (dst[s] == ' ') {
		s++;
	}
	if (s != 0) {
		memmove(dst, dst + s, n - s + 1);
	}
}

static const char *spd_str(uint32_t spd)
{
	switch (spd) {
	case 1:
		return "1.5 Gbps";
	case 2:
		return "3.0 Gbps";
	case 3:
		return "6.0 Gbps";
	default:
		return "-";
	}
}

/* Back to an idle port after an error: engine off, errors cleared, CLO */
static void port_recover(const struct device *dev)
{
	struct ahci_data *data = dev->data;

	port_stop(dev);
	ahci_wr(dev, P0(AHCI_PxSERR), UINT32_MAX);
	ahci_wr(dev, P0(AHCI_PxIS), UINT32_MAX);
	if ((ahci_rd(dev, P0(AHCI_PxTFD)) & (TFD_BSY | TFD_DRQ)) != 0) {
		if ((data->cap & AHCI_CAP_SCLO) != 0) {
			ahci_rmw(dev, P0(AHCI_PxCMD), 0, PxCMD_CLO);
			(void)wait_reg(dev, P0(AHCI_PxCMD), PxCMD_CLO, 0, 500);
		} else {
			LOG_WRN("device busy and no CLO: COMRESET needed (ata link reset)");
		}
	}
	port_start(dev);
}

/*
 * One command in slot 0: the H2D FIS as given, header options (CFL, W,
 * R, C, PMP) as given. Waits for the slot to clear; with check, a task
 * file or interface error ends it early.
 */
static int ahci_issue(const struct device *dev, uint32_t opts, const uint8_t *cfis, void *buf,
		      size_t len, bool write, uint32_t ms, bool check)
{
	struct ahci_data *data = dev->data;
	struct ahci_cmd_hdr *hdr = &data->clb[0];
	struct ahci_cmd_tbl *tbl = data->tbl;
	uint64_t ct = phys(tbl);
	int64_t end;
	uint32_t is;

	if (len > data->buf_size || (len & 1) != 0) {
		return -EINVAL;
	}
	memset(tbl, 0, sizeof(*tbl));
	memcpy(tbl->cfis, cfis, 20);
	memset(hdr, 0, sizeof(*hdr));
	hdr->opts = opts | ((len != 0 ? 1U : 0U) << 16) | (write ? BIT(6) : 0);
	hdr->ctba = (uint32_t)ct;
	hdr->ctbau = (uint32_t)(ct >> 32);
	if (len != 0) {
		uint64_t b = phys(buf);

		tbl->prd[0].dba = (uint32_t)b;
		tbl->prd[0].dbau = (uint32_t)(b >> 32);
		tbl->prd[0].dbc = (uint32_t)(len - 1);
		if (write) {
			(void)sys_cache_data_flush_range(buf, len);
		} else {
			(void)sys_cache_data_flush_and_invd_range(buf, len);
		}
	}
	(void)sys_cache_data_flush_range(hdr, sizeof(*hdr));
	(void)sys_cache_data_flush_range(tbl, sizeof(*tbl));
	(void)sys_cache_data_flush_and_invd_range(data->rfis, 256);

	data->last_is = 0;
	k_sem_reset(&data->done);
	ahci_wr(dev, P0(AHCI_PxIS), UINT32_MAX);
	ahci_wr(dev, P0(AHCI_PxCI), BIT(0));
	data->cmds++;

	end = k_uptime_get() + ms;
	for (;;) {
		(void)k_sem_take(&data->done, K_MSEC(2));
		is = data->last_is | ahci_rd(dev, P0(AHCI_PxIS));
		if (check && ((is & PxIS_ERRORS) != 0 ||
			      (ahci_rd(dev, P0(AHCI_PxTFD)) & TFD_ERR) != 0)) {
			data->last_is = is;
			break;
		}
		if ((ahci_rd(dev, P0(AHCI_PxCI)) & BIT(0)) == 0) {
			break;
		}
		if (k_uptime_get() > end) {
			data->last_is = is;
			if (len != 0 && !write) {
				(void)sys_cache_data_invd_range(buf, len);
			}
			return -ETIMEDOUT;
		}
	}
	data->last_is = is;
	if (len != 0 && !write) {
		(void)sys_cache_data_invd_range(buf, len);
	}
	(void)sys_cache_data_invd_range(data->rfis, 256);
	if (check && ((is & PxIS_ERRORS) != 0 || (ahci_rd(dev, P0(AHCI_PxTFD)) & TFD_ERR) != 0)) {
		return -EIO;
	}
	return 0;
}

static void build_fis(uint8_t *fis, uint8_t pmp, uint8_t cmd, uint16_t feat, uint64_t lba,
		      uint16_t count, uint8_t device)
{
	memset(fis, 0, 20);
	fis[0] = 0x27;          /* Register H2D */
	fis[1] = 0x80 | (pmp & 0xf); /* command, PM port */
	fis[2] = cmd;
	fis[3] = (uint8_t)feat;
	fis[4] = (uint8_t)lba;
	fis[5] = (uint8_t)(lba >> 8);
	fis[6] = (uint8_t)(lba >> 16);
	fis[7] = device;
	fis[8] = (uint8_t)(lba >> 24);
	fis[9] = (uint8_t)(lba >> 32);
	fis[10] = (uint8_t)(lba >> 40);
	fis[11] = (uint8_t)(feat >> 8);
	fis[12] = (uint8_t)count;
	fis[13] = (uint8_t)(count >> 8);
}

int ahci_exec(const struct device *dev, uint8_t pmp, uint8_t cmd, uint16_t feat, uint64_t lba,
	      uint16_t count, void *buf, size_t len, bool write, uint8_t *d2h)
{
	struct ahci_data *data = dev->data;
	uint8_t fis[20];
	int ret;

	if (!data->link) {
		return -ENODEV;
	}
	build_fis(fis, pmp, cmd, feat, lba, count,
		  (cmd == ATA_READ_DMA_EXT || cmd == ATA_WRITE_DMA_EXT) ? 0x40 : 0);
	ret = ahci_issue(dev, 5 | ((uint32_t)(pmp & 0xf) << 12), fis, buf, len, write,
			 CONFIG_AHCI_CMD_TIMEOUT_MS, true);
	if (d2h != NULL) {
		memcpy(d2h, data->rfis + 0x40, 20);
	}
	if (ret != 0) {
		data->errors++;
		LOG_ERR("pmp %u cmd %02x lba %llu: %s, IS %08x TFD %08x SERR %08x", pmp, cmd,
			(unsigned long long)lba, ret == -EIO ? "error" : "timeout", data->last_is,
			ahci_rd(dev, P0(AHCI_PxTFD)), ahci_rd(dev, P0(AHCI_PxSERR)));
		port_recover(dev);
	}
	return ret;
}

/* Port multiplier registers: READ/WRITE PORT MULTIPLIER on the control port */
int ahci_pm_read(const struct device *dev, uint8_t port, uint16_t reg, uint32_t *val)
{
	struct ahci_data *data = dev->data;
	uint8_t fis[20];
	const uint8_t *d = data->rfis + 0x40;
	int ret;

	build_fis(fis, PMP_CTRL, ATA_READ_PM, reg, 0, 0, port & 0xf);
	ret = ahci_issue(dev, 5 | ((uint32_t)PMP_CTRL << 12), fis, NULL, 0, false, 1000, true);
	if (ret != 0) {
		port_recover(dev);
		return ret;
	}
	*val = d[12] | ((uint32_t)d[4] << 8) | ((uint32_t)d[5] << 16) | ((uint32_t)d[6] << 24);
	return 0;
}

int ahci_pm_write(const struct device *dev, uint8_t port, uint16_t reg, uint32_t val)
{
	uint8_t fis[20];
	int ret;

	build_fis(fis, PMP_CTRL, ATA_WRITE_PM, reg, (uint64_t)(val >> 8) & 0xffffff,
		  (uint16_t)(val & 0xff), port & 0xf);
	ret = ahci_issue(dev, 5 | ((uint32_t)PMP_CTRL << 12), fis, NULL, 0, false, 1000, true);
	if (ret != 0) {
		port_recover(dev);
	}
	return ret;
}

/*
 * Software reset of one device (or the PM on its control port): SRST set
 * with R and C in the header, then SRST cleared; the signature comes back
 * in the D2H FIS.
 */
static int ahci_softreset(const struct device *dev, uint8_t pmp, uint32_t *sig)
{
	struct ahci_data *data = dev->data;
	const uint8_t *d = data->rfis + 0x40;
	uint8_t fis[20] = {0x27, pmp & 0xf};
	uint32_t opts = 5 | ((uint32_t)(pmp & 0xf) << 12);
	int ret;

	fis[15] = 0x04; /* SRST */
	ret = ahci_issue(dev, opts | BIT(8) | BIT(10), fis, NULL, 0, false, 500, false);
	if (ret != 0) {
		port_recover(dev);
		return ret;
	}
	k_busy_wait(20);
	fis[15] = 0x00;
	memset(data->rfis + 0x40, 0, 20);
	(void)sys_cache_data_flush_range(data->rfis, 256);
	ret = ahci_issue(dev, opts, fis, NULL, 0, false, 500, false);
	if (ret != 0) {
		port_recover(dev);
		return ret;
	}
	/* the D2H with the signature, then BSY clears */
	int64_t end = k_uptime_get() + CONFIG_AHCI_READY_TIMEOUT_MS;

	do {
		(void)sys_cache_data_invd_range(data->rfis, 256);
		if (d[0] == 0x34 && (d[2] & TFD_BSY) == 0) {
			*sig = d[12] | ((uint32_t)d[4] << 8) | ((uint32_t)d[5] << 16) |
			       ((uint32_t)d[6] << 24);
			return 0;
		}
		k_msleep(5);
	} while (k_uptime_get() < end);
	return -ETIMEDOUT;
}

int ahci_identify(const struct device *dev, struct ahci_disk *dk)
{
	struct ahci_data *data = dev->data;
	struct ahci_ident *id = &dk->id;
	uint16_t *w = (uint16_t *)data->buf;
	int ret;

	ret = ahci_exec(dev, dk->pmp, ATA_IDENTIFY, 0, 0, 0, data->buf, 512, false, NULL);
	if (ret != 0) {
		return ret;
	}
	for (int i = 0; i < 256; i++) {
		w[i] = sys_le16_to_cpu(w[i]);
	}
	ata_string(id->serial, &w[10], 10);
	ata_string(id->fw, &w[23], 4);
	ata_string(id->model, &w[27], 20);
	id->lba48 = (w[83] & BIT(10)) != 0;
	id->sectors = id->lba48 ? ((uint64_t)w[103] << 48 | (uint64_t)w[102] << 32 |
				   (uint64_t)w[101] << 16 | w[100])
				: ((uint32_t)w[61] << 16 | w[60]);
	id->sector_size = 512;
	if ((w[106] & 0xc000) == 0x4000 && (w[106] & BIT(12)) != 0) {
		id->sector_size = 2 * ((uint32_t)w[118] << 16 | w[117]);
	}
	id->phys_sector_size = id->sector_size;
	if ((w[106] & 0xc000) == 0x4000 && (w[106] & BIT(13)) != 0) {
		id->phys_sector_size = id->sector_size << (w[106] & 0xf);
	}
	id->sata_cap = w[76];
	id->ncq = (w[76] & BIT(8)) != 0;
	id->ncq_ext = w[77];
	id->qdepth = id->ncq ? (uint8_t)((w[75] & 0x1f) + 1) : 1;
	id->zoned = w[69] & 0x3;
	id->sec_supported = (w[82] & BIT(1)) != 0;
	id->sec = w[128];
	id->sec_erase = w[89];
	id->sec_enh_erase = w[90];
	id->trusted = (w[48] & 0xc001) == 0x4001;
	id->smart = (w[82] & BIT(0)) != 0;
	id->trim = (w[169] & BIT(0)) != 0;
	id->rpm = w[217];
	return 0;
}

static void disk_announce(const struct device *dev, struct ahci_disk *dk, const char *link,
			  int64_t ms)
{
	ARG_UNUSED(dev);
	LOG_INF("%s: %s%s, %s, ready after %lld ms, SIG %08x", dk->name, link,
		dk->pmp != 0 || ((struct ahci_data *)dev->data)->pm ? " via PM" : "",
		dk->id.model, ms, dk->sig);
	LOG_INF("%s: %llu sectors of %u B (%llu MB), serial %s, fw %s, NCQ depth %u", dk->name,
		(unsigned long long)dk->id.sectors, dk->id.sector_size,
		(unsigned long long)(dk->id.sectors * dk->id.sector_size / 1000000),
		dk->id.serial, dk->id.fw, dk->id.qdepth);
}

static void disk_register(const struct device *dev, struct ahci_disk *dk);

/* One port behind the PM: link (COMRESET through SControl if needed), SRST, identify */
static int pm_port_up(const struct device *dev, uint8_t p, bool comreset)
{
	struct ahci_data *data = dev->data;
	struct ahci_disk *dk = &data->d[p];
	uint32_t ss = 0;
	int64_t t0, end;
	int ret;

	dk->present = false;
	dk->up = false;
	uint32_t sc = 0;

	ret = ahci_pm_read(dev, p, PSCR_SSTATUS, &ss);
	if (ret == 0) {
		ret = ahci_pm_read(dev, p, PSCR_SCONTROL, &sc);
	}
	if (ret != 0) {
		return ret;
	}
	/*
	 * Also a port that came up by itself (SControl still 0): the reset
	 * renegotiates the speed and turns partial/slumber off (IPM = 3).
	 */
	if (comreset || SSTS_DET(ss) != 3 || (sc & 0xf00) != 0x300) {
		(void)ahci_pm_write(dev, p, PSCR_SCONTROL, 0x301);
		k_msleep(2);
		(void)ahci_pm_write(dev, p, PSCR_SCONTROL, 0x300);
		end = k_uptime_get() + CONFIG_AHCI_LINK_TIMEOUT_MS;
		do {
			k_msleep(10);
			ret = ahci_pm_read(dev, p, PSCR_SSTATUS, &ss);
		} while (ret == 0 && SSTS_DET(ss) != 3 && k_uptime_get() < end);
	}
	if (SSTS_DET(ss) != 3) {
		return -ENODEV;
	}
	(void)ahci_pm_write(dev, p, PSCR_SERROR, UINT32_MAX);
	ahci_wr(dev, P0(AHCI_PxSERR), UINT32_MAX);

	t0 = k_uptime_get();
	ret = ahci_softreset(dev, p, &dk->sig);
	if (ret != 0) {
		LOG_WRN("%s: PM port %u link %s, no answer to reset (%d)", dk->name, p,
			spd_str(SSTS_SPD(ss)), ret);
		return ret;
	}
	dk->present = true;
	if (dk->sig != SIG_ATA) {
		LOG_INF("%s: PM port %u: signature %08x (%s), not handled", dk->name, p, dk->sig,
			dk->sig == SIG_ATAPI ? "ATAPI" : "?");
		return -ENOTSUP;
	}
	ret = ahci_identify(dev, dk);
	if (ret != 0) {
		return ret;
	}
	dk->up = true;
	disk_announce(dev, dk, spd_str(SSTS_SPD(ss)), k_uptime_get() - t0);
	disk_register(dev, dk);
	return 0;
}

int ahci_port_up(const struct device *dev, bool comreset)
{
	const struct ahci_config *cfg = dev->config;
	struct ahci_data *data = dev->data;
	uint64_t clb = phys(data->clb), fb = phys(data->rfis);
	bool spm = (data->cap & AHCI_CAP_SPM) != 0;
	uint32_t ssts, sig = 0;
	int64_t t0;
	int ret;

	if (!data->hba_ok) {
		return -EIO;
	}
	data->link = false;
	data->pm = false;
	for (int i = 0; i < AHCI_MAX_DEV; i++) {
		data->d[i].up = false;
		data->d[i].present = false;
	}

	/* Engines off, DMA structures in; PMA only with the engine stopped */
	port_stop(dev);
	ahci_rmw(dev, P0(AHCI_PxCMD), PxCMD_FRE, 0);
	(void)wait_reg(dev, P0(AHCI_PxCMD), PxCMD_FR, 0, 500);
	memset(data->clb, 0, 32 * sizeof(*data->clb));
	memset(data->rfis, 0, 256);
	(void)sys_cache_data_flush_range(data->clb, 32 * sizeof(*data->clb));
	(void)sys_cache_data_flush_range(data->rfis, 256);
	ahci_wr(dev, P0(AHCI_PxCLB), (uint32_t)clb);
	ahci_wr(dev, P0(AHCI_PxCLBU), (uint32_t)(clb >> 32));
	ahci_wr(dev, P0(AHCI_PxFB), (uint32_t)fb);
	ahci_wr(dev, P0(AHCI_PxFBU), (uint32_t)(fb >> 32));
	ahci_rmw(dev, P0(AHCI_PxCMD), PxCMD_PMA,
		 PxCMD_FRE | PxCMD_POD | ((data->cap & AHCI_CAP_SSS) != 0 ? PxCMD_SUD : 0) |
			 (spm ? PxCMD_PMA : 0));
	ahci_wr(dev, P0(AHCI_PxSERR), UINT32_MAX);
	ahci_wr(dev, P0(AHCI_PxIS), UINT32_MAX);

	/* COMRESET, partial/slumber off (IPM = 3) */
	if (comreset || SSTS_DET(ahci_rd(dev, P0(AHCI_PxSSTS))) != 3) {
		ahci_wr(dev, P0(AHCI_PxSCTL), 0x301);
		k_msleep(2);
		ahci_wr(dev, P0(AHCI_PxSCTL), 0x300);
	}
	if (!wait_reg(dev, P0(AHCI_PxSSTS), 0xf, 3, CONFIG_AHCI_LINK_TIMEOUT_MS)) {
		ssts = ahci_rd(dev, P0(AHCI_PxSSTS));
		LOG_INF("%s: no device (SSTS %08x, COMPHY %ld)", cfg->disk_name, ssts,
			data->comphy_ret);
		return -ENODEV;
	}
	ssts = ahci_rd(dev, P0(AHCI_PxSSTS));
	ahci_wr(dev, P0(AHCI_PxSERR), UINT32_MAX);

	/* The first D2H FIS: BSY clears once a disk has spun up */
	t0 = k_uptime_get();
	if (!wait_reg(dev, P0(AHCI_PxTFD), TFD_BSY | TFD_DRQ, 0, CONFIG_AHCI_READY_TIMEOUT_MS)) {
		LOG_ERR("%s: link %s but the device stays busy (TFD %08x)", cfg->disk_name,
			spd_str(SSTS_SPD(ssts)), ahci_rd(dev, P0(AHCI_PxTFD)));
		return -EBUSY;
	}
	ahci_wr(dev, P0(AHCI_PxIS), UINT32_MAX);
	ahci_wr(dev, P0(AHCI_PxIE), PxIE_DEFAULT);
	port_start(dev);
	data->link = true;

	/* A port multiplier answers a reset of its control port with its own signature */
	ret = ahci_softreset(dev, PMP_CTRL, &sig);
	if (ret == 0 && sig == SIG_PM) {
		uint32_t info = 0;

		data->pm = true;
		(void)ahci_pm_read(dev, PMP_CTRL, GSCR_PROD_ID, &data->pm_id);
		(void)ahci_pm_read(dev, PMP_CTRL, GSCR_REV, &data->pm_rev);
		(void)ahci_pm_read(dev, PMP_CTRL, GSCR_INFO, &info);
		data->pm_ports = MIN((uint8_t)(info & 0xf), (uint8_t)AHCI_MAX_DEV);
		LOG_INF("%s: port multiplier %04x:%04x rev %08x, %u ports (link %s, HBA %s%s)",
			cfg->disk_name, data->pm_id & 0xffff, data->pm_id >> 16, data->pm_rev,
			info & 0xf, spd_str(SSTS_SPD(ssts)), spm ? "SPM" : "no SPM",
			(data->cap & AHCI_CAP_FBS) != 0 ? ", FBS" : "");
		/* no asynchronous notifications or PHY event reporting needed */
		for (uint8_t p = 0; p < data->pm_ports; p++) {
			ret = pm_port_up(dev, p, comreset);
			if (ret == -ENODEV) {
				LOG_INF("%s: PM port %u: empty", data->d[p].name, p);
			}
		}
		return data->d[0].up ? 0 : -ENODEV;
	}

	/* Direct attached device */
	if (spm) {
		port_stop(dev);
		ahci_rmw(dev, P0(AHCI_PxCMD), PxCMD_PMA, 0);
		port_start(dev);
	}
	data->d[0].sig = (ret == 0) ? sig : ahci_rd(dev, P0(AHCI_PxSIG));
	data->d[0].present = true;
	if (data->d[0].sig == SIG_ATAPI) {
		LOG_INF("%s: ATAPI device (signature %08x), not handled yet", cfg->disk_name,
			data->d[0].sig);
		return -ENOTSUP;
	}
	ret = ahci_identify(dev, &data->d[0]);
	if (ret != 0) {
		return ret;
	}
	data->d[0].up = true;
	disk_announce(dev, &data->d[0], spd_str(SSTS_SPD(ssts)), k_uptime_get() - t0);
	return 0;
}

int ahci_rw(const struct device *dev, struct ahci_disk *dk, uint64_t lba, uint32_t count,
	    void *buf, bool write)
{
	struct ahci_data *data = dev->data;
	uint32_t ss = dk->id.sector_size;
	uint32_t max = MIN(data->buf_size / ss, 65536U);
	uint8_t *p = buf;
	int ret = 0;

	if (!dk->up) {
		return -ENODEV;
	}
	if (lba + count > dk->id.sectors) {
		return -EINVAL;
	}
	while (count != 0) {
		uint32_t n = MIN(count, max);

		if (write) {
			memcpy(data->buf, p, (size_t)n * ss);
		}
		ret = ahci_exec(dev, dk->pmp, write ? ATA_WRITE_DMA_EXT : ATA_READ_DMA_EXT, 0, lba,
				(uint16_t)n, data->buf, (size_t)n * ss, write, NULL);
		if (ret != 0) {
			break;
		}
		if (!write) {
			memcpy(p, data->buf, (size_t)n * ss);
		}
		p += (size_t)n * ss;
		lba += n;
		count -= n;
	}
	return ret;
}

/* ------------------------------------------------------------------------ */
/* disk access */

static struct ahci_disk *to_dk(struct disk_info *disk)
{
	return CONTAINER_OF(disk, struct ahci_disk, disk);
}

static int ahci_disk_status(struct disk_info *disk)
{
	return to_dk(disk)->up ? DISK_STATUS_OK : DISK_STATUS_NOMEDIA;
}

static int ahci_disk_init(struct disk_info *disk)
{
	const struct device *dev = disk->dev;
	struct ahci_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->link) {
		(void)ahci_port_up(dev, false);
	}
	ret = to_dk(disk)->up ? 0 : -ENODEV;
	k_mutex_unlock(&data->lock);
	return ret;
}

static int ahci_disk_read(struct disk_info *disk, uint8_t *buf, uint32_t sector, uint32_t count)
{
	struct ahci_data *data = disk->dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = ahci_rw(disk->dev, to_dk(disk), sector, count, buf, false);
	k_mutex_unlock(&data->lock);
	return ret;
}

static int ahci_disk_write(struct disk_info *disk, const uint8_t *buf, uint32_t sector,
			   uint32_t count)
{
	struct ahci_data *data = disk->dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = ahci_rw(disk->dev, to_dk(disk), sector, count, (void *)buf, true);
	k_mutex_unlock(&data->lock);
	return ret;
}

static int ahci_disk_ioctl(struct disk_info *disk, uint8_t cmd, void *buff)
{
	struct ahci_data *data = disk->dev->data;
	struct ahci_disk *dk = to_dk(disk);
	int ret = 0;

	switch (cmd) {
	case DISK_IOCTL_CTRL_INIT:
		return ahci_disk_init(disk);
	case DISK_IOCTL_GET_SECTOR_COUNT:
		if (buff == NULL || !dk->up) {
			return -EINVAL;
		}
		*(uint32_t *)buff = (uint32_t)MIN(dk->id.sectors, (uint64_t)UINT32_MAX);
		return 0;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		if (buff == NULL || !dk->up) {
			return -EINVAL;
		}
		*(uint32_t *)buff = dk->id.sector_size;
		return 0;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		if (buff == NULL) {
			return -EINVAL;
		}
		*(uint32_t *)buff = MAX(1U, dk->id.phys_sector_size / MAX(1U, dk->id.sector_size));
		return 0;
	case DISK_IOCTL_CTRL_SYNC:
	case DISK_IOCTL_CTRL_DEINIT:
		k_mutex_lock(&data->lock, K_FOREVER);
		if (dk->up) {
			ret = ahci_exec(disk->dev, dk->pmp, ATA_FLUSH_CACHE_EXT, 0, 0, 0, NULL, 0,
					false, NULL);
		}
		k_mutex_unlock(&data->lock);
		return ret;
	default:
		return -EINVAL;
	}
}

static const struct disk_operations ahci_disk_ops = {
	.init = ahci_disk_init,
	.status = ahci_disk_status,
	.read = ahci_disk_read,
	.write = ahci_disk_write,
	.ioctl = ahci_disk_ioctl,
};

/* Each device under its own name: "sata0" for port 0, "sata<n>" behind a PM */
static void disk_register(const struct device *dev, struct ahci_disk *dk)
{
	int ret;

	if (dk->registered) {
		return;
	}
	dk->disk.name = dk->name;
	dk->disk.ops = &ahci_disk_ops;
	dk->disk.dev = dev;
	ret = disk_access_register(&dk->disk);
	if (ret != 0) {
		LOG_ERR("%s: disk register: %d", dk->name, ret);
		return;
	}
	dk->registered = true;
}

static void disk_names(const struct device *dev)
{
	const struct ahci_config *cfg = dev->config;
	struct ahci_data *data = dev->data;
	size_t base = strlen(cfg->disk_name);

	while (base > 0 && cfg->disk_name[base - 1] >= '0' && cfg->disk_name[base - 1] <= '9') {
		base--;
	}
	for (int i = 0; i < AHCI_MAX_DEV; i++) {
		data->d[i].pmp = i;
		if (i == 0) {
			strncpy(data->d[i].name, cfg->disk_name, sizeof(data->d[i].name) - 1);
		} else {
			snprintk(data->d[i].name, sizeof(data->d[i].name), "%.*s%d", (int)base,
				 cfg->disk_name, i);
		}
	}
}

/* ------------------------------------------------------------------------ */

static void ahci_isr(const struct device *dev)
{
	struct ahci_data *data = dev->data;
	uint32_t is = ahci_rd(dev, AHCI_IS);
	uint32_t pis;

	if (is == 0) {
		return;
	}
	pis = ahci_rd(dev, P0(AHCI_PxIS));
	ahci_wr(dev, P0(AHCI_PxIS), pis);
	ahci_wr(dev, AHCI_IS, is);
	data->last_is |= pis;
	data->irqs++;
	k_sem_give(&data->done);
}

#if defined(CONFIG_AHCI_BOOT_PROBE)
static void ahci_probe_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct ahci_data *data = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->link) {
		(void)ahci_port_up(dev, false);
	}
	k_mutex_unlock(&data->lock);
}
#endif

static int ahci_init_hba(const struct device *dev)
{
	const struct ahci_config *cfg = dev->config;
	struct ahci_data *data = dev->data;
	uint32_t vs;
	int ret;

	ret = clock_control_on(cfg->clk_dev, (clock_control_subsys_t)cfg->clk_id);
	if (ret != 0 && ret != -EALREADY && ret != -ENOSYS) {
		LOG_WRN("clock on: %d", ret);
	}

	/* Vendor-specific setting before the PHY (U-Boot order, Linux ahci_mvebu) */
	ahci_wr(dev, A3700_VENDOR_ADDR, 0);
	ahci_rmw(dev, A3700_VENDOR_DATA, 0, BIT(6));

	data->comphy_ret = ahci_comphy_power_on(dev);
	if (data->comphy_ret != 0) {
		LOG_WRN("COMPHY lane %u SATA power on: %ld", cfg->comphy_lane, data->comphy_ret);
	}

	/* HBA reset, AHCI mode */
	ahci_rmw(dev, AHCI_GHC, 0, AHCI_GHC_AE);
	ahci_rmw(dev, AHCI_GHC, 0, AHCI_GHC_HR);
	if (!wait_reg(dev, AHCI_GHC, AHCI_GHC_HR, 0, 1000)) {
		LOG_ERR("HBA reset does not complete");
		return -EIO;
	}
	ahci_rmw(dev, AHCI_GHC, 0, AHCI_GHC_AE);

	data->cap = ahci_rd(dev, AHCI_CAP);
	vs = ahci_rd(dev, AHCI_VS);
	if ((ahci_rd(dev, AHCI_PI) & BIT(0)) == 0) {
		LOG_ERR("port 0 not implemented (PI %08x)", ahci_rd(dev, AHCI_PI));
		return -ENODEV;
	}
	LOG_INF("AHCI %x.%x, CAP %08x, %u slots, ISS %lu%s", vs >> 16, (vs >> 4) & 0xff,
		data->cap, (uint32_t)FIELD_GET(AHCI_CAP_NCS, data->cap) + 1,
		FIELD_GET(AHCI_CAP_ISS, data->cap),
		(data->cap & AHCI_CAP_S64A) != 0 ? ", 64-bit" : "");

	ahci_wr(dev, P0(AHCI_PxIE), 0);
	ahci_wr(dev, AHCI_IS, UINT32_MAX);
	cfg->irq_config(dev);
	ahci_rmw(dev, AHCI_GHC, 0, AHCI_GHC_IE);
	data->hba_ok = true;
	return 0;
}

static int ahci_init(const struct device *dev)
{
	struct ahci_data *data = dev->data;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	data->dev = dev;
	k_mutex_init(&data->lock);
	k_sem_init(&data->done, 0, 1);

	ret = ahci_init_hba(dev);
	if (ret != 0) {
		return 0; /* the shell can still look at the registers */
	}

	/* port 0 right away (mounts bring the link up), the others when found */
	disk_names(dev);
	disk_register(dev, &data->d[0]);
	return 0;
}

#define AHCI_STACK 2048

#define AHCI_PROBE_THREAD(n)                                                                       \
	IF_ENABLED(CONFIG_AHCI_BOOT_PROBE,                                                         \
		   (K_THREAD_STACK_DEFINE(ahci_stack_##n, AHCI_STACK);                             \
		    static struct k_thread ahci_thread_##n;                                        \
		    static int ahci_start_##n(void)                                                \
		    {                                                                              \
			    const struct device *dev = DEVICE_DT_INST_GET(n);                      \
			    struct ahci_data *data = dev->data;                                    \
			    if (data->hba_ok) {                                                    \
				    k_thread_create(&ahci_thread_##n, ahci_stack_##n, AHCI_STACK,  \
						    ahci_probe_thread, (void *)dev, NULL, NULL,    \
						    K_LOWEST_APPLICATION_THREAD_PRIO, 0,           \
						    K_NO_WAIT);                                    \
				    k_thread_name_set(&ahci_thread_##n, "ahci_probe");             \
			    }                                                                      \
			    return 0;                                                              \
		    }                                                                              \
		    SYS_INIT(ahci_start_##n, APPLICATION, 0);))

#define AHCI_DEFINE(n)                                                                             \
	static struct ahci_cmd_hdr ahci_clb_##n[32] __aligned(1024);                               \
	static uint8_t ahci_rfis_##n[256] __aligned(256);                                          \
	static struct ahci_cmd_tbl ahci_tbl_##n __aligned(128);                                    \
	static uint8_t ahci_buf_##n[CONFIG_AHCI_BOUNCE_SECTORS * 512] __aligned(64);               \
	static void ahci_irq_config_##n(const struct device *dev)                                  \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), ahci_isr,                   \
			    DEVICE_DT_INST_GET(n), DT_INST_IRQ(n, flags));                         \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	static const struct ahci_config ahci_cfg_##n = {                                           \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                  \
		.clk_id = DT_INST_CLOCKS_CELL(n, index),                                           \
		.disk_name = DT_INST_PROP(n, disk_name),                                           \
		.comphy_lane = DT_INST_PROP(n, marvell_comphy_lane),                               \
		.comphy_invert = DT_INST_PROP(n, marvell_comphy_polarity_invert),                  \
		.irq_config = ahci_irq_config_##n,                                                 \
	};                                                                                         \
	static struct ahci_data ahci_data_##n = {                                                  \
		.clb = ahci_clb_##n,                                                               \
		.rfis = ahci_rfis_##n,                                                             \
		.tbl = &ahci_tbl_##n,                                                              \
		.buf = ahci_buf_##n,                                                               \
		.buf_size = sizeof(ahci_buf_##n),                                                  \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, ahci_init, NULL, &ahci_data_##n, &ahci_cfg_##n, POST_KERNEL,      \
			      CONFIG_AHCI_INIT_PRIORITY, NULL);                                    \
	AHCI_PROBE_THREAD(n)

DT_INST_FOREACH_STATUS_OKAY(AHCI_DEFINE)

const struct device *ahci_first(void)
{
	return DEVICE_DT_GET_OR_NULL(DT_COMPAT_GET_ANY_STATUS_OKAY(DT_DRV_COMPAT));
}
