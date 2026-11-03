/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_DISK_AHCI_AHCI_H_
#define ZEPHYR_DRIVERS_DISK_AHCI_AHCI_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>

/* Generic host control */
#define AHCI_CAP         0x00
#define AHCI_CAP_NP      GENMASK(4, 0)
#define AHCI_CAP_NCS     GENMASK(12, 8)
#define AHCI_CAP_ISS     GENMASK(23, 20)
#define AHCI_CAP_SCLO    BIT(24)
#define AHCI_CAP_SSS     BIT(27)
#define AHCI_CAP_SNCQ    BIT(30)
#define AHCI_CAP_S64A    BIT(31)
#define AHCI_GHC         0x04
#define AHCI_GHC_HR      BIT(0)
#define AHCI_GHC_IE      BIT(1)
#define AHCI_GHC_AE      BIT(31)
#define AHCI_IS          0x08
#define AHCI_PI          0x0c
#define AHCI_VS          0x10
#define AHCI_CAP2        0x24

/* Armada 3700 vendor-specific indirect register (Linux ahci_mvebu) */
#define A3700_VENDOR_ADDR 0xa0
#define A3700_VENDOR_DATA 0xa4
/* COMPHY lane 2 indirect access (16-bit PHY registers, SATA at +0x200) */
#define A3700_PHY_ADDR    0x178
#define A3700_PHY_DATA    0x17c
#define A3700_PHY_SATA    0x200

/* Port 0 */
#define AHCI_P0          0x100
#define AHCI_PxCLB       0x00
#define AHCI_PxCLBU      0x04
#define AHCI_PxFB        0x08
#define AHCI_PxFBU       0x0c
#define AHCI_PxIS        0x10
#define AHCI_PxIE        0x14
#define AHCI_PxCMD       0x18
#define AHCI_PxTFD       0x20
#define AHCI_PxSIG       0x24
#define AHCI_PxSSTS      0x28
#define AHCI_PxSCTL      0x2c
#define AHCI_PxSERR      0x30
#define AHCI_PxSACT      0x34
#define AHCI_PxCI        0x38

#define PxCMD_ST         BIT(0)
#define PxCMD_SUD        BIT(1)
#define PxCMD_POD        BIT(2)
#define PxCMD_CLO        BIT(3)
#define PxCMD_FRE        BIT(4)
#define PxCMD_FR         BIT(14)
#define PxCMD_CR         BIT(15)

#define PxIS_DHRS        BIT(0)
#define PxIS_PSS         BIT(1)
#define PxIS_DSS         BIT(2)
#define PxIS_SDBS        BIT(3)
#define PxIS_PCS         BIT(6)
#define PxIS_PRCS        BIT(22)
#define PxIS_IFS         BIT(27)
#define PxIS_HBDS        BIT(28)
#define PxIS_HBFS        BIT(29)
#define PxIS_TFES        BIT(30)
#define PxIS_ERRORS      (BIT(30) | BIT(29) | BIT(28) | BIT(27) | BIT(26) | BIT(24) | BIT(23))

#define TFD_ERR          BIT(0)
#define TFD_DRQ          BIT(3)
#define TFD_BSY          BIT(7)

#define SSTS_DET(v)      ((v) & 0xf)
#define SSTS_SPD(v)      (((v) >> 4) & 0xf)
#define SSTS_IPM(v)      (((v) >> 8) & 0xf)

/* ATA commands */
#define ATA_READ_DMA_EXT       0x25
#define ATA_WRITE_DMA_EXT      0x35
#define ATA_FLUSH_CACHE_EXT    0xea
#define ATA_IDENTIFY           0xec
#define ATA_STANDBY_IMMEDIATE  0xe0
#define ATA_IDLE_IMMEDIATE     0xe1
#define ATA_CHECK_POWER_MODE   0xe5
#define ATA_SMART              0xb0

/* DMA structures */
struct ahci_cmd_hdr {
	uint32_t opts;	/* CFL[4:0], A, W, P, R, B, C, PMP, PRDTL[31:16] */
	uint32_t prdbc;
	uint32_t ctba;
	uint32_t ctbau;
	uint32_t rsvd[4];
};

struct ahci_prd {
	uint32_t dba;
	uint32_t dbau;
	uint32_t rsvd;
	uint32_t dbc;	/* byte count - 1, bit 31 interrupt */
};

struct ahci_cmd_tbl {
	uint8_t cfis[64];
	uint8_t acmd[16];
	uint8_t rsvd[48];
	struct ahci_prd prd[1];
};

/* Identify data the driver keeps */
struct ahci_ident {
	char model[41];
	char serial[21];
	char fw[9];
	uint64_t sectors;
	uint32_t sector_size;
	uint32_t phys_sector_size;
	uint16_t sata_cap;	/* word 76 */
	uint16_t rpm;		/* word 217 */
	bool lba48;
	bool ncq;
	bool smart;
	bool trim;
};

struct ahci_config {
	DEVICE_MMIO_ROM;
	const struct device *clk_dev;
	uintptr_t clk_id;
	const char *disk_name;
	uint8_t comphy_lane;
	uint8_t comphy_invert;
	void (*irq_config)(const struct device *dev);
};

struct ahci_data {
	DEVICE_MMIO_RAM;
	const struct device *dev;
	struct disk_info disk;
	struct k_mutex lock;
	struct k_sem done;
	struct ahci_cmd_hdr *clb;
	uint8_t *rfis;
	struct ahci_cmd_tbl *tbl;
	uint8_t *buf;
	size_t buf_size;
	uint32_t cap;
	long comphy_ret;
	volatile uint32_t last_is;
	uint32_t irqs;
	uint32_t cmds;
	uint32_t errors;
	bool hba_ok;
	bool up;
	struct ahci_ident id;
};

/* Shared with the shell */
uint32_t ahci_rd(const struct device *dev, uint32_t off);
void ahci_wr(const struct device *dev, uint32_t off, uint32_t v);
uint16_t ahci_phy_rd(const struct device *dev, uint16_t reg);
void ahci_phy_wr(const struct device *dev, uint16_t reg, uint16_t v);
long ahci_comphy_power_on(const struct device *dev);
int ahci_port_up(const struct device *dev, bool comreset);
int ahci_exec(const struct device *dev, uint8_t cmd, uint16_t feat, uint64_t lba,
	      uint16_t count, void *buf, size_t len, bool write, uint8_t *d2h);
int ahci_rw(const struct device *dev, uint64_t lba, uint32_t count, void *buf, bool write);
int ahci_identify(const struct device *dev);
const struct device *ahci_first(void);

#endif /* ZEPHYR_DRIVERS_DISK_AHCI_AHCI_H_ */
