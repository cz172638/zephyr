/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PCIe Advanced Error Reporting below a root port
 *
 * Controller neutral: configuration space through pcie_ctrl_conf_read()
 * and pcie_ctrl_conf_write() of a root complex controller. The tree is
 * walked through the bridges' bus numbers, only device 0 below root and
 * downstream ports, so the walk itself does not provoke Unsupported
 * Requests.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PCIE_AER_H_
#define ZEPHYR_INCLUDE_DRIVERS_PCIE_AER_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/pcie/pcie.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Device/Port Type of the PCI Express capability */
#define PCIE_AER_TYPE_EP        0
#define PCIE_AER_TYPE_LEGACY_EP 1
#define PCIE_AER_TYPE_RP        4
#define PCIE_AER_TYPE_UP        5
#define PCIE_AER_TYPE_DP        6
#define PCIE_AER_TYPE_PCIE_PCI  7
#define PCIE_AER_TYPE_PCI_PCIE  8
#define PCIE_AER_TYPE_RCIEP     9
#define PCIE_AER_TYPE_RCEC      10
#define PCIE_AER_TYPE_NONE      0xff /* no PCI Express capability */

/* pcie_aer_setup() flags */
#define PCIE_AER_UR_EP     BIT(0) /* Unsupported Request reporting on endpoints */
#define PCIE_AER_UR_BRIDGE BIT(1) /* ... and on ports / bridges */
#define PCIE_AER_ECRC      BIT(2) /* ECRC generation and check where capable */

/* Severity of a received error message */
enum pcie_aer_sev {
	PCIE_AER_COR,
	PCIE_AER_NONFATAL,
	PCIE_AER_FATAL,
};

/** State of one function */
struct pcie_aer_info {
	uint8_t type;	   /**< PCIE_AER_TYPE_* */
	bool bridge;	   /**< type 1 header */
	uint16_t exp;	   /**< PCI Express capability offset, 0 none */
	uint16_t aer;	   /**< AER extended capability offset, 0 none */
	uint16_t cmd;	   /**< Command */
	uint16_t sta;	   /**< Status */
	uint16_t bctl;	   /**< Bridge Control (bridges) */
	uint16_t devctl;   /**< Device Control */
	uint16_t devsta;   /**< Device Status */
	uint32_t ue, uem, ues;	/**< Uncorrectable status, mask, severity */
	uint32_t ce, cem;	/**< Correctable status, mask */
	uint32_t capctl;	/**< AER Capabilities and Control */
	uint32_t hdr[4];	/**< Header Log */
	uint32_t root_cmd;	/**< Root Error Command (root ports) */
	uint32_t root_sta;	/**< Root Error Status */
	uint32_t root_src;	/**< Error Source Identification */
};

/** One error message the root port received, with the source's status */
struct pcie_aer_event {
	pcie_bdf_t bdf;	/**< source; PCIE_BDF_NONE when not identified */
	enum pcie_aer_sev sev;
	bool multiple;	/**< more of the kind arrived before this was serviced */
	uint32_t ue, ce;
	uint32_t hdr[4];
	bool hdr_valid;
};

/** Messages counted per source */
struct pcie_aer_stat {
	pcie_bdf_t bdf;
	uint32_t cnt[3]; /**< by enum pcie_aer_sev */
	uint32_t ue_acc, ce_acc; /**< status bits seen, accumulated */
	int64_t last_ms;
};

typedef void (*pcie_aer_visit_t)(const struct device *dev, pcie_bdf_t bdf, void *user);
typedef void (*pcie_aer_report_t)(const struct device *dev, const struct pcie_aer_event *ev);

/** Names of the uncorrectable (ue) and correctable (ce) status bits, NULL reserved */
extern const char *const pcie_aer_ue_name[32];
extern const char *const pcie_aer_ce_name[32];
/** Name of a PCIE_AER_TYPE_* */
const char *pcie_aer_type_name(uint8_t type);
/** "Name Name ..." of the set bits into buf */
int pcie_aer_bits_str(char *buf, size_t len, uint32_t v, const char *const *names);

/** Every function at and below root, depth first (root first) */
int pcie_aer_walk(const struct device *dev, pcie_bdf_t root, pcie_aer_visit_t visit,
		  void *user);

/** Read the error state of one function; -ENODEV when absent */
int pcie_aer_info(const struct device *dev, pcie_bdf_t bdf, struct pcie_aer_info *info);

/** Clear the latched status of one function (RW1C bits) */
void pcie_aer_clear(const struct device *dev, pcie_bdf_t bdf);

/**
 * Enable error reporting on every function of the tree, forwarding on
 * every bridge, and the root port's interrupts for received messages.
 * Latched status is cleared. Returns the number of functions configured.
 */
int pcie_aer_setup(const struct device *dev, pcie_bdf_t root, uint32_t flags);

/**
 * Service the root port (from a thread, it makes config requests):
 * Root Error Status and Source ID, then the source's AER status, which
 * is cleared. When the root port latched no source, the tree is
 * searched for functions with status set. report (may be NULL) is
 * called for every event; the per source counters are updated.
 * Returns the number of events.
 */
int pcie_aer_root_service(const struct device *dev, pcie_bdf_t root, pcie_aer_report_t report);

/** Counters: entry i (NULL past the end), and reset */
const struct pcie_aer_stat *pcie_aer_stat_get(int i);
void pcie_aer_stat_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PCIE_AER_H_ */
