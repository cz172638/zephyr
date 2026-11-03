/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Controller-neutral "pcie" shell.
 *
 * Root complex and endpoint controller drivers register a backend; the
 * shell offers the same commands for all of them:
 *
 *   pcie controller [info|mode [rc|ep]|link [retrain]|scan|irq|id ...|dsn ...]
 *   pcie ls [-v]
 *   pcie cfg <b:d.f> [<off> [<len>]] | pcie cfg set <b:d.f> <off> <val>
 *   pcie cap ls|get|set <b:d.f> [<cap> [<off> [<val>]]]
 *   pcie debug ...     (driver specific, SHELL_SUBCMD_ADD((pcie_debug), ...))
 *
 * In root complex mode b:d.f are the devices below the controller; in
 * endpoint mode 0:0.<fn> are the local functions.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PCIE_PCIE_SHELL_H_
#define ZEPHYR_INCLUDE_DRIVERS_PCIE_PCIE_SHELL_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/pcie/pcie.h>
#include <zephyr/shell/shell.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Controller role */
enum pcie_shell_role {
	PCIE_SHELL_ROLE_RC,
	PCIE_SHELL_ROLE_EP,
};

/** Identity of a function (endpoint) or of the root port (root complex) */
struct pcie_shell_id {
	uint16_t vid, did, svid, ssid;
	/** class code (base << 16 | sub << 8 | prog-if); UINT32_MAX: unchanged */
	uint32_t class_code;
};

/** Backend of one controller; all callbacks optional except cfg_read */
struct pcie_shell_ctrl {
	/** Short name shown by "pcie controller" (not a command) */
	const char *name;
	const struct device *dev;
	enum pcie_shell_role role;
	/** Is this backend the one in use this boot? */
	bool (*active)(const struct device *dev);
	/** Free-form state: link, LTSSM, windows, ... */
	void (*info)(const struct device *dev, const struct shell *sh);
	/** Role for the next boot; NULL when the role is fixed */
	int (*mode_set)(const struct device *dev, enum pcie_shell_role role);
	/** Saved role, false when none */
	bool (*mode_saved)(const struct device *dev, enum pcie_shell_role *role);
	/** Link state; retrain: take it down and up again (RC: PERST#, rescan) */
	int (*link)(const struct device *dev, const struct shell *sh, bool retrain);
	/** Enumerate again (root complex) */
	int (*scan)(const struct device *dev, const struct shell *sh);
	/** Interrupt counters */
	void (*irq)(const struct device *dev, const struct shell *sh);
	/** Identity of b:d.f (RC: the root port 0:0.0, EP: a local function) */
	int (*id_get)(const struct device *dev, pcie_bdf_t bdf, struct pcie_shell_id *id);
	/** Change it now and keep it (save) for the next boots */
	int (*id_set)(const struct device *dev, pcie_bdf_t bdf, const struct pcie_shell_id *id);
	/** Forget the saved identity */
	int (*id_reset)(const struct device *dev, pcie_bdf_t bdf);
	/**
	 * Device Serial Number of b:d.f. arg NULL: show; "uid": from the
	 * board's unique ID; "new": random; "reset": forget the saved one;
	 * else a hex EUI-64. Changes are saved. *v: the DSN now, *src: where
	 * it came from (may stay NULL).
	 */
	int (*dsn)(const struct device *dev, pcie_bdf_t bdf, const char *arg, uint64_t *v,
		   const char **src);
	/** Configuration space access, reg = dword index; ~0 when absent */
	uint32_t (*cfg_read)(const struct device *dev, pcie_bdf_t bdf, unsigned int reg);
	void (*cfg_write)(const struct device *dev, pcie_bdf_t bdf, unsigned int reg, uint32_t v);
};

/**
 * @brief Register a controller backend (from the driver's init, also when
 * the controller is not used this boot).
 */
void pcie_shell_register(const struct pcie_shell_ctrl *ctrl);

/** @brief Parse "bus:dev.fn" (hex) */
int pcie_shell_parse_bdf(const char *s, pcie_bdf_t *bdf);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PCIE_PCIE_SHELL_H_ */
