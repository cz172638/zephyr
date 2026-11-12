/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Role of a dual-mode PCIe controller (root complex or endpoint).
 *
 * For SoCs whose one PCIe controller can be either (RK3399, ...): both
 * drivers are built in, the role for this boot is read once before they
 * initialize and the one not picked leaves the hardware alone.
 * "pcie controller mode rc|ep" stores the role for the next boot.
 *
 * Storage: settings ("pcie/mode") when CONFIG_PCIE_ROLE_SETTINGS, else a
 * retention register (CONFIG_PCIE_ROLE_RETENTION_REG) that survives a
 * warm reset but not a power cycle, else only the build default.
 * (Armada 3700 keeps its own "advk/mode", see aardvark.h.)
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PCIE_ROLE_H_
#define ZEPHYR_INCLUDE_DRIVERS_PCIE_ROLE_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

enum pcie_role {
	PCIE_ROLE_EP,
	PCIE_ROLE_RC,
};

/** @brief Role chosen for this boot. */
enum pcie_role pcie_role_get(void);

/**
 * @brief Is this boot's PCIe controller a root complex?
 *
 * Whichever role mechanism the build has (PCIE_AARDVARK_MODE, PCIE_ROLE);
 * false on endpoint-only builds. For endpoint functions that bow out in
 * the root complex role.
 */
static inline bool pcie_ctrl_is_rc(void);

/** @brief Store the role for the next boot; -ENOTSUP without storage. */
int pcie_role_save(enum pcie_role role);

/** @brief Saved role; false when nothing (valid) is saved. */
bool pcie_role_saved(enum pcie_role *role);

/** @brief Where the role is kept: "settings", "retention register", "none". */
const char *pcie_role_storage(void);

#if defined(CONFIG_PCIE_AARDVARK_MODE)
#include <zephyr/drivers/pcie/aardvark.h>
#endif

static inline bool pcie_ctrl_is_rc(void)
{
#if defined(CONFIG_PCIE_AARDVARK_MODE)
	return advk_mode_get() == ADVK_MODE_RC;
#elif defined(CONFIG_PCIE_ROLE)
	return pcie_role_get() == PCIE_ROLE_RC;
#else
	return false;
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PCIE_ROLE_H_ */
