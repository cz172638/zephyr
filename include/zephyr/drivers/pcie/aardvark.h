/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Armada 3700 (Aardvark) PCIe controller role selection.
 *
 * The controller is either a root complex or an endpoint. With both
 * drivers built in, the role comes from the setting "advk/mode" ("rc" or
 * "ep"), read once at boot; "advk mode rc|ep" in the shell stores it for
 * the next boot.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PCIE_AARDVARK_H_
#define ZEPHYR_INCLUDE_DRIVERS_PCIE_AARDVARK_H_

#ifdef __cplusplus
extern "C" {
#endif

/** Role of the Aardvark controller */
enum advk_mode {
	/** Endpoint (reset default of the hardware) */
	ADVK_MODE_EP,
	/** Root complex */
	ADVK_MODE_RC,
};

#if defined(CONFIG_PCIE_AARDVARK_MODE)
/** @brief Role chosen for this boot. */
enum advk_mode advk_mode_get(void);
#else
static inline enum advk_mode advk_mode_get(void)
{
#if defined(CONFIG_PCIE_AARDVARK)
	return ADVK_MODE_RC;
#else
	return ADVK_MODE_EP;
#endif
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PCIE_AARDVARK_H_ */
