/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_RESET_RK3399_CRU_RESET_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_RESET_RK3399_CRU_RESET_H_

/*
 * RK3399 soft resets, encoded as (SOFTRST_CON index * 16) + bit, which is
 * also the numbering used by the Linux rk3399-cru binding.
 */

/* CRU */
#define SRST_A_PCIE		128
#define SRST_P_PCIE		129
#define SRST_PCIE_CORE		130
#define SRST_PCIE_MGMT		131
#define SRST_PCIE_MGMT_STICKY	132
#define SRST_PCIE_PIPE		133
#define SRST_PCIE_PM		134
#define SRST_PCIEPHY		135
#define SRST_CM0S_NOC		177
#define SRST_CM0S		178
#define SRST_CM0S_DBG		179
#define SRST_CM0S_PO		180

/* PMUCRU */
#define SRST_H_CM0S		2
#define SRST_H_CM0S_NOC		3
#define SRST_DBG_CM0S		4
#define SRST_PO_CM0S		5

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_RESET_RK3399_CRU_RESET_H_ */
