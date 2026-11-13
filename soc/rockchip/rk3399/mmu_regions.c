/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm64/arm_mmu.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

#include <soc.h>

#define RK3399_PCIE_OB_BASE 0xfa000000UL
#define RK3399_PCIE_OB_SIZE MB(32)

/*
 * System controllers accessed by SoC code through fixed addresses. Drivers
 * map their own registers through the device MMIO API.
 *
 * The PCIe outbound window is identity mapped as normal non-cacheable
 * memory: pcie_ep_xfer_data_memcpy() uses the physical address returned by
 * pcie_ep_map_addr() as a pointer, and memcpy() may issue unaligned accesses.
 */
static const struct arm_mmu_region mmu_regions[] = {
	MMU_REGION_FLAT_ENTRY("pcie_ob", RK3399_PCIE_OB_BASE, RK3399_PCIE_OB_SIZE,
			      MT_NORMAL_NC | MT_P_RW_U_NA | MT_DEFAULT_SECURE_STATE),
	MMU_REGION_FLAT_ENTRY("pmu_pmugrf_sgrf", RK3399_PMU_BASE, KB(192),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_DEFAULT_SECURE_STATE),
	MMU_REGION_FLAT_ENTRY("pmucru_cru_grf", RK3399_PMUCRU_BASE, KB(192),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_DEFAULT_SECURE_STATE),
#if defined(CONFIG_RK3399_EARLY_MARKS)
	MMU_REGION_FLAT_ENTRY("uart2_marks", 0xff1a0000UL, KB(4),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_DEFAULT_SECURE_STATE),
#endif
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};
