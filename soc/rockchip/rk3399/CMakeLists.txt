# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

if(CONFIG_ARM64)
  zephyr_sources(soc_a.c mmu_regions.c)
  zephyr_sources_ifdef(CONFIG_RK3399_REMOTE_CORE_CONTROL rk3399_cores.c)
  zephyr_sources_ifdef(CONFIG_RK3399_NOC_ERRLOG noc_errlog.c)
  set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm64/scripts/linker.ld CACHE INTERNAL "")
else()
  set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
endif()
