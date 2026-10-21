# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

if(CONFIG_SOC_ARMADA3720_CM3)
  zephyr_sources(soc_cm3.c cm3_entry.S)
  zephyr_sources_ifdef(CONFIG_SOC_ARMADA3720_CM3_CONSOLE_BRIDGE cm3_console.c)
  zephyr_sources_ifdef(CONFIG_SOC_ARMADA3720_CM3_FLASH_SERVER cm3_flash.c)
  zephyr_linker_sources(ROM_START SORT_KEY 0x1a3700entry cm3_entry.ld)
  if(CONFIG_SOC_ARMADA3720_CM3_SYS_INIT OR CONFIG_SOC_ARMADA3720_CM3_START_AP)
    zephyr_library_named(a3700_wtmi)
    zephyr_library_include_directories(wtmi wtmi/ddr)
    # Marvell code: keep its own conventions
    zephyr_library_compile_options(-Wno-unused-variable -Wno-unused-function -Wno-sign-compare
      -Wno-unused-but-set-variable -Wno-pointer-sign -Wno-implicit-fallthrough
      -Dasm=__asm__ -DDEBUG=0)
    zephyr_library_sources(wtmi/sys_init.c)
    zephyr_library_sources_ifdef(CONFIG_SOC_ARMADA3720_CM3_SYS_INIT
      wtmi/delay.c wtmi/clock.c wtmi/avs.c
      wtmi/ddr/ddr.c wtmi/ddr/ddr_support.c wtmi/ddr/ddrcore.c wtmi/ddr/dll_tuning.c
      wtmi/ddr/new_qs_gating.c wtmi/ddr/vref_training.c)
    zephyr_include_directories(wtmi)
  endif()
  set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
else()
  set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm64/scripts/linker.ld CACHE INTERNAL "")
endif()

zephyr_sources_ifdef(CONFIG_SOC_ARMADA37XX_CCI_SHELL cci400.c)
