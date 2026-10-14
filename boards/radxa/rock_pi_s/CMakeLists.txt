# Copyright (c) 2026 Jiri Kastner <cz172638@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Optionally wrap zephyr.bin into zephyr.itb, a FIT image the Rockchip U-Boot
# SPL loads in place of u-boot.itb: SPL -> TF-A BL31 -> Zephyr. No OP-TEE, no
# U-Boot proper.
#
# Inputs, as CMake cache variables or environment variables:
#   BL31      TF-A bl31.elf                                      (required)
#   MKIMAGE   mkimage binary (default: UBOOT_DIR/tools/mkimage, then PATH)
#   UBOOT_DIR U-Boot build dir, only used to locate mkimage      (optional)
#   BL31_FDT  DTB image for BL33, only if your SPL requires one  (optional)

foreach(var BL31 MKIMAGE UBOOT_DIR BL31_FDT)
  if(NOT DEFINED ${var} AND DEFINED ENV{${var}})
    set(${var} $ENV{${var}})
  endif()
endforeach()

if(NOT BL31)
  message(STATUS "rock_pi_s: BL31 not set, not creating zephyr.itb")
  return()
endif()

if(NOT MKIMAGE)
  if(UBOOT_DIR AND EXISTS ${UBOOT_DIR}/tools/mkimage)
    set(MKIMAGE ${UBOOT_DIR}/tools/mkimage)
  else()
    find_program(MKIMAGE mkimage REQUIRED)
  endif()
endif()

set(itb_args
  --zephyr ${CMAKE_BINARY_DIR}/zephyr/${KERNEL_BIN_NAME}
  --load ${CONFIG_SRAM_BASE_ADDRESS}
  --bl31 ${BL31}
  --mkimage ${MKIMAGE}
  --its ${CMAKE_BINARY_DIR}/zephyr/zephyr.its
  -o ${CMAKE_BINARY_DIR}/zephyr/zephyr.itb
)
if(BL31_FDT)
  list(APPEND itb_args --fdt ${BL31_FDT})
endif()

message(STATUS "rock_pi_s: zephyr.itb with BL31=${BL31}")

set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
  COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/support/mkitb.py ${itb_args}
)
set_property(GLOBAL APPEND PROPERTY extra_post_build_byproducts
  ${CMAKE_BINARY_DIR}/zephyr/zephyr.itb
  ${CMAKE_BINARY_DIR}/zephyr/zephyr.its
)
