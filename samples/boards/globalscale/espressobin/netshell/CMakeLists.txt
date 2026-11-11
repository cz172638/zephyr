# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(netshell)

target_sources(app PRIVATE src/main.c src/config.c src/top.c)
# TCAM rule API of the switch driver
target_include_directories(app PRIVATE ${ZEPHYR_BASE}/drivers/ethernet/phy)

# Public key allowed to log in, PEM/PKCS8, e.g.
#   ssh-keygen -e -m PKCS8 -f ~/.ssh/id_rsa.pub > authorized_key.pem
# taken from -DNETSHELL_AUTHORIZED_KEY=<file> or authorized_key.pem here.
if(NOT NETSHELL_AUTHORIZED_KEY)
  set(NETSHELL_AUTHORIZED_KEY ${CMAKE_CURRENT_SOURCE_DIR}/authorized_key.pem)
endif()

if(EXISTS ${NETSHELL_AUTHORIZED_KEY})
  set(gen_dir ${ZEPHYR_BINARY_DIR}/include/generated)
  generate_inc_file_for_target(app ${NETSHELL_AUTHORIZED_KEY} ${gen_dir}/authorized_key.inc)
  target_compile_definitions(app PRIVATE NETSHELL_HAVE_AUTHORIZED_KEY=1)
  message(STATUS "netshell: authorized key ${NETSHELL_AUTHORIZED_KEY}")
elseif(CONFIG_NETSHELL_PASSWORD STREQUAL "")
  message(FATAL_ERROR "netshell: give an authorized key (authorized_key.pem or "
                      "-DNETSHELL_AUTHORIZED_KEY=...) or CONFIG_NETSHELL_PASSWORD")
endif()

# Lab agent: built-in LLEXT self test (lab selftest, source in
# subsys/lab/ext), and debug modules: <image>-<name>.llext, built with
#   west build -d <dir> -t lab_mods
# without touching the firmware (zephyr.elf is only read, by the
# symbol check: a module that needs a symbol the firmware does not
# export means a new firmware image).
if(CONFIG_LAB_EXT)
  set(lab_ext_bin ${PROJECT_BINARY_DIR}/llext/lab_hello.llext)
  add_llext_target(lab_hello_ext
    OUTPUT  ${lab_ext_bin}
    SOURCES ${ZEPHYR_BASE}/subsys/lab/ext/lab_hello.c
  )
  generate_inc_file_for_target(app ${lab_ext_bin}
    ${ZEPHYR_BINARY_DIR}/include/generated/lab_hello.inc)
  target_sources(app PRIVATE src/lab_selftest.c src/dbg_exports.c)
  target_include_directories(app PRIVATE ${ZEPHYR_BASE}/drivers/pcie/endpoint)

  set(lab_img ${CONFIG_LAB_IMAGE})
  set(lab_mod_dir ${PROJECT_BINARY_DIR}/llext)
  set(epc_dir ${ZEPHYR_BASE}/drivers/pcie/endpoint)
  set(lab_mod_files)
  function(lab_dbg_module name)
    set(out ${lab_mod_dir}/${lab_img}-${name}.llext)
    add_llext_target(lab_mod_${name} OUTPUT ${out} SOURCES ${ARGN})
    get_target_property(lib lab_mod_${name} lib_target)
    target_include_directories(${lib} PRIVATE ${epc_dir})
    set(lab_mod_files ${lab_mod_files} ${out} PARENT_SCOPE)
    set(lab_mod_targets ${lab_mod_targets} lab_mod_${name} PARENT_SCOPE)
  endfunction()

  lab_dbg_module(irqtest ${epc_dir}/pcie_epc_aardvark_a3700_irqtest.c)
  lab_dbg_module(xortest ${epc_dir}/pcie_epc_aardvark_a3700_xortest.c)
  lab_dbg_module(loopback ${CMAKE_CURRENT_SOURCE_DIR}/dbg/pcie_loopback.c)
  lab_dbg_module(comphy ${CMAKE_CURRENT_SOURCE_DIR}/dbg/comphy.c)
  lab_dbg_module(netlb ${CMAKE_CURRENT_SOURCE_DIR}/dbg/netlb.c)
  lab_dbg_module(ibmap ${CMAKE_CURRENT_SOURCE_DIR}/dbg/ibmap.c)

  # with the firmware: build and name the modules
  add_custom_target(lab_mods_build ALL
    COMMAND ${CMAKE_COMMAND} -E copy ${lab_ext_bin} ${lab_mod_dir}/${lab_img}-lab_hello.llext
    DEPENDS ${lab_mod_targets} lab_hello_ext
    VERBATIM)
  # on their own (-t lab_mods): build, then check against the existing
  # zephyr.elf, which is not rebuilt
  add_custom_target(lab_mods
    COMMAND ${ZEPHYR_BASE}/scripts/lab/llext-symcheck.pl -nm ${CMAKE_NM}
            ${ZEPHYR_BINARY_DIR}/zephyr.elf ${lab_mod_files}
    DEPENDS lab_mods_build
    COMMENT "lab debug modules for image ${lab_img}, symbol check"
    VERBATIM)
endif()
