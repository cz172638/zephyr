# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rk3308_usb_probe)

include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)
target_sources(app PRIVATE src/main.c src/cpufreq.c)

target_sources_ifdef(CONFIG_DAP_BACKEND_TCP app PRIVATE src/dap.c)

if(CONFIG_SSH_SERVER)
  target_sources(app PRIVATE src/ssh.c)
  if(NOT CONFIG_USB_PROBE_SSH_HOST_KEY STREQUAL "")
    get_filename_component(host_key ${CONFIG_USB_PROBE_SSH_HOST_KEY}
                           ABSOLUTE BASE_DIR ${APPLICATION_SOURCE_DIR})
    set(gen_dir ${ZEPHYR_BINARY_DIR}/include/generated)
    generate_inc_file_for_target(app ${host_key} ${gen_dir}/ssh_host_key.der.inc)
    target_compile_definitions(app PRIVATE USB_PROBE_SSH_HOST_KEY_INC)
  endif()
endif()
