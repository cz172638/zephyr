# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(netshell)

target_sources(app PRIVATE src/main.c src/config.c)
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
