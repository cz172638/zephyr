# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lab_ext)

# the loader glue of the lab agent, without the network parts
target_sources(app PRIVATE src/main.c ${ZEPHYR_BASE}/subsys/lab/lab_llext.c)
target_include_directories(app PRIVATE ${ZEPHYR_BASE}/subsys/lab)

set(ext_bin ${PROJECT_BINARY_DIR}/llext/lab_hello.llext)
add_llext_target(lab_hello_ext
  OUTPUT  ${ext_bin}
  SOURCES ${ZEPHYR_BASE}/subsys/lab/ext/lab_hello.c
)
generate_inc_file_for_target(app ${ext_bin} ${ZEPHYR_BINARY_DIR}/include/generated/lab_hello.inc)
