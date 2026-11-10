# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lab_ext)

# the loader glue of the lab agent, without the network parts
target_sources(app PRIVATE src/main.c ${ZEPHYR_BASE}/subsys/lab/lab_llext.c
  ${ZEPHYR_BASE}/subsys/lab/lab_mod.c ${ZEPHYR_BASE}/subsys/lab/lab_exports.c)
target_include_directories(app PRIVATE ${ZEPHYR_BASE}/subsys/lab)

set(gen ${ZEPHYR_BINARY_DIR}/include/generated)
add_llext_target(lab_hello_ext OUTPUT ${PROJECT_BINARY_DIR}/llext/lab_hello.llext
  SOURCES ${ZEPHYR_BASE}/subsys/lab/ext/lab_hello.c)
generate_inc_file_for_target(app ${PROJECT_BINARY_DIR}/llext/lab_hello.llext ${gen}/lab_hello.inc)

add_llext_target(mod_ext OUTPUT ${PROJECT_BINARY_DIR}/llext/mod.llext SOURCES src/mod_ext.c)
generate_inc_file_for_target(app ${PROJECT_BINARY_DIR}/llext/mod.llext ${gen}/mod.inc)

add_llext_target(mod99_ext OUTPUT ${PROJECT_BINARY_DIR}/llext/mod99.llext SOURCES src/mod_ext.c)
get_target_property(lib99 mod99_ext lib_target)
target_compile_definitions(${lib99} PRIVATE LAB_TEST_OTHER_IMAGE)
generate_inc_file_for_target(app ${PROJECT_BINARY_DIR}/llext/mod99.llext ${gen}/mod99.inc)
