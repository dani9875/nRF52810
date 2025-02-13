# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0
#
# This CMake file is picked by the Zephyr build system because it is defined
# as the module CMake entry point (see zephyr/module.yml).

cmake_minimum_required(VERSION 3.22)
project(bt_hid_opt_mouse)

# This is needed so that custom driver classes using system calls are taken into
# account
zephyr_syscall_include_directories(include)
zephyr_include_directories(include)

set(APP_DIR ${CMAKE_SOURCE_DIR}/app)

add_subdirectory(drivers)
add_subdirectory(lib)
