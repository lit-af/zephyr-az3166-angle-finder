# Copyright (c) 2022 TOKITA Hiroshi <tokita.hiroshi@fujitsu.com>
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(accel_polling)

# Define the path to the zscilib module
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../modules/zscilib)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
