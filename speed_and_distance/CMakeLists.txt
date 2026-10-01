# Copyright (c) 2026 Semtech Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Adapted from flrc_throughput/CMakeLists.txt (adds the ranging module).

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(speed_and_distance)

target_include_directories(app PRIVATE
    src
    src/oled_display
)

file(GLOB OLED_DISPLAY src/oled_display/*.c)

target_sources(app PRIVATE
    ${OLED_DISPLAY}
    src/app_flrc.c
    src/app_ranging.c
    src/main.c
)
