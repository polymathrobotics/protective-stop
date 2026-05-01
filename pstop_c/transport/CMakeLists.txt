# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.12)
project(transport_udp C)

add_library(transport_udp
  src/transport/udp/udp_transport.c
)

target_compile_options(transport_udp PRIVATE -Wall)

target_include_directories(transport_udp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../pstop/include>
  include
)
