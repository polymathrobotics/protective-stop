# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.12)
project(PSTOP C)

add_subdirectory(pstop)
add_subdirectory(transport)

add_executable(machine_app
  examples/machine/machine_app.c
)

target_include_directories(machine_app PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/pstop/include
  ${CMAKE_CURRENT_SOURCE_DIR}/transport/include
  include
)

target_link_libraries(machine_app PUBLIC
  pstop
  transport_udp
)

add_executable(client_app
  examples/client/client_app.c
)

target_include_directories(client_app PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/pstop/include
  ${CMAKE_CURRENT_SOURCE_DIR}/transport/include
  include
)

target_link_libraries(client_app PUBLIC
  pstop
  transport_udp
)
