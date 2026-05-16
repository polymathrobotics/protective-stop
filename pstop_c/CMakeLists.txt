# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.14)
project(PSTOP C)

option(PSTOP_ENABLE_COVERAGE "Build with BullseyeCoverage instrumentation" OFF)

if(PSTOP_ENABLE_COVERAGE)
  find_program(BULLSEYE_COV01     cov01     REQUIRED)
  find_program(BULLSEYE_COVSELECT covselect REQUIRED)
  find_program(BULLSEYE_COVSRC    covsrc    REQUIRED)
  find_program(BULLSEYE_COVHTML   covhtml   REQUIRED)
  set(PSTOP_COVFILE "${CMAKE_BINARY_DIR}/pstop.cov"
      CACHE FILEPATH "BullseyeCoverage .cov file location")
endif()

add_subdirectory(pstop)
add_subdirectory(transport)

if(PSTOP_ENABLE_COVERAGE)
  include(cmake/Coverage.cmake)
endif()

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
