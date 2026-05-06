# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.12)
project(PSTOPCore C)

find_package(unity REQUIRED)

add_library(pstop
  src/pstop/checksum.c
  src/pstop/device_id.c
  src/pstop/machine_client_data.c
  src/pstop/machine.c
  src/pstop/os.c
  src/pstop/protocol.c
  src/pstop/pstop_application.c
  src/pstop/pstop_client_data.c
  src/pstop/pstop_msg.c
  src/pstop/time.c
)

target_compile_options(pstop PRIVATE -Wall -fanalyzer)

target_include_directories(pstop PUBLIC include)

add_executable(pstop_test
  test/src/pstop/device_id_test.c
  test/src/pstop/machine_test.c
  test/src/pstop/machine_timeout_test.c
  test/src/pstop/pstop_client_data_test.c
  test/src/pstop/pstop_msg_test.c

  test/src/pstop/test_utils.c

  test/src/pstop/main.c
)

target_include_directories(pstop_test PUBLIC
  include
  test/include
)

target_link_libraries(pstop_test PUBLIC
  pstop
  unity
)
