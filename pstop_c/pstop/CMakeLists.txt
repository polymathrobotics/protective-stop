# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.14)
project(PSTOPCore C)

include(FetchContent)
fetchcontent_declare(unity
  GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
  GIT_TAG        v2.6.0
)
fetchcontent_makeavailable(unity)

# Tests #include <unity/unity.h>; upstream ships it as src/unity.h.
# Stage a unity/ subdir so the path resolves.
set(_unity_compat_dir "${CMAKE_BINARY_DIR}/unity_compat/unity")
file(MAKE_DIRECTORY "${_unity_compat_dir}")
foreach(_h unity.h unity_internals.h)
  configure_file(
    "${unity_SOURCE_DIR}/src/${_h}"
    "${_unity_compat_dir}/${_h}"
    COPYONLY
  )
endforeach()
target_include_directories(unity INTERFACE
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/unity_compat>
)

add_library(pstop
  src/pstop/checksum.c
  src/pstop/device_id.c
  src/pstop/endian.c
  src/pstop/machine.c
  src/pstop/os.c
  src/pstop/protocol.c
  src/pstop/protocol_data.c
  src/pstop/pstop_application.c
  src/pstop/pstop_remote_data.c
  src/pstop/pstop_msg.c
  src/pstop/time.c
)

target_compile_options(pstop PRIVATE -Wall -Wextra -Werror -fanalyzer)

target_include_directories(pstop PUBLIC include)

add_executable(pstop_test
  test/src/pstop/device_id_test.c
  test/src/pstop/endian_test.c
  test/src/pstop/machine_test.c
  test/src/pstop/machine_timeout_test.c
  test/src/pstop/protocol_test.c
  test/src/pstop/pstop_application_test.c
  test/src/pstop/pstop_remote_data_test.c
  test/src/pstop/pstop_msg_test.c

  test/src/pstop/main.c
)

target_compile_options(pstop_test PRIVATE -Wall -Wextra -Werror -fanalyzer)

target_include_directories(pstop_test PUBLIC
  include
)

target_link_libraries(pstop_test PUBLIC
  pstop
  unity
)

add_executable(pstop_requirements_test
  test/src/pstop/requirements/req_2_01_test.c
  test/src/pstop/requirements/req_2_02_test.c
  test/src/pstop/requirements/req_2_03_test.c

  test/src/pstop/requirements/req_2_06_test.c
  test/src/pstop/requirements/req_2_07_test.c
  test/src/pstop/requirements/req_2_08_test.c

  test/src/pstop/requirements/req_3_01_test.c
  test/src/pstop/requirements/req_3_02_test.c
  test/src/pstop/requirements/req_3_03_test.c
  test/src/pstop/requirements/req_3_04_test.c
  test/src/pstop/requirements/req_3_05_test.c
  test/src/pstop/requirements/req_3_06_test.c
  test/src/pstop/requirements/req_3_07_test.c
  test/src/pstop/requirements/req_3_08_test.c
  test/src/pstop/requirements/req_3_09_test.c
  test/src/pstop/requirements/req_3_10_test.c
  test/src/pstop/requirements/req_3_11_test.c
  test/src/pstop/requirements/req_3_12_test.c

  test/src/pstop/requirements/req_3_13_test.c


  test/src/pstop/requirements/req_3_14_test.c
  test/src/pstop/requirements/req_3_16_test.c

  test/src/pstop/requirements/test_utils.c

  test/src/pstop/requirements/main.c
)

target_compile_options(pstop_requirements_test PRIVATE -Wall -Wextra -Werror -fanalyzer)

target_include_directories(pstop_requirements_test PUBLIC
  include
  test/include
)

target_link_libraries(pstop_requirements_test PUBLIC
  pstop
  unity
)
