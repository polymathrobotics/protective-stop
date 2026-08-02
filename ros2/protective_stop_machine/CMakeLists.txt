# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.8)
project(protective_stop_machine)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
# The vendored pstop_c sources use C11 (atomics, designated inits) — pin it so
# we don't depend on the toolchain default (matches host/machine_app_runner).
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
  set(CMAKE_C_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(protective_stop_msg REQUIRED)
find_package(CURL REQUIRED)

# Vendored, unmodified pstop_c sources (same set host/machine_app_runner links).
set(PSTOP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../pstop_c)
file(GLOB PSTOP_SRCS CONFIGURE_DEPENDS ${PSTOP_ROOT}/pstop/src/pstop/*.c)
set(TRANSPORT_SRCS ${PSTOP_ROOT}/transport/src/transport/udp/udp_transport.c)

add_executable(machine_bridge_node
  src/main.cpp
  src/machine_bridge_node.cpp
  src/software_backend.cpp
  src/hardware_backend.cpp
  ${PSTOP_SRCS}
  ${TRANSPORT_SRCS}
)
target_include_directories(machine_bridge_node PRIVATE
  include
  ${PSTOP_ROOT}/pstop/include
  ${PSTOP_ROOT}/transport/include
)
ament_target_dependencies(machine_bridge_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  diagnostic_updater
  diagnostic_msgs
  protective_stop_msg
)
target_link_libraries(machine_bridge_node CURL::libcurl)

install(TARGETS machine_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # vendored C + house C++ style: skip the picky copyright/cpplint hooks
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_json_lite test/test_json_lite.cpp)
  target_include_directories(test_json_lite PRIVATE include)
endif()

ament_package()
