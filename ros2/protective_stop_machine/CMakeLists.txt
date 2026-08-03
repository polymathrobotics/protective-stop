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

  # Unit-test the pure /state.json -> MachineSnapshot mapping (SR-M-03 / DU-9).
  # Links hardware_backend.cpp (parse_state lives there) + libcurl for its HTTP
  # helpers; no ROS runtime is exercised by the test.
  ament_add_gtest(test_hardware_parse test/test_hardware_parse.cpp src/hardware_backend.cpp)
  target_include_directories(test_hardware_parse PRIVATE include)
  target_link_libraries(test_hardware_parse CURL::libcurl)

  # HTTP/threading paths of the hardware backend (http_get/http_post/write_cb,
  # poll loop, start/stop, configure) exercised against a loopback HTTP stub +
  # a closed port — no live device. SR-M-03.
  ament_add_gtest(test_hardware_backend test/test_hardware_backend.cpp src/hardware_backend.cpp)
  target_include_directories(test_hardware_backend PRIVATE include test)
  target_link_libraries(test_hardware_backend CURL::libcurl)

  # Timing-floor validator (SR-M-01) — header-only, no ROS runtime.
  ament_add_gtest(test_timing_floors test/test_timing_floors.cpp)
  target_include_directories(test_timing_floors PRIVATE include)

  # Lifecycle transitions (configure/activate/.../reject unsafe config). Links
  # the node + both backends + pstop_c, same as the executable but with its own
  # main(). SR-M-01 / SR-M-07.
  ament_add_gtest(test_lifecycle test/test_lifecycle.cpp
    src/machine_bridge_node.cpp src/software_backend.cpp src/hardware_backend.cpp
    ${PSTOP_SRCS} ${TRANSPORT_SRCS})
  target_include_directories(test_lifecycle PRIVATE
    include ${PSTOP_ROOT}/pstop/include ${PSTOP_ROOT}/transport/include)
  ament_target_dependencies(test_lifecycle
    rclcpp rclcpp_lifecycle lifecycle_msgs diagnostic_updater diagnostic_msgs protective_stop_msg)
  target_link_libraries(test_lifecycle CURL::libcurl)

  # Runtime node paths that need a spun executor: publish_tick, every diagnostics
  # summary branch (driven off a loopback /state.json stub), on_set_parameters,
  # the ~/configure_machine service, and the error/shutdown/start-failure
  # transitions. Same link set as the executable, own main(). SR-M-01 / SR-M-07.
  ament_add_gtest(test_node_runtime test/test_node_runtime.cpp
    src/machine_bridge_node.cpp src/software_backend.cpp src/hardware_backend.cpp
    ${PSTOP_SRCS} ${TRANSPORT_SRCS})
  target_include_directories(test_node_runtime PRIVATE
    include test ${PSTOP_ROOT}/pstop/include ${PSTOP_ROOT}/transport/include)
  ament_target_dependencies(test_node_runtime
    rclcpp rclcpp_lifecycle lifecycle_msgs diagnostic_updater diagnostic_msgs protective_stop_msg)
  target_link_libraries(test_node_runtime CURL::libcurl)
endif()

ament_package()
