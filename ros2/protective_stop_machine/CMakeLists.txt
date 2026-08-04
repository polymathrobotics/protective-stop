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

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# libcurl is a system dep (rosdep libcurl-dev), so ament_auto skips it.
find_package(CURL REQUIRED)

# Typed, validated parameters (SR-M-01 floors are the timing.* bounds).
generate_parameter_library(${PROJECT_NAME}_parameters
  src/${PROJECT_NAME}_params.yaml
)

# Vendored, unmodified pstop_c sources (same set host/machine_app_runner links).
set(PSTOP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../pstop_c)
file(GLOB PSTOP_SRCS CONFIGURE_DEPENDS ${PSTOP_ROOT}/pstop/src/pstop/*.c)
set(TRANSPORT_SRCS ${PSTOP_ROOT}/transport/src/transport/udp/udp_transport.c)

# Composable component library; the executable is code-generated below.
add_library(${PROJECT_NAME} SHARED
  src/machine_bridge_node.cpp
  src/software_backend.cpp
  src/hardware_backend.cpp
  src/announce.cpp
  ${PSTOP_SRCS}
  ${TRANSPORT_SRCS}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
# pstop_c headers are implementation detail, not public API.
target_include_directories(${PROJECT_NAME} PRIVATE
  ${PSTOP_ROOT}/pstop/include
  ${PSTOP_ROOT}/transport/include
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    diagnostic_updater::diagnostic_updater
    ${lifecycle_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${protective_stop_msgs_TARGETS}
    ${PROJECT_NAME}_parameters
  PRIVATE
    rclcpp_components::component
    CURL::libcurl
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "protective_stop_machine::MachineBridgeNode"
  EXECUTABLE machine_bridge_node
)

# ${PROJECT_NAME}_parameters is a header-only INTERFACE target (compiled into the
# component .so), so it needs no separate install.
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Tests link the component library instead of recompiling its sources.
  ament_add_gtest(test_json_lite test/test_json_lite.cpp)
  target_include_directories(test_json_lite PRIVATE include)

  # Operator-authorization policy (SAFETY): unlisted remote = stop-only (may STOP,
  # never re-arm); only a listed operator may re-arm. Header-only decision fn, no
  # ROS runtime / no pstop link.
  ament_add_gtest(test_operator_policy test/test_operator_policy.cpp)
  target_include_directories(test_operator_policy PRIVATE include)

  # Fleet check-in payload builder — header-inline, network-free. Verifies the
  # JSON shape matches the host runner's announce (name+port core) plus the
  # additive machine/status fields. No POST, no live fleet.
  ament_add_gtest(test_announce test/test_announce.cpp)
  target_include_directories(test_announce PRIVATE include)

  # Pure /state.json -> MachineSnapshot mapping (SR-M-03 / DU-9).
  ament_add_gtest(test_hardware_parse test/test_hardware_parse.cpp)
  target_link_libraries(test_hardware_parse ${PROJECT_NAME} CURL::libcurl)

  # Hardware backend HTTP/threading against a loopback stub + closed port (SR-M-03).
  ament_add_gtest(test_hardware_backend test/test_hardware_backend.cpp)
  target_include_directories(test_hardware_backend PRIVATE test)
  target_link_libraries(test_hardware_backend ${PROJECT_NAME} CURL::libcurl)

  # Lifecycle transitions + spun-executor runtime paths (SR-M-01 / SR-M-07).
  ament_add_gtest(test_lifecycle test/test_lifecycle.cpp)
  target_link_libraries(test_lifecycle ${PROJECT_NAME})

  # test_node_runtime drives the loopback HTTP stub (http_stub.hpp) directly.
  ament_add_gtest(test_node_runtime test/test_node_runtime.cpp)
  target_include_directories(test_node_runtime PRIVATE test)
  target_link_libraries(test_node_runtime ${PROJECT_NAME} CURL::libcurl)
endif()

ament_package()
