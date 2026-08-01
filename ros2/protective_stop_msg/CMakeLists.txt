# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.8)
project(protective_stop_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # remote-side (restored)
  "msg/ProtectiveStopStatus.msg"
  "msg/ProtectiveStop.msg"
  "msg/ProtectiveStopParams.msg"
  "msg/ProtectiveStopHeartbeat.msg"
  "msg/ProtectiveStopDebug.msg"
  "msg/ProtectiveStopDebugRemote.msg"
  "srv/ProtectiveStop.srv"
  # machine-side (new)
  "msg/MachineRelayStatus.msg"
  "msg/BondedRemote.msg"
  "msg/BondedRemoteArray.msg"
  "srv/ConfigureMachine.srv"
  DEPENDENCIES builtin_interfaces lifecycle_msgs unique_identifier_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
