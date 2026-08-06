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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ProtectiveStopStatus.msg"
  "msg/ProtectiveStopHeartbeat.msg"
  "msg/MachineRelayStatus.msg"
  "msg/BondedRemote.msg"
  "msg/BondedRemoteArray.msg"
  # Slated for removal: the block below has no consumer outside archive/
  # (protective_stop_node and the Foxglove remote example). Delete it when those
  # packages are phased out; it is a closed set, so nothing else drops with it.
  "msg/ProtectiveStop.msg"
  "msg/ProtectiveStopParams.msg"
  "msg/ProtectiveStopDebug.msg"
  "msg/ProtectiveStopDebugRemote.msg"
  "srv/ProtectiveStop.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()
