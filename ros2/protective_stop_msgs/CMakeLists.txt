# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.8)
project(protective_stop_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # remote-side (restored)
  "msg/ProtectiveStopStatus.msg"
  "msg/ProtectiveStop.msg"
  "msg/ProtectiveStopParams.msg"
  "msg/ProtectiveStopHeartbeat.msg"
  "msg/ProtectiveStopDebug.msg"
  "msg/ProtectiveStopDebugRemote.msg"
  # machine-side (new)
  "msg/MachineRelayStatus.msg"
  "msg/BondedRemote.msg"
  "msg/BondedRemoteArray.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
