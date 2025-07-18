cmake_minimum_required(VERSION 3.8)
project(protective_stop_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

# Generate ROS2 message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ProtectiveStopStatus.msg"
  "msg/ProtectiveStop.msg"
  "msg/ProtectiveStopParams.msg"
  "msg/ProtectiveStopHeartbeat.msg"
  "msg/ProtectiveStopDebug.msg"
  "msg/ProtectiveStopDebugRemote.msg"
  "srv/BypassProtectiveStop.srv"
  "srv/ProtectiveStop.srv"
  DEPENDENCIES lifecycle_msgs builtin_interfaces unique_identifier_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
