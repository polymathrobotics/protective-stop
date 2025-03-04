cmake_minimum_required(VERSION 3.5)
project(protective_stop_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(protective_stop_msg REQUIRED)

# Install Python package (Ensure package directory exists)
ament_python_install_package(protective_stop_node
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY
  # protective_stop_node
  # DESTINATION lib/${PROJECT_NAME}
# )

# Install Python executables
# install(PROGRAMS
  # protective_stop_node/protective_stop_node.py
  # DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

find_package(launch_testing_ament_cmake REQUIRED)

if(BUILD_TESTING)
  add_launch_test(
    test/launch.test.py
    TIMEOUT "60"
  )
endif()

ament_package()
