// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "protective_stop_machine/machine_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Multi-threaded: the backend runs its own thread; the executor serves ROS
  // callbacks (publish timer, service, params, diagnostics) without contending
  // with the machine loop.
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<protective_stop_machine::MachineBridgeNode>(
    rclcpp::NodeOptions());
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
