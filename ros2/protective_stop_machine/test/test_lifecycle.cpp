// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Lifecycle tests for MachineBridgeNode — drives the managed transitions
// (configure/activate/deactivate/cleanup) synchronously and asserts:
//  - a valid software config configures + runs the full lifecycle,
//  - an unsafe timing config is REJECTED at configure (build_backend ->
//    validate_timing -> FAILURE, node stays UNCONFIGURED) — SR-M-01 / SR-M-07,
//  - an unknown backend is rejected.
// No spinning is needed: the LifecycleNode convenience methods invoke the
// on_* callbacks inline and return the resulting State.

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "protective_stop_machine/machine_bridge_node.hpp"

using protective_stop_machine::MachineBridgeNode;
using State = lifecycle_msgs::msg::State;

static rclcpp::NodeOptions with(std::vector<rclcpp::Parameter> overrides)
{
  rclcpp::NodeOptions o;
  o.parameter_overrides(std::move(overrides));
  return o;
}

class Lifecycle : public ::testing::Test
{
protected:
  void SetUp() override {if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}}
};

TEST_F(Lifecycle, ConfiguresValidSoftware)
{
  auto n = std::make_shared<MachineBridgeNode>(with({rclcpp::Parameter("backend", "software")}));
  EXPECT_EQ(n->configure().id(), State::PRIMARY_STATE_INACTIVE);
}

// min_stop_ms=0 defeats the arming gesture (SF-3) — configure must fail and the
// node must stay unconfigured (no backend brought up on an unsafe envelope).
TEST_F(Lifecycle, RejectsMinStopZero)
{
  auto n = std::make_shared<MachineBridgeNode>(with({
      rclcpp::Parameter("backend", "software"),
      rclcpp::Parameter("timing.min_stop_ms", 0)}));
  n->configure();
  EXPECT_EQ(n->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

// Heartbeat window > 1 s defeats stop latency (SF-1).
TEST_F(Lifecycle, RejectsOversizeHeartbeat)
{
  auto n = std::make_shared<MachineBridgeNode>(with({
      rclcpp::Parameter("backend", "software"),
      rclcpp::Parameter("timing.heartbeat_ms", 5000)}));
  n->configure();
  EXPECT_EQ(n->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(Lifecycle, RejectsUnknownBackend)
{
  auto n = std::make_shared<MachineBridgeNode>(with({rclcpp::Parameter("backend", "bogus")}));
  n->configure();
  EXPECT_EQ(n->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

// Full happy path on the software backend (binds a private test port).
TEST_F(Lifecycle, FullSoftwareLifecycle)
{
  auto n = std::make_shared<MachineBridgeNode>(with({
      rclcpp::Parameter("backend", "software"),
      rclcpp::Parameter("software.port", 18899)}));
  EXPECT_EQ(n->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(n->activate().id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(n->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(n->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return rc;
}
