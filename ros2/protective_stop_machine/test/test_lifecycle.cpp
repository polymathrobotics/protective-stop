// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Lifecycle tests for MachineBridgeNode — drives the managed transitions
// (configure/activate/deactivate/cleanup) synchronously and asserts:
//  - a valid software config configures + runs the full lifecycle,
//  - an unsafe timing override is REJECTED at construction — the SR-M-01 floors
//    are declared parameter ranges, so the generated ParamListener throws before
//    a node on an unsafe envelope ever exists (SR-M-01 / SR-M-07),
//  - an unknown backend is rejected.
// No spinning is needed: the LifecycleNode convenience methods invoke the
// on_* callbacks inline and return the resulting State.

#include <gtest/gtest.h>

#include <memory>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "protective_stop_machine/machine_bridge_node.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

using protective_stop_machine::MachineBridgeNode;
using State = lifecycle_msgs::msg::State;

static rclcpp::NodeOptions with(std::vector<rclcpp::Parameter> overrides)
{
  // These tests drive the lifecycle by hand, so suppress the constructor's
  // self-activation (autostart defaults to true for `ros2 run`). Placed first
  // so any explicit per-test override still wins.
  std::vector<rclcpp::Parameter> params{rclcpp::Parameter("autostart", false)};
  params.insert(params.end(), overrides.begin(), overrides.end());
  rclcpp::NodeOptions options;
  options.parameter_overrides(std::move(params));
  return options;
}

class Lifecycle : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(Lifecycle, ConfiguresValidSoftware)
{
  auto node = std::make_shared<MachineBridgeNode>(with({rclcpp::Parameter("backend", "software")}));
  EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
}

// min_stop_ms=0 defeats the arming gesture (SF-3). The SR-M-01 floors are
// declared as parameter ranges, so an unsafe override is rejected the moment
// the node declares its parameters — construction throws, so no node on an
// unsafe envelope ever exists to configure.
TEST_F(Lifecycle, RejectsMinStopZero)
{
  EXPECT_THROW(
    std::make_shared<MachineBridgeNode>(
      with({rclcpp::Parameter("backend", "software"), rclcpp::Parameter("timing.min_stop_ms", 0)})),
    rclcpp::exceptions::InvalidParameterValueException);
}

// Heartbeat window > 1 s defeats stop latency (SF-1) — same construction-time
// rejection via the declared parameter range.
TEST_F(Lifecycle, RejectsOversizeHeartbeat)
{
  EXPECT_THROW(
    std::make_shared<MachineBridgeNode>(
      with({rclcpp::Parameter("backend", "software"), rclcpp::Parameter("timing.heartbeat_ms", 5000)})),
    rclcpp::exceptions::InvalidParameterValueException);
}

TEST_F(Lifecycle, RejectsUnknownBackend)
{
  auto node = std::make_shared<MachineBridgeNode>(with({rclcpp::Parameter("backend", "bogus")}));
  node->configure();
  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

// Full happy path on the software backend (binds a private test port).
TEST_F(Lifecycle, FullSoftwareLifecycle)
{
  auto node = std::make_shared<MachineBridgeNode>(
    with({rclcpp::Parameter("backend", "software"), rclcpp::Parameter("software.port", 18899)}));
  EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  const int test_result = RUN_ALL_TESTS();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return test_result;
}
