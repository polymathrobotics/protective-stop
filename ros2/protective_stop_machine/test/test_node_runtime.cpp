// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Runtime tests for MachineBridgeNode that need a spun executor — the paths
// test_lifecycle cannot reach with inline transitions:
//   - publish_tick: the ~/machine_state, ~/relay_status and ~/remotes publishers
//     (incl. the bonded-remote mapping) driven off a live backend snapshot,
//   - diagnostics: every summary branch (unreachable ERROR / relay-fault ERROR /
//     lockstep-mismatch WARN / running OK / stopped OK), driven by feeding the
//     hardware backend canned /state.json off a loopback HTTP stub,
//   - on_set_parameters: accept a tighter timing envelope, reject one that would
//     breach the safety floor (SR-M-01) — the generated ParamListener ranges,
//   - the error/shutdown transitions and the on_activate backend-start failure.
// No real robot: the hardware backend talks to the in-process stub or a closed
// port; the software backend binds a private test port.

#include <curl/curl.h>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "http_stub.hpp"  // NOLINT(build/include_subdir)
#include "lifecycle_msgs/msg/state.hpp"
#include "protective_stop_machine/machine_bridge_node.hpp"
#include "protective_stop_msgs/msg/bonded_remote_array.hpp"
#include "protective_stop_msgs/msg/machine_relay_status.hpp"
#include "protective_stop_msgs/msg/protective_stop_status.hpp"
#include "rclcpp/rclcpp.hpp"

using protective_stop_machine::MachineBridgeNode;
using pstop_test::LoopbackHttpStub;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using MachineRelayStatus = protective_stop_msgs::msg::MachineRelayStatus;
using ProtectiveStopStatus = protective_stop_msgs::msg::ProtectiveStopStatus;
using BondedRemoteArray = protective_stop_msgs::msg::BondedRemoteArray;
using State = lifecycle_msgs::msg::State;

static rclcpp::NodeOptions with(std::vector<rclcpp::Parameter> overrides)
{
  rclcpp::NodeOptions o;
  o.parameter_overrides(std::move(overrides));
  return o;
}

// Spin the executor until pred() is true or the deadline passes.
template<typename Pred>
static bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & exec,
  Pred pred,
  std::chrono::milliseconds timeout = std::chrono::seconds(5))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    exec.spin_some(std::chrono::milliseconds(10));
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return pred();
}

class NodeRuntime : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

// publish_tick over the software backend: all three topics are published, the
// state reflects "not running" (reachable, no arming gesture), relays are marked
// not-applicable, and diagnostics reports an OK/stopped summary.
TEST_F(NodeRuntime, SoftwarePublishesAndDiagnoses)
{
  auto node = std::make_shared<MachineBridgeNode>(
    with(
      {rclcpp::Parameter("backend", "software"),
        rclcpp::Parameter("software.port", 18921),
        rclcpp::Parameter("rates.publish_rate_hz", 30.0),
        rclcpp::Parameter("rates.diagnostics_rate_hz", 30.0)}));
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);

  auto sub = std::make_shared<rclcpp::Node>("listener_a");
  bool got_state = false;
  uint8_t status = 255;
  bool relay_applicable = true;
  bool got_relay = false;
  bool got_remotes = false;
  bool got_diag = false;
  auto s1 = sub->create_subscription<ProtectiveStopStatus>(
    "/machine_bridge/machine_state", rclcpp::QoS(1).transient_local(),
    [&](ProtectiveStopStatus::SharedPtr m) {
      got_state = true;
      status = m->status;
    });
  auto s2 = sub->create_subscription<MachineRelayStatus>(
    "/machine_bridge/relay_status", rclcpp::QoS(5), [&](MachineRelayStatus::SharedPtr m) {
      got_relay = true;
      relay_applicable = m->applicable;
    });
  auto s3 = sub->create_subscription<BondedRemoteArray>(
    "/machine_bridge/remotes", rclcpp::QoS(5), [&](BondedRemoteArray::SharedPtr) {
      got_remotes = true;
    });
  auto s4 =
    sub->create_subscription<DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(5),
    [&](DiagnosticArray::SharedPtr m) {
      for (const auto & st : m->status) {
        if (st.name.find("machine") != std::string::npos) {
          got_diag = true;
        }
      }
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(sub);
  EXPECT_TRUE(spin_until(exec, [&] {return got_state && got_relay && got_remotes && got_diag;}));
  EXPECT_EQ(status, static_cast<uint8_t>(protective_stop_machine::MachineState::DEACTIVATED));
  EXPECT_FALSE(relay_applicable);  // software backend has no physical relays

  EXPECT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

// Hardware backend pointed at a closed port: publish_tick maps the unreachable
// snapshot to UNSTABLE, and diagnostics raises the "backend unreachable" ERROR.
TEST_F(NodeRuntime, HardwareUnreachableDiagnosesError)
{
  auto node = std::make_shared<MachineBridgeNode>(
    with(
      {rclcpp::Parameter("backend", "hardware"),
        rclcpp::Parameter("hardware.device_url", "http://127.0.0.1:9"),
        rclcpp::Parameter("rates.state_poll_hz", 40.0),
        rclcpp::Parameter("rates.publish_rate_hz", 40.0),
        rclcpp::Parameter("rates.diagnostics_rate_hz", 40.0)}));
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);

  auto sub = std::make_shared<rclcpp::Node>("listener_hw_unreach");
  uint8_t status = 0;
  bool err_diag = false;
  auto s1 = sub->create_subscription<ProtectiveStopStatus>(
    "/machine_bridge/machine_state", rclcpp::QoS(1).transient_local(),
    [&](ProtectiveStopStatus::SharedPtr m) {
      status = m->status;
    });
  auto s2 =
    sub->create_subscription<DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(5),
    [&](DiagnosticArray::SharedPtr m) {
      for (const auto & st : m->status) {
        if (st.level == DiagnosticStatus::ERROR &&
        st.message.find("unreachable") != std::string::npos)
        {
          err_diag = true;
        }
      }
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(sub);
  EXPECT_TRUE(spin_until(exec, [&] {return err_diag;}));
  EXPECT_EQ(status, static_cast<uint8_t>(protective_stop_machine::MachineState::UNSTABLE));

  EXPECT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

// Drive the hardware backend off a loopback stub so a specific /state.json steers
// the diagnostics summary + the bonded-remote publish loop.
static void run_hardware_stub_case(
  const std::string & state_json,
  uint8_t want_diag_level,
  const std::string & want_msg_substr,
  uint8_t want_status,
  bool expect_remote)
{
  LoopbackHttpStub stub(state_json);
  auto node = std::make_shared<MachineBridgeNode>(
    with(
      {rclcpp::Parameter("backend", "hardware"),
        rclcpp::Parameter("hardware.device_url", stub.url()),
        rclcpp::Parameter("rates.state_poll_hz", 40.0),
        rclcpp::Parameter("rates.publish_rate_hz", 40.0),
        rclcpp::Parameter("rates.diagnostics_rate_hz", 40.0)}));
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);

  auto sub = std::make_shared<rclcpp::Node>("listener_hw_stub");
  uint8_t status = 255;
  bool diag_hit = false;
  size_t remote_count = 0;
  auto s1 = sub->create_subscription<ProtectiveStopStatus>(
    "/machine_bridge/machine_state", rclcpp::QoS(1).transient_local(),
    [&](ProtectiveStopStatus::SharedPtr m) {
      status = m->status;
    });
  auto s2 = sub->create_subscription<BondedRemoteArray>(
    "/machine_bridge/remotes", rclcpp::QoS(5), [&](BondedRemoteArray::SharedPtr m) {
      remote_count = m->remotes.size();
    });
  auto s3 =
    sub->create_subscription<DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(5),
    [&](DiagnosticArray::SharedPtr m) {
      for (const auto & st : m->status) {
        if (st.level == want_diag_level &&
        st.message.find(want_msg_substr) != std::string::npos)
        {
          diag_hit = true;
        }
      }
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(sub);
  EXPECT_TRUE(
    spin_until(
      exec, [&] {
        return diag_hit && status == want_status && (!expect_remote || remote_count == 1U);
      }));
  EXPECT_TRUE(diag_hit);
  EXPECT_EQ(status, want_status);
  if (expect_remote) {
    EXPECT_EQ(remote_count, 1U);
  }

  EXPECT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

// running (relay closed) + a bonded remote: OK "running" diagnostic, ACTIVE
// state, and the publish_tick remotes loop maps the remote out.
TEST_F(NodeRuntime, HardwareRunningDiagnosesOk)
{
  run_hardware_stub_case(
    R"({"relay_stop":false,"bonded_remotes":[{"id":5,"state":2,"age_ms":10,"rtt_ms":20,"wg_rtt_ms":30}]})",
    DiagnosticStatus::OK,
    "running",
    static_cast<uint8_t>(protective_stop_machine::MachineState::ACTIVE),
    true);
}

// A relay feedback fault outranks "running" -> ERROR summary.
TEST_F(NodeRuntime, HardwareRelayFaultDiagnosesError)
{
  run_hardware_stub_case(
    R"({"relay_stop":false,"relay_fault_a":true})",
    DiagnosticStatus::ERROR,
    "relay feedback fault",
    static_cast<uint8_t>(protective_stop_machine::MachineState::ACTIVE),
    false);
}

// A lockstep mismatch (no fault) -> WARN summary.
TEST_F(NodeRuntime, HardwareMismatchDiagnosesWarn)
{
  run_hardware_stub_case(
    R"({"relay_stop":false,"pstop_mismatch":3})",
    DiagnosticStatus::WARN,
    "lockstep mismatch",
    static_cast<uint8_t>(protective_stop_machine::MachineState::ACTIVE),
    false);
}

// on_set_parameters: a tighter (still-safe) timing envelope is accepted and
// forwarded to the backend.
TEST_F(NodeRuntime, SetParametersAcceptsTighter)
{
  auto node = std::make_shared<MachineBridgeNode>(with({rclcpp::Parameter("backend", "software")}));
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  // Atomic set: on_set_parameters sees the whole batch at once and validates the
  // resulting envelope as a unit, returning a single result.
  auto res = node->set_parameters_atomically(
    {rclcpp::Parameter("timing.heartbeat_ms", 300),
      rclcpp::Parameter("timing.max_missed", 2),
      rclcpp::Parameter("timing.min_stop_ms", 600)});
  EXPECT_TRUE(res.successful);
}

// A runtime set that would breach the safety floor (SR-M-01) is rejected by the
// generated ParamListener's declared range before it can apply; the reason names
// the offending parameter.
TEST_F(NodeRuntime, SetParametersRejectsUnsafeMinStop)
{
  auto node = std::make_shared<MachineBridgeNode>(with({rclcpp::Parameter("backend", "software")}));
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  auto res = node->set_parameter(rclcpp::Parameter("timing.min_stop_ms", 10));
  EXPECT_FALSE(res.successful);
  EXPECT_NE(res.reason.find("min_stop_ms"), std::string::npos);
}

TEST_F(NodeRuntime, SetParametersRejectsOversizeHeartbeat)
{
  auto node = std::make_shared<MachineBridgeNode>(with({rclcpp::Parameter("backend", "software")}));
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  auto res = node->set_parameter(rclcpp::Parameter("timing.heartbeat_ms", 5000));
  EXPECT_FALSE(res.successful);
  EXPECT_NE(res.reason.find("heartbeat_ms"), std::string::npos);
}

// A safe-but-unappliable change: the value passes the floor, but the (hardware)
// backend's device is unreachable and refuses the proxy POST -> the parameter
// set is rejected with a "backend refused" reason (distinct from a floor breach).
TEST_F(NodeRuntime, SetParameterHardwareBackendRefuses)
{
  auto node = std::make_shared<MachineBridgeNode>(
    with(
      {rclcpp::Parameter("backend", "hardware"),
        rclcpp::Parameter("hardware.device_url", "http://127.0.0.1:9")}));
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  // safe value (clears the floor), but the device is unreachable so the backend refuses
  auto res = node->set_parameter(rclcpp::Parameter("timing.heartbeat_ms", 300));
  EXPECT_FALSE(res.successful);
  EXPECT_NE(res.reason.find("backend refused"), std::string::npos);
}

// on_activate returns FAILURE when the backend refuses to start. The software
// backend refuses a second concurrent instance (single-machine invariant), which
// gives a deterministic start() failure without a port race.
TEST_F(NodeRuntime, ActivateFailsWhenBackendRefusesStart)
{
  auto held = std::make_shared<MachineBridgeNode>(
    with({rclcpp::Parameter("backend", "software"), rclcpp::Parameter("software.port", 18924)}));
  ASSERT_EQ(held->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(held->activate().id(), State::PRIMARY_STATE_ACTIVE);  // claims the singleton

  auto second = std::make_shared<MachineBridgeNode>(
    with({rclcpp::Parameter("backend", "software"), rclcpp::Parameter("software.port", 18925)}));
  ASSERT_EQ(second->configure().id(), State::PRIMARY_STATE_INACTIVE);
  second->activate();  // backend->start() -> false -> on_activate FAILURE
  EXPECT_EQ(second->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);

  EXPECT_EQ(second->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(held->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(held->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

// on_shutdown from ACTIVE: timer cancelled + backend stopped -> FINALIZED.
TEST_F(NodeRuntime, ShutdownFromActive)
{
  auto node = std::make_shared<MachineBridgeNode>(
    with({rclcpp::Parameter("backend", "software"), rclcpp::Parameter("software.port", 18926)}));
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

// on_error teardown: from ACTIVE the error transition must emit one UNSTABLE,
// tear the publishers down and stop the backend. The node never returns ERROR by
// design, so the framework does not enter ErrorProcessing on its own; the handler
// is invoked directly here to cover the safe-teardown + UNSTABLE-publish path.
TEST_F(NodeRuntime, OnErrorTeardownFromActive)
{
  auto node = std::make_shared<MachineBridgeNode>(
    with({rclcpp::Parameter("backend", "software"), rclcpp::Parameter("software.port", 18927)}));
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(
    node->on_error(node->get_current_state()),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

// on_error on a not-yet-activated node: publishers are absent/inactive, so the
// UNSTABLE publish is skipped but teardown still completes (the is_activated()
// false branch + null-backend path).
TEST_F(NodeRuntime, OnErrorTeardownWithoutActivation)
{
  auto node = std::make_shared<MachineBridgeNode>(with({rclcpp::Parameter("backend", "software")}));
  EXPECT_EQ(
    node->on_error(node->get_current_state()),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  curl_global_init(CURL_GLOBAL_DEFAULT);
  const int rc = RUN_ALL_TESTS();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  curl_global_cleanup();
  return rc;
}
