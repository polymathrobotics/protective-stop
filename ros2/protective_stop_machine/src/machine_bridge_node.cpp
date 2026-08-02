// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// MachineBridgeNode — managed (lifecycle) node presenting one ROS 2 surface over
// either machine backend. ROS glue only; all machine logic lives behind
// IMachineBackend. See docs/MACHINE_ROS2_NODE_DESIGN.md.
#include "protective_stop_machine/machine_bridge_node.hpp"

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "protective_stop_machine/hardware_backend.hpp"
#include "protective_stop_machine/software_backend.hpp"

namespace protective_stop_machine
{

using ProtectiveStopStatus = protective_stop_msg::msg::ProtectiveStopStatus;
using MachineRelayStatus = protective_stop_msg::msg::MachineRelayStatus;
using BondedRemoteArray = protective_stop_msg::msg::BondedRemoteArray;
using BondedRemote = protective_stop_msg::msg::BondedRemote;
using ConfigureMachine = protective_stop_msg::srv::ConfigureMachine;

// Compile-time safety envelope. A runtime timing change may only stay INSIDE
// this — the validator rejects anything that would loosen it (design §4).
namespace floor
{
constexpr uint64_t kMinHeartbeatMs = 50;
constexpr uint64_t kMaxHeartbeatMs = 1000;      // window can't exceed 1 s
constexpr uint16_t kMinMaxMissed = 1;
constexpr uint16_t kMaxMaxMissed = 5;           // can't tolerate more than 5
constexpr uint64_t kMinStopFloorMs = 100;       // anti-blip arming delay floor
}  // namespace floor

MachineBridgeNode::MachineBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("machine_bridge", options)
{
  declare_all_parameters();
}

void MachineBridgeNode::declare_all_parameters()
{
  auto ro = []() {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.read_only = true;
      return d;
    };

  declare_parameter<std::string>("backend", "software", ro());
  declare_parameter<int>("machine_id", 0x01020304, ro());
  declare_parameter<std::string>("frame_id", "pstop_machine", ro());

  declare_parameter<std::string>("software.bind_addr", "0.0.0.0", ro());
  declare_parameter<int>("software.port", 8890, ro());

  declare_parameter<std::string>("hardware.device_url", "http://127.0.0.1", ro());
  declare_parameter<std::string>("hardware.admin_user", "admin", ro());

  // Only the timing.* params are dynamic (validated + applied at runtime).
  declare_parameter<int>("timing.heartbeat_ms", 400);
  declare_parameter<int>("timing.max_missed", 3);
  declare_parameter<int>("timing.min_stop_ms", 500);

  // Rates are read at configure/activate time; changing them needs a restart.
  declare_parameter<double>("rates.publish_rate_hz", 10.0, ro());
  declare_parameter<double>("rates.state_poll_hz", 5.0, ro());
  declare_parameter<double>("rates.diagnostics_rate_hz", 1.0, ro());
}

bool MachineBridgeNode::validate_timing(const MachineTiming & t, std::string & reason) const
{
  if (t.heartbeat_ms < floor::kMinHeartbeatMs || t.heartbeat_ms > floor::kMaxHeartbeatMs) {
    reason = "heartbeat_ms out of [" + std::to_string(floor::kMinHeartbeatMs) + "," +
      std::to_string(floor::kMaxHeartbeatMs) + "]";
    return false;
  }
  if (t.max_missed < floor::kMinMaxMissed || t.max_missed > floor::kMaxMaxMissed) {
    reason = "max_missed out of [" + std::to_string(floor::kMinMaxMissed) + "," +
      std::to_string(floor::kMaxMaxMissed) + "]";
    return false;
  }
  if (t.min_stop_ms < floor::kMinStopFloorMs) {
    reason = "min_stop_ms below safety floor " + std::to_string(floor::kMinStopFloorMs);
    return false;
  }
  reason.clear();
  return true;
}

bool MachineBridgeNode::build_backend(std::string & error)
{
  timing_.heartbeat_ms = static_cast<uint64_t>(get_parameter("timing.heartbeat_ms").as_int());
  timing_.max_missed = static_cast<uint16_t>(get_parameter("timing.max_missed").as_int());
  timing_.min_stop_ms = static_cast<uint64_t>(get_parameter("timing.min_stop_ms").as_int());
  if (!validate_timing(timing_, error)) {return false;}

  frame_id_ = get_parameter("frame_id").as_string();
  backend_kind_ = get_parameter("backend").as_string();

  if (backend_kind_ == "software") {
    SoftwareConfig sc;
    sc.bind_addr = get_parameter("software.bind_addr").as_string();
    sc.port = static_cast<int>(get_parameter("software.port").as_int());
    sc.machine_id = static_cast<uint32_t>(get_parameter("machine_id").as_int());
    sc.timing = timing_;
    backend_ = std::make_unique<SoftwareMachineBackend>(sc);
    return true;
  } else if (backend_kind_ == "hardware") {
    HardwareConfig hc;
    hc.device_url = get_parameter("hardware.device_url").as_string();
    hc.admin_user = get_parameter("hardware.admin_user").as_string();
    hc.poll_hz = get_parameter("rates.state_poll_hz").as_double();
    // Admin password from the environment — never a parameter file.
    const char * pw = std::getenv("PSTOP_MACHINE_ADMIN_PASS");
    hc.admin_pass = pw ? pw : "";
    backend_ = std::make_unique<HardwareMachineBackend>(hc);
    return true;
  }
  error = "unknown backend '" + backend_kind_ + "' (want software|hardware)";
  return false;
}

MachineBridgeNode::CallbackReturn
MachineBridgeNode::on_configure(const rclcpp_lifecycle::State &)
{
  std::string err;
  if (!build_backend(err)) {
    RCLCPP_ERROR(get_logger(), "configure failed: %s", err.c_str());
    return CallbackReturn::FAILURE;
  }

  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  const auto data = rclcpp::QoS(5).reliable();
  state_pub_ = create_publisher<ProtectiveStopStatus>("~/machine_state", latched);
  relay_pub_ = create_publisher<MachineRelayStatus>("~/relay_status", data);
  remotes_pub_ = create_publisher<BondedRemoteArray>("~/remotes", data);

  configure_srv_ = create_service<ConfigureMachine>(
    "~/configure_machine",
    std::bind(&MachineBridgeNode::handle_configure, this,
    std::placeholders::_1, std::placeholders::_2));

  param_cb_handle_ = add_on_set_parameters_callback(
    std::bind(&MachineBridgeNode::on_set_parameters, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "configured: backend=%s", backend_kind_.c_str());
  return CallbackReturn::SUCCESS;
}

MachineBridgeNode::CallbackReturn
MachineBridgeNode::on_activate(const rclcpp_lifecycle::State & s)
{
  // Start the backend FIRST so a failure needs no rollback (publishers/timer
  // not yet activated).
  if (!backend_ || !backend_->start()) {
    RCLCPP_ERROR(get_logger(), "backend %s failed to start", backend_kind_.c_str());
    return CallbackReturn::FAILURE;
  }

  LifecycleNode::on_activate(s);   // activate managed publishers
  state_pub_->on_activate();
  relay_pub_->on_activate();
  remotes_pub_->on_activate();

  const double hz = std::max(1.0, get_parameter("rates.publish_rate_hz").as_double());
  pub_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / hz),
    std::bind(&MachineBridgeNode::publish_tick, this));

  const double dhz = std::max(0.1, get_parameter("rates.diagnostics_rate_hz").as_double());
  diag_ = std::make_shared<diagnostic_updater::Updater>(this, 1.0 / dhz);
  diag_->setHardwareID(backend_kind_);
  diag_->add("machine", this, &MachineBridgeNode::diagnostics);

  RCLCPP_INFO(get_logger(), "activated: publishing at %.1f Hz", hz);
  return CallbackReturn::SUCCESS;
}

MachineBridgeNode::CallbackReturn
MachineBridgeNode::on_deactivate(const rclcpp_lifecycle::State & s)
{
  if (pub_timer_) {pub_timer_->cancel(); pub_timer_.reset();}
  diag_.reset();
  if (backend_) {backend_->stop();}   // -> machine safe (remotes fail-safe)
  state_pub_->on_deactivate();
  relay_pub_->on_deactivate();
  remotes_pub_->on_deactivate();
  LifecycleNode::on_deactivate(s);
  RCLCPP_INFO(get_logger(), "deactivated: backend stopped (safe state)");
  return CallbackReturn::SUCCESS;
}

MachineBridgeNode::CallbackReturn
MachineBridgeNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  param_cb_handle_.reset();
  configure_srv_.reset();
  state_pub_.reset();
  relay_pub_.reset();
  remotes_pub_.reset();
  backend_.reset();
  return CallbackReturn::SUCCESS;
}

MachineBridgeNode::CallbackReturn
MachineBridgeNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (pub_timer_) {pub_timer_->cancel();}
  if (backend_) {backend_->stop();}
  return CallbackReturn::SUCCESS;
}

MachineBridgeNode::CallbackReturn
MachineBridgeNode::on_error(const rclcpp_lifecycle::State &)
{
  // error -> unconfigured: the framework does NOT call on_cleanup afterward, so
  // release everything here (a leaked timer would keep firing publish_tick).
  if (pub_timer_) {pub_timer_->cancel(); pub_timer_.reset();}
  diag_.reset();
  if (backend_) {backend_->stop();}   // safe: machine stops -> remotes fail-safe
  // Emit one explicit UNSTABLE before tearing down the publishers (design §8).
  if (state_pub_ && state_pub_->is_activated()) {
    ProtectiveStopStatus st;
    st.status = static_cast<uint8_t>(MachineState::UNSTABLE);
    st.message = "error transition";
    state_pub_->publish(st);
    state_pub_->on_deactivate();
    relay_pub_->on_deactivate();
    remotes_pub_->on_deactivate();
  }
  param_cb_handle_.reset();
  configure_srv_.reset();
  state_pub_.reset();
  relay_pub_.reset();
  remotes_pub_.reset();
  backend_.reset();
  RCLCPP_ERROR(get_logger(), "error transition: safe teardown + UNSTABLE published");
  return CallbackReturn::SUCCESS;
}

void MachineBridgeNode::publish_tick()
{
  if (!backend_) {return;}
  last_snapshot_ = backend_->snapshot();
  const auto & s = last_snapshot_;
  const auto now = this->now();

  ProtectiveStopStatus st;
  st.status = static_cast<uint8_t>(s.state());
  st.message = s.status_reason;
  state_pub_->publish(st);

  MachineRelayStatus relay;
  relay.stamp = now;
  relay.applicable = s.relay.applicable;
  relay.run = s.relay.run;
  relay.relay_stop = s.relay.relay_stop;
  relay.relay_fault_a = s.relay.fault_a;
  relay.relay_fault_b = s.relay.fault_b;
  relay.mismatch = s.relay.mismatch;
  relay_pub_->publish(relay);

  BondedRemoteArray arr;
  arr.stamp = now;
  for (const auto & r : s.remotes) {
    BondedRemote b;
    b.device_id = r.device_id;
    b.bond_state = r.bond_state;
    b.in_use = r.in_use;
    b.stop_only = r.stop_only;
    b.reply_age_ms = r.reply_age_ms;
    b.loop_rtt_ms = r.loop_rtt_ms;
    b.disco_rtt_ms = r.disco_rtt_ms;
    b.rebonds = r.rebonds;
    arr.remotes.push_back(std::move(b));
  }
  remotes_pub_->publish(arr);
}

void MachineBridgeNode::diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const auto & s = last_snapshot_;
  using diagnostic_msgs::msg::DiagnosticStatus;
  if (!s.reachable) {
    stat.summary(DiagnosticStatus::ERROR, "backend unreachable — ROS blind "
      "(machine still enforces independently)");
  } else if (s.relay.fault_a || s.relay.fault_b) {
    stat.summary(DiagnosticStatus::ERROR, "relay feedback fault");
  } else if (s.relay.mismatch > 0) {
    stat.summary(DiagnosticStatus::WARN, "lockstep mismatch present");
  } else if (s.running) {
    stat.summary(DiagnosticStatus::OK, "running");
  } else {
    stat.summary(DiagnosticStatus::OK, s.status_reason);
  }
  stat.add("backend", backend_kind_);
  stat.add("running", s.running);
  stat.add("active_remotes", s.active_remotes);
  stat.add("relay_stop", s.relay.relay_stop);
  stat.add("mismatch", s.relay.mismatch);
}

rcl_interfaces::msg::SetParametersResult
MachineBridgeNode::on_set_parameters(const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  MachineTiming proposed = timing_;
  bool timing_changed = false;
  for (const auto & p : params) {
    if (p.get_name() == "timing.heartbeat_ms") {
      proposed.heartbeat_ms = static_cast<uint64_t>(p.as_int()); timing_changed = true;
    } else if (p.get_name() == "timing.max_missed") {
      proposed.max_missed = static_cast<uint16_t>(p.as_int()); timing_changed = true;
    } else if (p.get_name() == "timing.min_stop_ms") {
      proposed.min_stop_ms = static_cast<uint64_t>(p.as_int()); timing_changed = true;
    }
  }
  if (timing_changed) {
    std::string reason;
    if (!validate_timing(proposed, reason)) {
      res.successful = false;
      res.reason = "rejected (safety floor): " + reason;
      return res;
    }
    // Apply. The software backend's configure() only stashes timing (the
    // machine thread applies it), so this is non-blocking there. The hardware
    // backend does a bounded admin POST — acceptable for a rare operator set.
    if (backend_) {
      std::string err;
      if (!backend_->configure(proposed, err)) {
        res.successful = false;
        res.reason = "backend refused: " + err;
        return res;
      }
    }
    timing_ = proposed;
  }
  return res;
}

void MachineBridgeNode::handle_configure(
  const std::shared_ptr<ConfigureMachine::Request> req,
  std::shared_ptr<ConfigureMachine::Response> resp)
{
  MachineTiming proposed = timing_;
  if (req->heartbeat_ms > 0) {proposed.heartbeat_ms = static_cast<uint64_t>(req->heartbeat_ms);}
  if (req->max_missed > 0) {proposed.max_missed = static_cast<uint16_t>(req->max_missed);}
  if (req->min_stop_ms >= 0) {proposed.min_stop_ms = static_cast<uint64_t>(req->min_stop_ms);}

  std::string reason;
  if (!validate_timing(proposed, reason)) {
    resp->success = false;
    resp->message = "rejected (safety floor): " + reason;
  } else if (!backend_) {
    resp->success = false;
    resp->message = "no backend";
  } else if (!backend_->configure(proposed, reason)) {
    resp->success = false;
    resp->message = "backend refused: " + reason;
  } else {
    timing_ = proposed;
    resp->success = true;
    resp->message = "applied";
  }
  resp->heartbeat_ms = static_cast<int64_t>(timing_.heartbeat_ms);
  resp->max_missed = static_cast<int32_t>(timing_.max_missed);
  resp->min_stop_ms = static_cast<int64_t>(timing_.min_stop_ms);
}

}  // namespace protective_stop_machine
