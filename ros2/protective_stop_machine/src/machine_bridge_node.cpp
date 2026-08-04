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
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "protective_stop_machine/hardware_backend.hpp"
#include "protective_stop_machine/software_backend.hpp"

namespace protective_stop_machine
{

using ProtectiveStopStatus = protective_stop_msgs::msg::ProtectiveStopStatus;
using MachineRelayStatus = protective_stop_msgs::msg::MachineRelayStatus;
using BondedRemoteArray = protective_stop_msgs::msg::BondedRemoteArray;
using BondedRemote = protective_stop_msgs::msg::BondedRemote;

// The timing.* safety floors (SR-M-01) are enforced declaratively by the
// generated ParamListener — the bounds<>/gt_eq<> ranges in
// protective_stop_machine_params.yaml. An out-of-floor override is rejected at
// declare time (construction throws) and a loosening set is rejected before it
// applies, so no node code re-checks the floor.

MachineBridgeNode::MachineBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("machine_bridge", options),
  param_listener_(get_node_parameters_interface()),
  params_(param_listener_.get_params())
{
  // Optional self-managed bring-up
  if (params_.autostart &&
    configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    activate();
  }
}

bool MachineBridgeNode::build_backend(std::string & error)
{
  params_ = param_listener_.get_params();
  timing_.heartbeat_ms = static_cast<uint64_t>(params_.timing.heartbeat_ms);
  timing_.max_missed = static_cast<uint16_t>(params_.timing.max_missed);
  timing_.min_stop_ms = static_cast<uint64_t>(params_.timing.min_stop_ms);

  frame_id_ = params_.frame_id;
  backend_kind_ = params_.backend;

  if (backend_kind_ == "software") {
    SoftwareConfig sc;
    sc.bind_addr = params_.software.bind_addr;
    sc.port = static_cast<int>(params_.software.port);
    sc.machine_id = static_cast<uint32_t>(params_.machine_id);
    sc.timing = timing_;
    backend_ = std::make_unique<SoftwareMachineBackend>(sc);
    return true;
  } else if (backend_kind_ == "hardware") {
    HardwareConfig hc;
    hc.device_url = params_.hardware.device_url;
    hc.admin_user = params_.hardware.admin_user;
    hc.poll_hz = params_.rates.state_poll_hz;
    // Admin password from the environment — never a parameter file.
    const char * pw = std::getenv("PSTOP_MACHINE_ADMIN_PASS");
    hc.admin_pass = pw ? pw : "";
    backend_ = std::make_unique<HardwareMachineBackend>(hc);
    return true;
  }
  error = "unknown backend '" + backend_kind_ + "' (want software|hardware)";
  return false;
}

MachineBridgeNode::CallbackReturn MachineBridgeNode::on_configure(const rclcpp_lifecycle::State &)
{
  std::string err;
  if (!build_backend(err)) {
    RCLCPP_ERROR(get_logger(), "configure failed to build backend: %s", err.c_str());
    return CallbackReturn::FAILURE;
  }

  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  const auto data = rclcpp::QoS(5).reliable();
  state_pub_ = create_publisher<ProtectiveStopStatus>("~/machine_state", latched);
  relay_pub_ = create_publisher<MachineRelayStatus>("~/relay_status", data);
  remotes_pub_ = create_publisher<BondedRemoteArray>("~/remotes", data);

  param_cb_handle_ =
    add_on_set_parameters_callback(
    std::bind(
      &MachineBridgeNode::on_set_parameters, this,
      std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "configured: backend=%s", backend_kind_.c_str());
  return CallbackReturn::SUCCESS;
}

MachineBridgeNode::CallbackReturn MachineBridgeNode::on_activate(const rclcpp_lifecycle::State & state)
{
  // Start the backend FIRST so a failure needs no rollback (publishers/timer
  // not yet activated).
  if (!backend_ || !backend_->start()) {
    RCLCPP_ERROR(get_logger(), "backend %s failed to start", backend_kind_.c_str());
    return CallbackReturn::FAILURE;
  }

  LifecycleNode::on_activate(state);  // activate managed publishers
  state_pub_->on_activate();
  relay_pub_->on_activate();
  remotes_pub_->on_activate();

  const double hz = params_.rates.publish_rate_hz;  // floored at 1.0 by param validation
  pub_timer_ =
    create_wall_timer(
    std::chrono::duration<double>(1.0 / hz),
    std::bind(&MachineBridgeNode::publish_tick, this));

  const double dhz = params_.rates.diagnostics_rate_hz;  // floored at 0.1 by param validation
  diag_ = std::make_shared<diagnostic_updater::Updater>(this, 1.0 / dhz);
  diag_->setHardwareID(backend_kind_);
  diag_->add("machine", this, &MachineBridgeNode::diagnostics);

  RCLCPP_INFO(get_logger(), "activated: publishing at %.1f Hz", hz);
  return CallbackReturn::SUCCESS;
}

MachineBridgeNode::CallbackReturn MachineBridgeNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (pub_timer_) {
    pub_timer_->cancel();
    pub_timer_.reset();
  }
  diag_.reset();
  if (backend_) {
    backend_->stop();
  }  // -> machine safe (remotes fail-safe)
  state_pub_->on_deactivate();
  relay_pub_->on_deactivate();
  remotes_pub_->on_deactivate();
  LifecycleNode::on_deactivate(s);
  RCLCPP_INFO(get_logger(), "deactivated: backend stopped (safe state)");
  return CallbackReturn::SUCCESS;
}

MachineBridgeNode::CallbackReturn MachineBridgeNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  param_cb_handle_.reset();
  state_pub_.reset();
  relay_pub_.reset();
  remotes_pub_.reset();
  backend_.reset();
  return CallbackReturn::SUCCESS;
}

MachineBridgeNode::CallbackReturn MachineBridgeNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (pub_timer_) {
    pub_timer_->cancel();
  }
  if (backend_) {
    backend_->stop();
  }
  return CallbackReturn::SUCCESS;
}

MachineBridgeNode::CallbackReturn MachineBridgeNode::on_error(const rclcpp_lifecycle::State &)
{
  // error -> unconfigured: the framework does NOT call on_cleanup afterward, so
  // release everything here (a leaked timer would keep firing publish_tick).
  if (pub_timer_) {
    pub_timer_->cancel();
    pub_timer_.reset();
  }
  diag_.reset();
  if (backend_) {
    backend_->stop();
  }  // safe: machine stops -> remotes fail-safe
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
  state_pub_.reset();
  relay_pub_.reset();
  remotes_pub_.reset();
  backend_.reset();
  RCLCPP_ERROR(get_logger(), "error transition: safe teardown + UNSTABLE published");
  return CallbackReturn::SUCCESS;
}

void MachineBridgeNode::publish_tick()
{
  if (!backend_) {
    return;
  }
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
    stat.summary(
      DiagnosticStatus::ERROR,
      "backend unreachable — ROS blind "
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

rcl_interfaces::msg::SetParametersResult MachineBridgeNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  MachineTiming proposed = timing_;
  bool timing_changed = false;
  for (const auto & p : params) {
    if (p.get_name() == "timing.heartbeat_ms") {
      proposed.heartbeat_ms = static_cast<uint64_t>(p.as_int());
      timing_changed = true;
    } else if (p.get_name() == "timing.max_missed") {
      proposed.max_missed = static_cast<uint16_t>(p.as_int());
      timing_changed = true;
    } else if (p.get_name() == "timing.min_stop_ms") {
      proposed.min_stop_ms = static_cast<uint64_t>(p.as_int());
      timing_changed = true;
    }
  }
  if (timing_changed) {
    // The floor is already enforced by the generated ParamListener's ranges,
    // which run before this callback — anything reaching here is in-envelope.
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

}  // namespace protective_stop_machine

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(protective_stop_machine::MachineBridgeNode)
