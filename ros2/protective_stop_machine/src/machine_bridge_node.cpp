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

using ProtectiveStopStatus = protective_stop_msg::msg::ProtectiveStopStatus;
using ProtectiveStopHeartbeat = protective_stop_msg::msg::ProtectiveStopHeartbeat;
using MachineRelayStatus = protective_stop_msg::msg::MachineRelayStatus;
using BondedRemoteArray = protective_stop_msg::msg::BondedRemoteArray;
using BondedRemote = protective_stop_msg::msg::BondedRemote;

// The timing.* safety floors (SR-M-01) are enforced declaratively by the
// generated ParamListener — the bounds<>/gt_eq<> ranges in
// protective_stop_machine_params.yaml. An out-of-floor override is rejected at
// declare time (construction throws) and a loosening set is rejected before it
// applies, so no node code re-checks the floor.

MachineBridgeNode::MachineBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("machine_bridge", options)
  , param_listener_(get_node_parameters_interface())
  , params_(param_listener_.get_params())
{
  // Optional self-managed bring-up
  if (params_.autostart && configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
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
    // Operator authorization (SAFETY): unlisted remotes stay STOP-ONLY, only a
    // listed 32-bit pstop id may re-arm. Empty list (default) => all stop-only.
    sc.default_stop_only = params_.software.default_stop_only;
    sc.operators.clear();
    sc.operators.reserve(params_.software.operators.size());
    for (int64_t operator_id : params_.software.operators) {
      sc.operators.push_back(static_cast<uint32_t>(operator_id));
    }
    backend_ = std::make_unique<SoftwareMachineBackend>(sc);

    // Resolve the optional fleet check-in (software machine only — the ESP32
    // machn already checks in on its own). Env overrides the params so URL/key
    // stay out of committed config, mirroring the host runner.
    announce_cfg_.url = params_.announce.url;
    announce_cfg_.key_file = params_.announce.key_file;
    announce_cfg_.name = params_.announce.name;
    announce_cfg_.interval_s = static_cast<int>(params_.announce.interval_s);
    if (const char * env_url = std::getenv("PSTOP_ANNOUNCE_URL"); env_url && env_url[0] != '\0') {
      announce_cfg_.url = env_url;
    }
    if (const char * env_key = std::getenv("PSTOP_ANNOUNCE_KEY_FILE");
      env_key && env_key[0] != '\0')
    {
      announce_cfg_.key_file = env_key;
    }
    announce_port_ = sc.port;
    machine_id_ = sc.machine_id;

    // Resolve the optional fleet DEVICE check-in (software machine only — the
    // ESP32 machn checks itself in). This registers the software machine with
    // pstop-fleet as a device_type="machine" device, mirroring the chip's
    // fleet_ota_checkin(); it is additive to the lighter announce above. The
    // [60, 300] cadence is enforced declaratively by the params bounds<>. Env
    // overrides the params so the proprietary URL/key stay out of committed config.
    fleet_cfg_.base_url = params_.fleet.checkin_url;
    fleet_cfg_.key_file = params_.fleet.api_key_file;
    fleet_cfg_.interval_s = static_cast<int>(params_.fleet.check_interval_s);
    if (const char * env_url = std::getenv("PSTOP_FLEET_CHECKIN_URL");
      env_url && env_url[0] != '\0')
    {
      fleet_cfg_.base_url = env_url;
    }
    if (const char * env_key = std::getenv("PSTOP_FLEET_API_KEY_FILE");
      env_key && env_key[0] != '\0')
    {
      fleet_cfg_.key_file = env_key;
    }
    // app_version is the ament package version (single source: package.xml,
    // injected by CMake). idf_version has no IDF analogue on a host, so it tags
    // the runtime distro instead, e.g. "ros2-jazzy".
#ifdef PSTOP_MACHINE_APP_VERSION
    fleet_cfg_.app_version = PSTOP_MACHINE_APP_VERSION;
#endif
    const char * distro = std::getenv("ROS_DISTRO");
    fleet_cfg_.idf_version = std::string("ros2-") + (distro &&
      distro[0] != '\0' ? distro : "unknown");
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
  heartbeat_pub_ =
    create_publisher<ProtectiveStopHeartbeat>("/pstop_hb", rclcpp::QoS(1).reliable());

  param_cb_handle_ =
    add_on_set_parameters_callback(std::bind(&MachineBridgeNode::on_set_parameters, this,
      std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "configured: backend=%s", backend_kind_.c_str());
  return CallbackReturn::SUCCESS;
}

MachineBridgeNode::CallbackReturn MachineBridgeNode::on_activate(
  const rclcpp_lifecycle::State & state)
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
  heartbeat_pub_->on_activate();

  const double hz = params_.rates.publish_rate_hz;  // floored at 1.0 by param validation
  pub_timer_ =
    create_wall_timer(std::chrono::duration<double>(1.0 / hz),
      std::bind(&MachineBridgeNode::publish_tick, this));

  const double dhz = params_.rates.diagnostics_rate_hz;  // floored at 0.1 by param validation
  diag_ = std::make_shared<diagnostic_updater::Updater>(this, 1.0 / dhz);
  diag_->setHardwareID(backend_kind_);
  diag_->add("machine", this, &MachineBridgeNode::diagnostics);

  // Optional fleet check-in — starts only while ACTIVE, software backend only,
  // and only when a URL is configured. Off the safety path: a failure to start
  // (or a dead console) only logs and never fails activation.
  if (backend_kind_ == "software" && !announce_cfg_.url.empty()) {
    announcer_ = std::make_unique<MachineAnnouncer>(announce_cfg_, announce_port_, machine_id_,
        [this]() {
          return backend_ ? backend_->snapshot() : MachineSnapshot{};
    });
    if (announcer_->start()) {
      RCLCPP_INFO(get_logger(), "announce: every %ds to %s", announce_cfg_.interval_s,
          announce_cfg_.url.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "announce: configured but did not start (see stderr)");
    }
  }

  // Optional fleet DEVICE check-in — same lifecycle/guards as the announcer:
  // ACTIVE only, software backend only, only when a base URL is configured. Off
  // the safety path: a failure to start (or a dead fleet) only logs.
  if (backend_kind_ == "software" && !fleet_cfg_.base_url.empty()) {
    fleet_checkin_ = std::make_unique<FleetCheckin>(fleet_cfg_, machine_id_, [this]() {
          return backend_ ? backend_->snapshot() : MachineSnapshot{};
    });
    if (fleet_checkin_->start()) {
      RCLCPP_INFO(
        get_logger(), "fleet-checkin: every %ds to %s/api/v1/checkin", fleet_cfg_.interval_s,
        fleet_cfg_.base_url.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "fleet-checkin: configured but did not start (see stderr)");
    }
  }

  RCLCPP_INFO(get_logger(), "activated: publishing at %.1f Hz", hz);
  return CallbackReturn::SUCCESS;
}

MachineBridgeNode::CallbackReturn MachineBridgeNode::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  if (pub_timer_) {
    pub_timer_->cancel();
    pub_timer_.reset();
  }
  diag_.reset();
  // Stop the check-in BEFORE the backend so its snapshot getter never races a
  // backend teardown.
  if (announcer_) {
    announcer_->stop();
    announcer_.reset();
  }
  if (fleet_checkin_) {
    fleet_checkin_->stop();
    fleet_checkin_.reset();
  }
  if (backend_) {
    backend_->stop();
  }  // -> machine safe (remotes fail-safe)
  publish_heartbeat(true, this->now());
  state_pub_->on_deactivate();
  relay_pub_->on_deactivate();
  remotes_pub_->on_deactivate();
  heartbeat_pub_->on_deactivate();
  LifecycleNode::on_deactivate(state);
  RCLCPP_INFO(get_logger(), "deactivated: backend stopped (safe state)");
  return CallbackReturn::SUCCESS;
}

MachineBridgeNode::CallbackReturn MachineBridgeNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (announcer_) {
    announcer_->stop();
    announcer_.reset();
  }
  if (fleet_checkin_) {
    fleet_checkin_->stop();
    fleet_checkin_.reset();
  }
  param_cb_handle_.reset();
  state_pub_.reset();
  relay_pub_.reset();
  remotes_pub_.reset();
  heartbeat_pub_.reset();
  backend_.reset();
  return CallbackReturn::SUCCESS;
}

MachineBridgeNode::CallbackReturn MachineBridgeNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (pub_timer_) {
    pub_timer_->cancel();
  }
  if (announcer_) {
    announcer_->stop();
    announcer_.reset();
  }
  if (fleet_checkin_) {
    fleet_checkin_->stop();
    fleet_checkin_.reset();
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
  if (announcer_) {
    announcer_->stop();
    announcer_.reset();
  }
  if (fleet_checkin_) {
    fleet_checkin_->stop();
    fleet_checkin_.reset();
  }
  if (backend_) {
    backend_->stop();
  }  // safe: machine stops -> remotes fail-safe
  // Emit one explicit UNSTABLE before tearing down the publishers (design §8).
  publish_heartbeat(true, this->now());
  if (state_pub_ && state_pub_->is_activated()) {
    ProtectiveStopStatus st;
    st.status = static_cast<uint8_t>(MachineState::UNSTABLE);
    st.message = "error transition";
    state_pub_->publish(st);
    state_pub_->on_deactivate();
    relay_pub_->on_deactivate();
    remotes_pub_->on_deactivate();
    heartbeat_pub_->on_deactivate();
  }
  param_cb_handle_.reset();
  state_pub_.reset();
  relay_pub_.reset();
  remotes_pub_.reset();
  heartbeat_pub_.reset();
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
  const auto & snapshot = last_snapshot_;
  const auto stamp = this->now();

  ProtectiveStopStatus status;
  status.status = static_cast<uint8_t>(snapshot.state());
  status.message = snapshot.status_reason;
  state_pub_->publish(status);

  publish_heartbeat(!snapshot.running, stamp);

  MachineRelayStatus relay_status;
  relay_status.stamp = stamp;
  relay_status.applicable = snapshot.relay.applicable;
  relay_status.run = snapshot.relay.run;
  relay_status.relay_stop = snapshot.relay.relay_stop;
  relay_status.relay_fault_a = snapshot.relay.fault_a;
  relay_status.relay_fault_b = snapshot.relay.fault_b;
  relay_status.mismatch = snapshot.relay.mismatch;
  relay_pub_->publish(relay_status);

  BondedRemoteArray remote_array;
  remote_array.stamp = stamp;
  for (const auto & remote : snapshot.remotes) {
    BondedRemote bonded_remote;
    bonded_remote.device_id = remote.device_id;
    bonded_remote.bond_state = remote.bond_state;
    bonded_remote.in_use = remote.in_use;
    bonded_remote.stop_only = remote.stop_only;
    bonded_remote.reply_age_ms = remote.reply_age_ms;
    bonded_remote.loop_rtt_ms = remote.loop_rtt_ms;
    bonded_remote.disco_rtt_ms = remote.disco_rtt_ms;
    bonded_remote.rebonds = remote.rebonds;
    remote_array.remotes.push_back(std::move(bonded_remote));
  }
  remotes_pub_->publish(remote_array);
}

void MachineBridgeNode::publish_heartbeat(bool stop, const rclcpp::Time & stamp)
{
  if (!heartbeat_pub_ || !heartbeat_pub_->is_activated()) {
    return;
  }
  ProtectiveStopHeartbeat heartbeat;
  heartbeat.stamp = stamp;
  heartbeat.stop = stop;
  heartbeat_pub_->publish(heartbeat);
}

void MachineBridgeNode::diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const auto & snapshot = last_snapshot_;
  using diagnostic_msgs::msg::DiagnosticStatus;
  if (!snapshot.reachable) {
    stat.summary(
      DiagnosticStatus::ERROR,
      "backend unreachable — ROS blind "
      "(machine still enforces independently)");
  } else if (snapshot.relay.fault_a || snapshot.relay.fault_b) {
    stat.summary(DiagnosticStatus::ERROR, "relay feedback fault");
  } else if (snapshot.relay.mismatch > 0) {
    stat.summary(DiagnosticStatus::WARN, "lockstep mismatch present");
  } else if (snapshot.running) {
    stat.summary(DiagnosticStatus::OK, "running");
  } else {
    stat.summary(DiagnosticStatus::OK, snapshot.status_reason);
  }
  stat.add("backend", backend_kind_);
  stat.add("running", snapshot.running);
  stat.add("active_remotes", snapshot.active_remotes);
  stat.add("relay_stop", snapshot.relay.relay_stop);
  stat.add("mismatch", snapshot.relay.mismatch);
}

rcl_interfaces::msg::SetParametersResult MachineBridgeNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  MachineTiming proposed = timing_;
  bool timing_changed = false;
  for (const auto & parameter : params) {
    if (parameter.get_name() == "timing.heartbeat_ms") {
      proposed.heartbeat_ms = static_cast<uint64_t>(parameter.as_int());
      timing_changed = true;
    } else if (parameter.get_name() == "timing.max_missed") {
      proposed.max_missed = static_cast<uint16_t>(parameter.as_int());
      timing_changed = true;
    } else if (parameter.get_name() == "timing.min_stop_ms") {
      proposed.min_stop_ms = static_cast<uint64_t>(parameter.as_int());
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
