// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "protective_stop_machine/announce.hpp"
#include "protective_stop_machine/backend.hpp"
#include "protective_stop_machine/fleet_checkin.hpp"
#include "protective_stop_machine/protective_stop_machine_parameters.hpp"
#include "protective_stop_msg/msg/bonded_remote_array.hpp"
#include "protective_stop_msg/msg/machine_relay_status.hpp"
#include "protective_stop_msg/msg/protective_stop_heartbeat.hpp"
#include "protective_stop_msg/msg/protective_stop_status.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace protective_stop_machine
{

/// @brief Managed node presenting one ROS 2 surface over either machine
/// backend. ROS glue only; machine logic lives behind IMachineBackend.
/// See docs/MACHINE_ROS2_NODE_DESIGN.md.
class MachineBridgeNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit MachineBridgeNode(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

private:
  bool build_backend(std::string & error);
  void publish_tick();
  void publish_heartbeat(bool stop, const rclcpp::Time & stamp);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /// @brief Applies a live timing.* set by pushing it to the backend.
  /// The SR-M-01 floors are enforced by the generated ParamListener before this
  /// runs; a backend refusal is the only rejection made here.
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);

  ParamListener param_listener_;
  Params params_;

  // config
  std::string backend_kind_;
  std::string frame_id_;
  MachineTiming timing_;
  MachineSnapshot last_snapshot_;

  /// Fleet announce, software backend only. Resolved at configure time; the
  /// thread runs only while ACTIVE.
  AnnounceConfig announce_config_;
  int announce_port_{0};
  uint32_t machine_id_{0};

  /// Fleet device check-in, software backend only, additive to the announcer.
  /// Resolved at configure time; the thread runs only while ACTIVE.
  FleetCheckinConfig fleet_config_;

  std::unique_ptr<IMachineBackend> backend_;
  std::unique_ptr<MachineAnnouncer> announcer_;
  std::unique_ptr<FleetCheckin> fleet_checkin_;

  rclcpp_lifecycle::LifecyclePublisher<protective_stop_msg::msg::ProtectiveStopStatus>::SharedPtr
    state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<protective_stop_msg::msg::MachineRelayStatus>::SharedPtr
    relay_pub_;
  rclcpp_lifecycle::LifecyclePublisher<protective_stop_msg::msg::BondedRemoteArray>::SharedPtr
    remotes_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    protective_stop_msg::msg::ProtectiveStopHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  std::shared_ptr<diagnostic_updater::Updater> diagnostics_updater_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace protective_stop_machine

