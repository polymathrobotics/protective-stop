// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef PROTECTIVE_STOP_MACHINE__MACHINE_BRIDGE_NODE_HPP_
#define PROTECTIVE_STOP_MACHINE__MACHINE_BRIDGE_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "protective_stop_machine/backend.hpp"
#include "protective_stop_machine/protective_stop_machine_parameters.hpp"
#include "protective_stop_msgs/msg/bonded_remote_array.hpp"
#include "protective_stop_msgs/msg/machine_relay_status.hpp"
#include "protective_stop_msgs/msg/protective_stop_status.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace protective_stop_machine
{

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
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  // Live timing.* sets: floors are enforced declaratively by the generated
  // ParamListener; this handler only pushes an accepted change to the backend
  // and rejects it if the backend refuses (which the ParamListener can't do).
  rcl_interfaces::msg::SetParametersResult on_set_parameters(const std::vector<rclcpp::Parameter> & params);

  // Typed, validated parameters (see protective_stop_machine_params.yaml).
  ParamListener param_listener_;
  Params params_;

  // config
  std::string backend_kind_;
  std::string frame_id_;
  MachineTiming timing_;
  MachineSnapshot last_snapshot_;

  std::unique_ptr<IMachineBackend> backend_;

  rclcpp_lifecycle::LifecyclePublisher<protective_stop_msgs::msg::ProtectiveStopStatus>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<protective_stop_msgs::msg::MachineRelayStatus>::SharedPtr relay_pub_;
  rclcpp_lifecycle::LifecyclePublisher<protective_stop_msgs::msg::BondedRemoteArray>::SharedPtr remotes_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  std::shared_ptr<diagnostic_updater::Updater> diag_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace protective_stop_machine

#endif  // PROTECTIVE_STOP_MACHINE__MACHINE_BRIDGE_NODE_HPP_
