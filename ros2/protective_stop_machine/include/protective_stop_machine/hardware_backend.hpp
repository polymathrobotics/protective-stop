// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "protective_stop_machine/backend.hpp"

namespace protective_stop_machine
{

/// @brief Deployment values for the hardware backend.
struct HardwareConfig
{
  /// ESP32 machn admin/state URL.
  std::string device_url{"http://127.0.0.1"};
  std::string admin_user{"admin"};
  /// Read from PSTOP_MACHINE_ADMIN_PASS, never a param file.
  std::string admin_pass;
  double poll_hz{5.0};
  double http_timeout_s{2.0};
};

/// @brief HTTP client to the ESP32 machn, which is itself the machine.
/// Polls /state.json for state and proxies runtime config to the device admin
/// API. The ESP32 enforces STOP independently of ROS 2, so an unreachable
/// device costs visibility only: state() reports UNSTABLE.
class HardwareMachineBackend : public IMachineBackend
{
public:
  explicit HardwareMachineBackend(const HardwareConfig & config);
  ~HardwareMachineBackend() override;

  bool start() override;
  void stop() override;
  MachineSnapshot snapshot() const override;
  bool configure(const MachineTiming & timing, std::string & error) override;

  const char * name() const override
  {
    return "hardware";
  }

  /// @brief Maps a /state.json body onto a snapshot. No HTTP, no threading.
  /// A parse error sets reachable=false. Ref: SR-M-03 / FMEA DU-9.
  static void parse_state(const std::string & body, MachineSnapshot & out_snapshot);

private:
  void poll_loop();
  /// @brief libcurl helpers. True with @p body filled on a 2xx.
  bool http_get(const std::string & path, std::string & body, long & status);
  bool http_post(
    const std::string & path, const std::string & json, std::string & body,
    long & status);

  HardwareConfig config_;
  std::thread poll_thread_;
  std::atomic<bool> running_{false};
  mutable std::mutex snapshot_mutex_;
  MachineSnapshot latest_snapshot_;
};

}  // namespace protective_stop_machine

