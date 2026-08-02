// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef PROTECTIVE_STOP_MACHINE__HARDWARE_BACKEND_HPP_
#define PROTECTIVE_STOP_MACHINE__HARDWARE_BACKEND_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "protective_stop_machine/backend.hpp"

namespace protective_stop_machine
{

struct HardwareConfig
{
  std::string device_url{"http://127.0.0.1"};   // ESP32 machn admin/state URL
  std::string admin_user{"admin"};
  std::string admin_pass;                        // from env, never a param file
  double poll_hz{5.0};
  double http_timeout_s{2.0};
};

// The ESP32 machn is the machine; this backend is an HTTP client. It polls
// /state.json to publish state, and proxies runtime config to the device admin
// API (control proxy). The ESP32 keeps enforcing STOP independently of ROS 2 —
// if polling stops or the device is unreachable, motion authority is unaffected
// (we only go blind: state() -> UNSTABLE).
class HardwareMachineBackend : public IMachineBackend
{
public:
  explicit HardwareMachineBackend(const HardwareConfig & cfg);
  ~HardwareMachineBackend() override;

  bool start() override;
  void stop() override;
  MachineSnapshot snapshot() const override;
  bool configure(const MachineTiming & timing, std::string & error) override;
  const char * name() const override {return "hardware";}

  // Pure /state.json -> MachineSnapshot mapping (no HTTP, no threading),
  // extracted so the safety-relevant parse (relay_stop -> run/stop, mismatch,
  // bonded-remote enumeration) is unit-testable. Sets out.reachable=false on a
  // parse error (fail-safe: an unparseable device reads as blind, not "run").
  // Ref: SR-M-03 / FMEA DU-9.
  static void parse_state(const std::string & body, MachineSnapshot & out);

private:
  void poll_loop();
  // HTTP helpers (libcurl). Return true + body on 2xx.
  bool http_get(const std::string & path, std::string & body, long & status);
  bool http_post(const std::string & path, const std::string & json,
    std::string & body, long & status);

  HardwareConfig cfg_;
  std::thread th_;
  std::atomic<bool> running_{false};
  mutable std::mutex mtx_;
  MachineSnapshot snap_;
};

}  // namespace protective_stop_machine

#endif  // PROTECTIVE_STOP_MACHINE__HARDWARE_BACKEND_HPP_
