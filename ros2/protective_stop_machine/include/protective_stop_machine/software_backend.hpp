// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "protective_stop_machine/backend.hpp"

namespace protective_stop_machine
{

/// @brief Deployment values for the in-process software backend.
struct SoftwareConfig
{
  /// Local address the UDP socket binds to.
  std::string bind_addr{"0.0.0.0"};
  /// UDP port remotes bond to.
  int port{8890};
  /// Device ID of this machine.
  uint32_t machine_id{0x01020304};
  /// Initial timing envelope.
  MachineTiming timing;
  /// Remote IDs permitted to bond; empty admits any remote.
  std::vector<uint32_t> allowlist{};
  /// Remote IDs refused regardless of the allowlist.
  std::vector<uint32_t> denylist{};
};

/// @brief Decides whether a remote may bond. Single source of truth, mirroring
/// machn/main.c (dcs_admission_allows) and host/machine_app_runner.c.
/// @return True when both lists are empty; the denylist wins over the allowlist.
inline bool software_remote_admitted(const SoftwareConfig & config, uint32_t remote_id)
{
  for (uint32_t listed_id : config.denylist) {
    if (listed_id == remote_id) {
      // deny wins
      return false;
    }
  }
  if (config.allowlist.empty()) {
    // open mode: no allowlist configured
    return true;
  }
  for (uint32_t listed_id : config.allowlist) {
    if (listed_id == remote_id) {
      return true;
    }
  }
  return false;
}

/// @brief Hosts pstop_c in-process, making the node itself the machine.
/// Runs a machine instance on a dedicated thread and binds UDP so remotes bond
/// directly to it. On stop() or node death the machine stops replying and
/// remotes fail-safe on their own heartbeat timeout (design §8).
class SoftwareMachineBackend : public IMachineBackend
{
public:
  /// Copies config; no I/O until start().
  explicit SoftwareMachineBackend(const SoftwareConfig & config);
  ~SoftwareMachineBackend() override;

  bool start() override;
  void stop() override;
  MachineSnapshot snapshot() const override;
  bool configure(const MachineTiming & timing, std::string & error) override;

  const char * name() const override
  {
    return "software";
  }

  /// @brief Opaque, defined in the .cpp, hiding the pstop_c C types.
  /// Public so the file-scope C callbacks can name it; the instance is private.
  struct Impl;

private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace protective_stop_machine
