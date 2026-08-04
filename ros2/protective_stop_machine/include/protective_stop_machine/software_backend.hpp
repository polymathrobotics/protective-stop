// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef PROTECTIVE_STOP_MACHINE__SOFTWARE_BACKEND_HPP_
#define PROTECTIVE_STOP_MACHINE__SOFTWARE_BACKEND_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "protective_stop_machine/backend.hpp"

namespace protective_stop_machine
{

struct SoftwareConfig
{
  std::string bind_addr{"0.0.0.0"};
  int port{8890};
  uint32_t machine_id{0x01020304};
  MachineTiming timing;
  bool allow_unlisted{true};
  // Operator authorization (SAFETY). A bonded remote is ACCEPTED and
  // heartbeat-monitored but STOP-ONLY by default: it may command STOP, never
  // re-arm (STOP->OK). Only a remote whose 32-bit pstop id is on `operators`
  // gains re-arm authority. Empty `operators` (the default) => every remote is
  // stop-only = maximally safe out of the box. `default_stop_only` is the
  // policy applied to UNLISTED remotes; it defaults true and a deployment
  // should keep it true.
  bool default_stop_only{true};
  std::vector<uint32_t> operators{};
};

// Single source of truth for the operator-authorization decision: whether a
// bonded remote is STOP-ONLY (may STOP + is heartbeat-monitored, may NEVER
// re-arm) rather than a full operator (may also re-arm). A remote is stop-only
// unless its 32-bit pstop id is on the operator allowlist; an empty allowlist
// therefore makes every remote stop-only. Header-inline + pstop-free so it is
// directly unit-testable. Mirrors machn/main.c and host/machine_app_runner.c.
inline bool software_remote_is_stop_only(const SoftwareConfig & cfg, uint32_t remote_id)
{
  for (uint32_t op : cfg.operators) {
    if (op == remote_id) {
      return false;  // listed operator: full re-arm authority
    }
  }
  return cfg.default_stop_only;  // unlisted: stop-only by default (safe)
}

// The node itself IS the machine: this backend links pstop_c and runs a machine
// instance on a dedicated thread, binding UDP so remotes bond directly to it.
// If the node dies or stop() is called, the machine stops replying and remotes
// fail-safe on their own heartbeat timeout (see design §8).
class SoftwareMachineBackend : public IMachineBackend
{
public:
  explicit SoftwareMachineBackend(const SoftwareConfig & cfg);
  ~SoftwareMachineBackend() override;

  bool start() override;
  void stop() override;
  MachineSnapshot snapshot() const override;
  bool configure(const MachineTiming & timing, std::string & error) override;

  const char * name() const override
  {
    return "software";
  }

  // Opaque; fully defined in the .cpp (hides the pstop_c C types). Public so the
  // file-scope C callbacks in the .cpp can reach it, but the instance is private.
  struct Impl;

private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace protective_stop_machine

#endif  // PROTECTIVE_STOP_MACHINE__SOFTWARE_BACKEND_HPP_
