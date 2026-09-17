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
  // ADMISSION (optional): may a remote BOND at all? Two independent global
  // lists, both empty by default ("open": every remote is admitted).
  //   allowlist  non-empty => ONLY listed ids may bond ("paranoid" mode)
  //   denylist   listed ids may never bond; wins over the allowlist
  // A refused BOND is answered with UNBOND so the remote can show it.
  // Admission is NOT authority: whether a bonded remote may re-arm is the
  // REMOTE's own announced role (common/pstop_aux_channel.h), re-read on every
  // frame by the machine thread.
  std::vector<uint32_t> allowlist{};
  std::vector<uint32_t> denylist{};
};

// Single source of truth for the admission decision. Header-inline +
// pstop-free so it is directly unit-testable. Mirrors machn/main.c
// (dcs_admission_allows) and host/machine_app_runner.c.
inline bool software_remote_admitted(const SoftwareConfig & cfg, uint32_t remote_id)
{
  for (uint32_t id : cfg.denylist) {
    if (id == remote_id) {
      return false;  // deny wins
    }
  }
  if (cfg.allowlist.empty()) {
    return true;  // open mode
  }
  for (uint32_t id : cfg.allowlist) {
    if (id == remote_id) {
      return true;
    }
  }
  return false;
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
