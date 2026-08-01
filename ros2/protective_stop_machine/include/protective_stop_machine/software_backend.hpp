// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef PROTECTIVE_STOP_MACHINE__SOFTWARE_BACKEND_HPP_
#define PROTECTIVE_STOP_MACHINE__SOFTWARE_BACKEND_HPP_

#include <memory>
#include <string>

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
};

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
  const char * name() const override {return "software";}

  // Opaque; fully defined in the .cpp (hides the pstop_c C types). Public so the
  // file-scope C callbacks in the .cpp can reach it, but the instance is private.
  struct Impl;

private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace protective_stop_machine

#endif  // PROTECTIVE_STOP_MACHINE__SOFTWARE_BACKEND_HPP_
