// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef PROTECTIVE_STOP_MACHINE__BACKEND_HPP_
#define PROTECTIVE_STOP_MACHINE__BACKEND_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace protective_stop_machine
{

// Mirrors protective_stop_msgs/ProtectiveStopStatus.
enum class MachineState : uint8_t
{
  ACTIVE = 0,  // bonded remote(s), armed, cleared to run
  DEACTIVATED = 1,  // stopped / not cleared (NEED_STOP, or backend down)
  UNSTABLE = 2  // backend unreachable / fault
};

struct RemoteInfo
{
  std::string device_id;  // hex UUID, e.g. "01d7eed0"
  uint8_t bond_state{0};  // 0 empty, 1 connecting, 2 bonded
  bool in_use{false};  // owns the current arming cycle
  bool stop_only{false};
  uint32_t reply_age_ms{0};
  uint32_t loop_rtt_ms{0};
  uint32_t disco_rtt_ms{0};
  uint32_t rebonds{0};
};

struct RelayInfo
{
  bool applicable{false};  // false on the software backend (no relays)
  bool run{false};  // circuit closed / cleared to move
  bool relay_stop{true};  // stop asserted (circuit open)
  bool fault_a{false};
  bool fault_b{false};
  uint32_t mismatch{0};
};

// One immutable read of machine state. Backends publish this; the node maps
// it straight onto the ROS messages.
struct MachineSnapshot
{
  bool reachable{false};  // backend healthy/reachable
  bool running{false};  // robot cleared to move (armed, no stop)
  bool need_stop{false};  // NEED_STOP: awaiting an arming gesture
  std::string status_reason;
  RelayInfo relay;
  std::vector<RemoteInfo> remotes;
  uint32_t active_remotes{0};

  MachineState state() const
  {
    if (!reachable) {
      return MachineState::UNSTABLE;
    }
    return running ? MachineState::ACTIVE : MachineState::DEACTIVATED;
  }
};

// Runtime-settable timing envelope. The node validates any change against a
// compile-time safe floor before calling configure() (can only tighten).
struct MachineTiming
{
  uint64_t heartbeat_ms{400};
  uint16_t max_missed{3};
  uint64_t min_stop_ms{500};
};

// Backend seam: the node depends only on this. Two impls — software (hosts
// pstop_c) and hardware (HTTP-polls the ESP32). No rclcpp here (pure logic).
class IMachineBackend
{
public:
  virtual ~IMachineBackend() = default;

  // Begin servicing (software: bind + machine loop; hardware: start polling).
  // Returns false on a fatal setup error.
  virtual bool start() = 0;

  // Stop servicing and leave the machine in its SAFE state. Software: stop the
  // loop so remotes fail-safe; hardware: stop polling (ESP32 keeps enforcing).
  virtual void stop() = 0;

  // Thread-safe copy of the latest state.
  virtual MachineSnapshot snapshot() const = 0;

  // Apply a (already safety-validated) timing change. Returns false + reason on
  // an error or if the backend cannot honour it.
  virtual bool configure(const MachineTiming & timing, std::string & error) = 0;

  virtual const char * name() const = 0;
};

}  // namespace protective_stop_machine

#endif  // PROTECTIVE_STOP_MACHINE__BACKEND_HPP_
