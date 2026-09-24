// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace protective_stop_machine
{

/// @brief Mirrors protective_stop_msg/ProtectiveStopStatus.
enum class MachineState : uint8_t
{
  /// Bonded remote(s), armed, cleared to run.
  ACTIVE = 0,
  /// Stopped or not cleared: NEED_STOP, or the backend is down.
  DEACTIVATED = 1,
  /// Backend unreachable or faulted.
  UNSTABLE = 2
};

struct RemoteInfo
{
  /// 8 hex digits, e.g. "01d7eed0".
  std::string device_id;
  /// 0 empty, 1 connecting, 2 bonded, 3 stopped.
  uint8_t bond_state{0};
  /// Owns the current arming cycle.
  bool in_use{false};
  bool stop_only{false};
  uint32_t reply_age_ms{0};
  uint32_t loop_rtt_ms{0};
  uint32_t disco_rtt_ms{0};
  uint32_t rebonds{0};
};

struct RelayInfo
{
  /// Set by the hardware backend; the software backend has no relays.
  bool applicable{false};
  bool run{false};
  bool relay_stop{true};
  bool fault_a{false};
  bool fault_b{false};
  uint32_t mismatch{0};
};

/// @brief One immutable read of machine state, mapped straight onto the ROS
/// messages.
struct MachineSnapshot
{
  bool reachable{false};
  bool running{false};
  bool need_stop{false};
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

/// @brief Runtime-settable timing envelope.
/// A change may only tighten: the node validates it against the SR-M-01 floors
/// before calling configure().
struct MachineTiming
{
  uint64_t heartbeat_ms{400};
  uint16_t max_missed{3};
  uint64_t min_stop_ms{500};
};

/// @brief Backend seam the node depends on, implemented by SoftwareMachineBackend
/// (hosts pstop_c) and HardwareMachineBackend (HTTP-polls the ESP32).
/// Implementations are rclcpp-free.
class IMachineBackend
{
public:
  virtual ~IMachineBackend() = default;

  /// @brief Begins servicing: software binds UDP and runs the machine loop,
  /// hardware starts polling.
  /// @return False on a fatal setup error.
  virtual bool start() = 0;

  /// @brief Stops servicing and leaves the machine in its SAFE state.
  /// Software stops the loop so remotes fail-safe; hardware stops polling while
  /// the ESP32 keeps enforcing. Idempotent.
  virtual void stop() = 0;

  /// @brief Thread-safe copy of the latest state.
  virtual MachineSnapshot snapshot() const = 0;

  /// @brief Applies an already safety-validated timing change.
  /// @param error Set when the backend cannot honour the change.
  /// @return False on an error or a refusal.
  virtual bool configure(const MachineTiming & timing, std::string & error) = 0;

  /// @brief Backend kind, "software" or "hardware".
  virtual const char * name() const = 0;
};

}  // namespace protective_stop_machine

