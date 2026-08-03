// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Compile-time safety envelope for the machine's runtime timing config. A
// runtime change may only stay INSIDE this — timing_within_floors() rejects
// anything that would loosen it (min_stop below the arming floor defeats SF-3;
// an over-wide heartbeat window defeats SF-1 stop latency). Extracted from
// MachineBridgeNode so the check is unit-testable without a live node.
// Mirrors the host runner's cfg_validate floors. Ref: SR-M-01 (parity SR-H-03).

#ifndef PROTECTIVE_STOP_MACHINE__TIMING_FLOORS_HPP_
#define PROTECTIVE_STOP_MACHINE__TIMING_FLOORS_HPP_

#include <cstdint>
#include <string>

#include "protective_stop_machine/backend.hpp"

namespace protective_stop_machine
{
namespace floor
{
constexpr uint64_t kMinHeartbeatMs = 50;
constexpr uint64_t kMaxHeartbeatMs = 1000;  // window can't exceed 1 s
constexpr uint16_t kMinMaxMissed = 1;
constexpr uint16_t kMaxMaxMissed = 5;  // can't tolerate more than 5
constexpr uint64_t kMinStopFloorMs = 100;  // anti-blip arming delay floor
}  // namespace floor

// True iff t is within the safety floors; else false with a human-readable
// reason. Pure — no node, no ROS runtime.
inline bool timing_within_floors(const MachineTiming & t, std::string & reason)
{
  if (t.heartbeat_ms < floor::kMinHeartbeatMs || t.heartbeat_ms > floor::kMaxHeartbeatMs) {
    reason = "heartbeat_ms out of [" + std::to_string(floor::kMinHeartbeatMs) + "," +
             std::to_string(floor::kMaxHeartbeatMs) + "]";
    return false;
  }
  if (t.max_missed < floor::kMinMaxMissed || t.max_missed > floor::kMaxMaxMissed) {
    reason =
      "max_missed out of [" + std::to_string(floor::kMinMaxMissed) + "," + std::to_string(floor::kMaxMaxMissed) + "]";
    return false;
  }
  if (t.min_stop_ms < floor::kMinStopFloorMs) {
    reason = "min_stop_ms below safety floor " + std::to_string(floor::kMinStopFloorMs);
    return false;
  }
  reason.clear();
  return true;
}

}  // namespace protective_stop_machine

#endif  // PROTECTIVE_STOP_MACHINE__TIMING_FLOORS_HPP_
