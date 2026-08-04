// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Unit tests for timing_within_floors — the machine's runtime-config safety
// envelope (SR-M-01). A change that would defeat the arming gesture
// (min_stop_ms below the floor) or the stop latency (heartbeat window too wide,
// too many missed) must be rejected. Parity with the host runner's cfg_validate.

#include <gtest/gtest.h>

#include <string>

#include "protective_stop_machine/timing_floors.hpp"

using protective_stop_machine::MachineTiming;
using protective_stop_machine::timing_within_floors;
namespace floor = protective_stop_machine::floor;

static MachineTiming mk(uint64_t hb, uint16_t mm, uint64_t ms)
{
  MachineTiming t;
  t.heartbeat_ms = hb;
  t.max_missed = mm;
  t.min_stop_ms = ms;
  return t;
}

TEST(TimingFloors, ValidDefaultsAccepted)
{
  std::string r = "dirty";
  EXPECT_TRUE(timing_within_floors(mk(400, 3, 500), r));
  EXPECT_TRUE(r.empty());  // reason cleared on success
}

TEST(TimingFloors, BoundariesAccepted)
{
  std::string r;
  EXPECT_TRUE(
    timing_within_floors(
      mk(
        floor::kMinHeartbeatMs, floor::kMinMaxMissed,
        floor::kMinStopFloorMs), r));
  EXPECT_TRUE(timing_within_floors(mk(floor::kMaxHeartbeatMs, floor::kMaxMaxMissed, 100000), r));
}

TEST(TimingFloors, HeartbeatTooLowRejected)
{
  std::string r;
  EXPECT_FALSE(timing_within_floors(mk(floor::kMinHeartbeatMs - 1, 3, 500), r));
  EXPECT_NE(r.find("heartbeat_ms"), std::string::npos);
}

TEST(TimingFloors, HeartbeatTooHighRejected)  // window > 1 s defeats SF-1 latency
{
  std::string r;
  EXPECT_FALSE(timing_within_floors(mk(floor::kMaxHeartbeatMs + 1, 3, 500), r));
  EXPECT_NE(r.find("heartbeat_ms"), std::string::npos);
}

TEST(TimingFloors, MaxMissedZeroRejected)
{
  std::string r;
  EXPECT_FALSE(timing_within_floors(mk(400, 0, 500), r));
  EXPECT_NE(r.find("max_missed"), std::string::npos);
}

TEST(TimingFloors, MaxMissedTooHighRejected)
{
  std::string r;
  EXPECT_FALSE(timing_within_floors(mk(400, floor::kMaxMaxMissed + 1, 500), r));
  EXPECT_NE(r.find("max_missed"), std::string::npos);
}

TEST(TimingFloors, MinStopBelowFloorRejected)
{
  std::string r;
  EXPECT_FALSE(timing_within_floors(mk(400, 3, floor::kMinStopFloorMs - 1), r));
  EXPECT_NE(r.find("min_stop_ms"), std::string::npos);
}

TEST(TimingFloors, MinStopZeroDefeatArmingRejected)  // SF-3
{
  std::string r;
  EXPECT_FALSE(timing_within_floors(mk(400, 3, 0), r));
  EXPECT_NE(r.find("min_stop_ms"), std::string::npos);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
