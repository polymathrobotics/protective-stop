// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Unit tests for HardwareMachineBackend::parse_state — the pure /state.json ->
// MachineSnapshot mapping that carries the machine's stop authority into ROS 2.
// Ref: SR-M-03 (unreachable/malformed -> blind, not "run") and DU-9 (tolerant
// parser; only relay_stop is safety-consumed and defaults to STOP).

#include <gtest/gtest.h>

#include <string>

#include "protective_stop_machine/hardware_backend.hpp"

using protective_stop_machine::HardwareMachineBackend;
using protective_stop_machine::MachineSnapshot;

static MachineSnapshot parse(const std::string & body)
{
  MachineSnapshot s;
  HardwareMachineBackend::parse_state(body, s);
  return s;
}

TEST(HwParse, RelayClosedRuns)
{
  const auto s = parse(R"({"relay_stop":false})");
  EXPECT_TRUE(s.reachable);
  EXPECT_TRUE(s.running);
  EXPECT_TRUE(s.relay.applicable);
  EXPECT_TRUE(s.relay.run);
  EXPECT_FALSE(s.relay.relay_stop);
  EXPECT_FALSE(s.need_stop);
}

TEST(HwParse, RelayOpenStops)
{
  const auto s = parse(R"({"relay_stop":true})");
  EXPECT_TRUE(s.reachable);
  EXPECT_FALSE(s.running);
  EXPECT_TRUE(s.relay.relay_stop);
}

// The single safety-consumed field defaults to STOP when absent — a device that
// omits relay_stop must never read as "run".
TEST(HwParse, MissingRelayDefaultsStopFailSafe)
{
  const auto s = parse(R"({"something_else":1})");
  EXPECT_TRUE(s.reachable);
  EXPECT_FALSE(s.running);
  EXPECT_TRUE(s.relay.relay_stop);
}

TEST(HwParse, FaultBitsAndMismatch)
{
  const auto s =
    parse(R"({"relay_stop":false,"relay_fault_a":true,"relay_fault_b":true,"pstop_mismatch":7})");
  EXPECT_TRUE(s.relay.fault_a);
  EXPECT_TRUE(s.relay.fault_b);
  EXPECT_EQ(s.relay.mismatch, 7u);
}

TEST(HwParse, BondedRemotesEnumerated)
{
  const auto s = parse(
    R"({"relay_stop":true,"bonded_remotes":[)"
    R"({"id":30928592,"state":2,"age_ms":101,"rtt_ms":209,"wg_rtt_ms":50},)"
    R"({"id":123,"state":1,"age_ms":5,"rtt_ms":9}]})");
  ASSERT_EQ(s.remotes.size(), 2u);
  EXPECT_EQ(s.active_remotes, 2u);
  EXPECT_EQ(s.remotes[0].device_id, "01d7eed0");  // 30928592 == 0x01D7EED0
  EXPECT_EQ(s.remotes[0].bond_state, 2);
  EXPECT_EQ(s.remotes[0].reply_age_ms, 101u);
  EXPECT_EQ(s.remotes[0].loop_rtt_ms, 209u);
  EXPECT_EQ(s.remotes[0].disco_rtt_ms, 50u);
  EXPECT_EQ(s.remotes[1].device_id, "0000007b");  // 123
  EXPECT_TRUE(s.need_stop);  // relay_stop && active>0
}

// id==0 rows are empty/ghost slots and must be skipped (the enumeration bug we
// fixed on real hardware: count real peers, not array length).
TEST(HwParse, SkipsZeroIdGhostEntries)
{
  const auto s =
    parse(R"({"relay_stop":false,"bonded_remotes":[{"id":0,"state":0},{"id":5,"state":2}]})");
  ASSERT_EQ(s.remotes.size(), 1u);
  EXPECT_EQ(s.active_remotes, 1u);
  EXPECT_EQ(s.remotes[0].device_id, "00000005");
}

// A non-object array item is skipped, not crashed on.
TEST(HwParse, SkipsNonObjectRemoteItems)
{
  const auto s = parse(R"({"relay_stop":false,"bonded_remotes":[5,"x",{"id":9,"state":2}]})");
  ASSERT_EQ(s.remotes.size(), 1u);
  EXPECT_EQ(s.remotes[0].device_id, "00000009");
}

// Malformed / empty body -> reachable=false (blind), never "running".
TEST(HwParse, MalformedJsonBlindNotRun)
{
  const auto s = parse("{not valid json");
  EXPECT_FALSE(s.reachable);
  EXPECT_FALSE(s.running);
}

TEST(HwParse, EmptyBodyUnreachable)
{
  const auto s = parse("");
  EXPECT_FALSE(s.reachable);
  EXPECT_FALSE(s.running);
}

// need_stop is asserted only when stopped AND at least one remote is bonded
// (an idle machine with no peers is not "awaiting an arming gesture").
TEST(HwParse, NeedStopFalseWhenNoRemotes)
{
  const auto s = parse(R"({"relay_stop":true})");
  EXPECT_FALSE(s.need_stop);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
