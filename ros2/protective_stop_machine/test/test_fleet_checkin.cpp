// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Fleet DEVICE check-in tests. Network-free: exercises the pure
// build_checkin_payload / checkin_state / checkin_endpoint helpers, plus the
// disabled-when-empty guard on the real FleetCheckin (which never posts when the
// base URL is empty). Locks in that the body matches the ESP32
// fleet_ota_checkin() schema (device_type="machine" + the required metadata) so
// pstop-fleet registers a software machine identically to a chip.
#include <gtest/gtest.h>

#include <string>

#include "protective_stop_machine/fleet_checkin.hpp"

using protective_stop_machine::build_checkin_payload;
using protective_stop_machine::checkin_endpoint;
using protective_stop_machine::checkin_state;
using protective_stop_machine::FleetCheckin;
using protective_stop_machine::FleetCheckinConfig;
using protective_stop_machine::MachineSnapshot;

// A substring is present in the payload.
static bool has(const std::string & hay, const std::string & needle)
{
  return hay.find(needle) != std::string::npos;
}

// The body must carry the ESP32 check-in schema so the fleet parses a software
// machine identically to a chip: device_type="machine" plus the required fields.
TEST(FleetCheckinPayload, MatchesEsp32MachineSchema)
{
  MachineSnapshot snap;
  const std::string body = build_checkin_payload(0x01020304U, "0.1.0", "ros2-jazzy", 42U, 300,
    "192.168.1.5", "", snap);

  EXPECT_TRUE(has(body, "\"device_type\":\"machine\""));
  EXPECT_TRUE(has(body, "\"device_id\":\"01020304\""));
  EXPECT_TRUE(has(body, "\"app_version\":\"0.1.0\""));
  EXPECT_TRUE(has(body, "\"idf_version\":\"ros2-jazzy\""));
  EXPECT_TRUE(has(body, "\"uptime_s\":42"));
  EXPECT_TRUE(has(body, "\"check_interval_s\":300"));
  EXPECT_TRUE(has(body, "\"local_ip\":\"192.168.1.5\""));
  EXPECT_TRUE(has(body, "\"state\":\""));
  EXPECT_TRUE(has(body, "\"rollback_occurred\":false"));
  // Additive extras carried over from the announce payload.
  EXPECT_TRUE(has(body, "\"running\":"));
  EXPECT_TRUE(has(body, "\"active_remotes\":"));
}

// device_id is the 32-bit pstop id as 8 hex digits (a different namespace from
// the ESP32's 12-hex MAC, but the fleet keys on the string).
TEST(FleetCheckinPayload, DeviceIdIsHexMachineId)
{
  MachineSnapshot snap;
  const std::string body = build_checkin_payload(0x0102abcdU, "1.2.3", "ros2-humble", 0U, 60, "",
    "", snap);
  EXPECT_TRUE(has(body, "\"device_id\":\"0102abcd\""));
}

// tailscale_ip is sent (empty), not omitted, so the schema stays byte-identical;
// free_heap is deliberately absent (no meaningful analogue for a Linux process).
TEST(FleetCheckinPayload, TailscaleEmptyPresentAndFreeHeapOmitted)
{
  MachineSnapshot snap;
  const std::string body = build_checkin_payload(1U, "0.1.0", "ros2-jazzy", 1U, 120, "10.0.0.2", "",
    snap);
  EXPECT_TRUE(has(body, "\"tailscale_ip\":\"\""));
  EXPECT_FALSE(has(body, "free_heap"));
}

// state derivation mirrors the ESP32's CONNECTED/CONNECTING/IDLE ladder, mapped
// onto the software machine's bond/backend state.
TEST(FleetCheckinState, DerivesFromSnapshot)
{
  MachineSnapshot idle;  // defaults: not reachable, not running
  idle.reachable = false;
  EXPECT_STREQ(checkin_state(idle), "IDLE");

  MachineSnapshot connecting;  // backend up, no bond yet
  connecting.reachable = true;
  connecting.running = false;
  connecting.active_remotes = 0;
  EXPECT_STREQ(checkin_state(connecting), "CONNECTING");

  MachineSnapshot bonded;  // a remote is bonded
  bonded.reachable = true;
  bonded.running = false;
  bonded.active_remotes = 1;
  EXPECT_STREQ(checkin_state(bonded), "CONNECTED");

  MachineSnapshot running;  // armed / cleared to move
  running.reachable = true;
  running.running = true;
  EXPECT_STREQ(checkin_state(running), "CONNECTED");

  // The derived state reaches the payload.
  const std::string body = build_checkin_payload(1U, "0.1.0", "ros2-jazzy", 0U, 300, "", "",
    running);
  EXPECT_TRUE(has(body, "\"state\":\"CONNECTED\""));
}

// running + active_remotes reflect the snapshot.
TEST(FleetCheckinPayload, RunningAndRemoteCountFromSnapshot)
{
  MachineSnapshot snap;
  snap.running = true;
  snap.active_remotes = 3;
  const std::string body = build_checkin_payload(1U, "0.1.0", "ros2-jazzy", 0U, 300, "", "", snap);
  EXPECT_TRUE(has(body, "\"running\":true"));
  EXPECT_TRUE(has(body, "\"active_remotes\":3"));
}

// The client appends the fixed /api/v1/checkin path to the base, exactly like the
// ESP32, and tolerates an operator-supplied trailing slash on the base.
TEST(FleetCheckinEndpoint, AppendsFixedPath)
{
  EXPECT_EQ(checkin_endpoint("http://fleet.example:8000"),
    "http://fleet.example:8000/api/v1/checkin");
  EXPECT_EQ(checkin_endpoint("http://fleet.example:8000/"),
    "http://fleet.example:8000/api/v1/checkin");
  EXPECT_EQ(checkin_endpoint("http://fleet.example:8000///"),
    "http://fleet.example:8000/api/v1/checkin");
}

// A version/tag with JSON metacharacters is escaped so the body stays valid JSON.
TEST(FleetCheckinPayload, VersionFieldsAreJsonEscaped)
{
  MachineSnapshot snap;
  const std::string body = build_checkin_payload(1U, "0.1\"x", "ros2-\\weird", 0U, 300, "", "",
    snap);
  EXPECT_TRUE(has(body, "\"app_version\":\"0.1\\\"x\""));
  EXPECT_TRUE(has(body, "\"idf_version\":\"ros2-\\\\weird\""));
}

// An empty base URL disables the check-in: start() returns false, enabled()
// stays false, and no thread/POST is created. This is the network-free
// disabled-opt-in path.
TEST(FleetCheckinLifecycle, DisabledWhenBaseUrlEmpty)
{
  FleetCheckinConfig cfg;  // base_url empty by default
  FleetCheckin checkin(cfg, 0x01020304U, []() {return MachineSnapshot{};});
  EXPECT_FALSE(checkin.start());
  EXPECT_FALSE(checkin.enabled());
}
