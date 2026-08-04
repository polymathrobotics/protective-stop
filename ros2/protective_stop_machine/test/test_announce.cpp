// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Fleet check-in payload builder tests. Network-free: exercises only the pure
// build_announce_payload / json_escape helpers, never a live POST. Locks in that
// the core {"name","port"} pair matches the host runner's announce byte-for-byte
// so the fleet console treats the software machine identically, and that the
// additive machine/status fields are emitted for the machines/overview.
#include <gtest/gtest.h>

#include <string>

#include "protective_stop_machine/announce.hpp"

using protective_stop_machine::MachineSnapshot;
using protective_stop_machine::RemoteInfo;
using protective_stop_machine::build_announce_payload;
using protective_stop_machine::json_escape;

// A substring is present in the payload.
static bool has(const std::string & hay, const std::string & needle)
{
  return hay.find(needle) != std::string::npos;
}

// The host runner posts exactly {"name":"<name>","port":<port>} — the core the
// console keys on. Our payload must open with that same pair.
TEST(AnnouncePayload, CoreNameAndPortMatchHostRunner)
{
  MachineSnapshot snap;  // defaults: not running, no remotes
  const std::string body = build_announce_payload("bench-laptop", 8890, 0x01020304U, snap);
  EXPECT_TRUE(has(body, "\"name\":\"bench-laptop\""));
  EXPECT_TRUE(has(body, "\"port\":8890"));
  // The name/port pair opens the object, same layout as the host runner.
  EXPECT_EQ(body.rfind("{\"name\":\"bench-laptop\",\"port\":8890", 0), 0U);
}

// The machine identity + type let the console distinguish machines from remotes.
TEST(AnnouncePayload, EmitsMachineIdentityAndType)
{
  MachineSnapshot snap;
  const std::string body = build_announce_payload("m", 9000, 0x0102abcdU, snap);
  EXPECT_TRUE(has(body, "\"machine_id\":\"0102abcd\""));
  EXPECT_TRUE(has(body, "\"device_type\":\"machine\""));
}

// running + active_remotes reflect the snapshot (the console renders `running`).
TEST(AnnouncePayload, RunningAndRemoteCountFromSnapshot)
{
  MachineSnapshot stopped;
  stopped.running = false;
  stopped.active_remotes = 0;
  const std::string body_stopped = build_announce_payload("m", 8890, 1U, stopped);
  EXPECT_TRUE(has(body_stopped, "\"running\":false"));
  EXPECT_TRUE(has(body_stopped, "\"active_remotes\":0"));

  MachineSnapshot running;
  running.running = true;
  running.active_remotes = 2;
  const std::string body_running = build_announce_payload("m", 8890, 1U, running);
  EXPECT_TRUE(has(body_running, "\"running\":true"));
  EXPECT_TRUE(has(body_running, "\"active_remotes\":2"));
}

// Each bonded remote appears in the per-remote array the overview renders.
TEST(AnnouncePayload, PerRemoteSummary)
{
  MachineSnapshot snap;
  RemoteInfo one;
  one.device_id = "01d7eed0";
  one.bond_state = 2;
  one.stop_only = true;
  one.in_use = false;
  RemoteInfo two;
  two.device_id = "01aabbcc";
  two.bond_state = 3;
  two.stop_only = false;
  two.in_use = true;
  snap.remotes = {one, two};
  snap.active_remotes = 2;

  const std::string body = build_announce_payload("m", 8890, 1U, snap);
  EXPECT_TRUE(has(body, "\"id\":\"01d7eed0\""));
  EXPECT_TRUE(has(body, "\"bond_state\":2"));
  EXPECT_TRUE(has(body, "\"id\":\"01aabbcc\""));
  EXPECT_TRUE(has(body, "\"in_use\":true"));
  // Two entries -> exactly one separator between the remote objects.
  const std::string arr = body.substr(body.find("\"remotes\":["));
  EXPECT_TRUE(has(arr, "},{"));
}

// Empty remote list -> an empty array, no trailing comma.
TEST(AnnouncePayload, EmptyRemotesArray)
{
  MachineSnapshot snap;
  const std::string body = build_announce_payload("m", 8890, 1U, snap);
  EXPECT_TRUE(has(body, "\"remotes\":[]"));
}

// A name with JSON metacharacters is escaped so the body stays valid JSON.
TEST(AnnouncePayload, NameIsJsonEscaped)
{
  EXPECT_EQ(json_escape("a\"b\\c"), "a\\\"b\\\\c");
  EXPECT_EQ(json_escape("line\nbreak"), "line\\nbreak");

  MachineSnapshot snap;
  const std::string body = build_announce_payload("evil\"name", 8890, 1U, snap);
  EXPECT_TRUE(has(body, "\"name\":\"evil\\\"name\""));
}
