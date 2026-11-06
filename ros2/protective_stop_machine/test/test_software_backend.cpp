// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Software backend heartbeat-timeout stop counter.
// Drives the backend over loopback UDP with a hand-built BOND frame.

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "bond_remote.hpp"  // NOLINT(build/include_subdir)
#include "protective_stop_machine/software_backend.hpp"

using protective_stop_machine::MachineSnapshot;
using protective_stop_machine::SoftwareConfig;
using protective_stop_machine::SoftwareMachineBackend;

namespace
{

constexpr int SILENT_REMOTE_PORT = 18931;
constexpr int NO_REMOTE_PORT = 18932;
constexpr uint32_t REMOTE_ID = 0x01d7791cU;

}  // namespace

TEST(SoftwareBackend, SilentRemoteCountsOneHeartbeatTimeoutStop)
{
  SoftwareConfig config;
  config.bind_addr = "127.0.0.1";
  config.port = SILENT_REMOTE_PORT;
  config.timing.heartbeat_ms = 50;
  config.timing.max_missed = 3;
  SoftwareMachineBackend backend(config);
  ASSERT_TRUE(backend.start());
  EXPECT_EQ(0U, backend.snapshot().heartbeat_timeout_stops);

  ASSERT_TRUE(pstop_test::bond_remote(config.port, config.machine_id, REMOTE_ID));

  // The remote sends nothing after the BOND.
  MachineSnapshot snapshot;
  for (int i = 0; 100 > i && 0U == snapshot.heartbeat_timeout_stops; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    snapshot = backend.snapshot();
  }
  EXPECT_EQ(1U, snapshot.heartbeat_timeout_stops);
  EXPECT_FALSE(snapshot.running);

  // The count stays at 1 across later polls.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(1U, backend.snapshot().heartbeat_timeout_stops);
  backend.stop();
}

TEST(SoftwareBackend, NoRemoteNoHeartbeatTimeoutStops)
{
  SoftwareConfig config;
  config.bind_addr = "127.0.0.1";
  config.port = NO_REMOTE_PORT;
  config.timing.heartbeat_ms = 50;
  SoftwareMachineBackend backend(config);
  ASSERT_TRUE(backend.start());
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  EXPECT_EQ(0U, backend.snapshot().heartbeat_timeout_stops);
  backend.stop();
}
