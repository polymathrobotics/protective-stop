// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// HardwareMachineBackend HTTP/threading tests — exercise the libcurl client
// (http_get / http_post / write_cb), the poll loop, start/stop and configure()
// WITHOUT a real ESP32 machn. A tiny loopback HTTP stub (test/http_stub.hpp)
// stands in for the device on 127.0.0.1, and a deliberately-closed port covers
// the unreachable / fail-safe paths. The pure /state.json mapping is covered
// separately in test_hardware_parse. Ref: SR-M-03 (unreachable -> blind).

#include <curl/curl.h>
#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>

#include "http_stub.hpp"
#include "protective_stop_machine/hardware_backend.hpp"

using protective_stop_machine::HardwareConfig;
using protective_stop_machine::HardwareMachineBackend;
using protective_stop_machine::MachineSnapshot;
using protective_stop_machine::MachineTiming;
using pstop_test::LoopbackHttpStub;

// A port on the IANA "discard" range that nothing listens on -> connect is
// refused immediately (fast, deterministic "device unreachable").
static constexpr const char * kClosedUrl = "http://127.0.0.1:9";

// Poll snapshot() until pred holds or the deadline passes.
template <typename Pred>
static bool wait_for(HardwareMachineBackend & be, Pred pred, std::chrono::milliseconds timeout = std::chrono::seconds(3))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred(be.snapshot())) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return pred(be.snapshot());
}

static HardwareConfig cfg_for(const std::string & url)
{
  HardwareConfig c;
  c.device_url = url;
  c.poll_hz = 50.0;  // poll fast so the test converges quickly
  c.http_timeout_s = 1.0;
  return c;
}

// A closed circuit (relay_stop=false) polled off the stub reads as running.
TEST(HwBackend, PollsStubRunning)
{
  LoopbackHttpStub stub(R"({"relay_stop":false})");
  HardwareMachineBackend be(cfg_for(stub.url()));
  ASSERT_TRUE(be.start());
  EXPECT_TRUE(wait_for(be, [](const MachineSnapshot & s) { return s.reachable && s.running; }));
  be.stop();
  // stop() wipes the snapshot -> unreachable (blind), never latched "run".
  EXPECT_FALSE(be.snapshot().reachable);
}

// relay_stop=true polled off the stub reads as stopped but reachable.
TEST(HwBackend, PollsStubStopped)
{
  LoopbackHttpStub stub(R"({"relay_stop":true})");
  HardwareMachineBackend be(cfg_for(stub.url()));
  ASSERT_TRUE(be.start());
  EXPECT_TRUE(wait_for(be, [](const MachineSnapshot & s) { return s.reachable && !s.running; }));
  be.stop();
}

// A body that carries a bonded remote is enumerated through the live poll path
// (write_cb -> parse_state -> snapshot), not just the static parse test.
TEST(HwBackend, PollsStubEnumeratesRemote)
{
  LoopbackHttpStub stub(R"({"relay_stop":true,"bonded_remotes":[{"id":5,"state":2}]})");
  HardwareMachineBackend be(cfg_for(stub.url()));
  ASSERT_TRUE(be.start());
  EXPECT_TRUE(wait_for(be, [](const MachineSnapshot & s) { return s.active_remotes == 1U; }));
  be.stop();
}

// Connection refused -> reachable=false with an http-status reason (fail-safe).
TEST(HwBackend, ClosedPortUnreachable)
{
  HardwareMachineBackend be(cfg_for(kClosedUrl));
  ASSERT_TRUE(be.start());
  EXPECT_TRUE(wait_for(
    be, [](const MachineSnapshot & s) { return !s.reachable && s.status_reason.find("unreachable") != std::string::npos; }));
  be.stop();
}

// A non-2xx device response is treated as unreachable, not "run".
TEST(HwBackend, Non2xxUnreachable)
{
  LoopbackHttpStub stub(R"({"relay_stop":false})", 503);
  HardwareMachineBackend be(cfg_for(stub.url()));
  ASSERT_TRUE(be.start());
  EXPECT_TRUE(wait_for(be, [](const MachineSnapshot & s) { return !s.reachable; }));
  be.stop();
}

// start() is idempotent; a second call while running is a no-op success.
TEST(HwBackend, StartIdempotent)
{
  LoopbackHttpStub stub(R"({"relay_stop":false})");
  HardwareMachineBackend be(cfg_for(stub.url()));
  ASSERT_TRUE(be.start());
  EXPECT_TRUE(be.start());  // already running -> true, no second thread
  be.stop();
}

// stop() without a start() is a safe no-op (the exchange guard).
TEST(HwBackend, StopWithoutStartIsSafe)
{
  HardwareMachineBackend be(cfg_for(kClosedUrl));
  be.stop();  // must not join a non-existent thread
  EXPECT_FALSE(be.snapshot().reachable);
}

// configure() POSTs the (already-validated) timing envelope; a 2xx device ->
// success. Also asserts the JSON the backend actually formatted on the wire.
TEST(HwBackend, ConfigureSuccessViaStub)
{
  LoopbackHttpStub stub(R"({"ok":true})");
  HardwareMachineBackend be(cfg_for(stub.url()));
  MachineTiming t;
  t.heartbeat_ms = 300;
  t.max_missed = 2;
  t.min_stop_ms = 600;
  std::string err = "dirty";
  EXPECT_TRUE(be.configure(t, err));
  EXPECT_TRUE(err.empty());
  const std::string sent = stub.last_request_body();
  EXPECT_NE(sent.find("\"heartbeat_ms\":300"), std::string::npos);
  EXPECT_NE(sent.find("\"max_missed\":2"), std::string::npos);
  EXPECT_NE(sent.find("\"min_stop_ms\":600"), std::string::npos);
}

// configure() to an unreachable device fails cleanly with a reason (the seam
// reports the device's non-acceptance rather than silently succeeding).
TEST(HwBackend, ConfigureUnreachableFails)
{
  HardwareMachineBackend be(cfg_for(kClosedUrl));
  MachineTiming t;
  std::string err;
  EXPECT_FALSE(be.configure(t, err));
  EXPECT_NE(err.find("http"), std::string::npos);
}

// A device that rejects the config (non-2xx) surfaces its status in the reason.
TEST(HwBackend, ConfigureDeviceRejects)
{
  LoopbackHttpStub stub(R"({"error":"unsupported"})", 500);
  HardwareMachineBackend be(cfg_for(stub.url()));
  MachineTiming t;
  std::string err;
  EXPECT_FALSE(be.configure(t, err));
  EXPECT_NE(err.find("500"), std::string::npos);
}

// With an admin password set, both HTTP helpers attach HTTP basic auth. The stub
// accepts any credentials; this covers the auth-header branch of http_get.
TEST(HwBackend, PollWithBasicAuth)
{
  LoopbackHttpStub stub(R"({"relay_stop":false})");
  HardwareConfig c = cfg_for(stub.url());
  c.admin_user = "admin";
  c.admin_pass = "s3cret";
  HardwareMachineBackend be(c);
  ASSERT_TRUE(be.start());
  EXPECT_TRUE(wait_for(be, [](const MachineSnapshot & s) { return s.reachable; }));
  be.stop();
}

// The auth-header branch of http_post (configure proxy).
TEST(HwBackend, ConfigureWithBasicAuth)
{
  LoopbackHttpStub stub(R"({"ok":true})");
  HardwareConfig c = cfg_for(stub.url());
  c.admin_user = "admin";
  c.admin_pass = "s3cret";
  HardwareMachineBackend be(c);
  MachineTiming t;
  std::string err;
  EXPECT_TRUE(be.configure(t, err));
}

TEST(HwBackend, NameIsHardware)
{
  HardwareMachineBackend be(cfg_for(kClosedUrl));
  EXPECT_STREQ(be.name(), "hardware");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  // Mirror main(): curl_global_* are process-global and not thread-safe, so do
  // them once before any backend spins its poll thread.
  curl_global_init(CURL_GLOBAL_DEFAULT);
  const int rc = RUN_ALL_TESTS();
  curl_global_cleanup();
  return rc;
}
