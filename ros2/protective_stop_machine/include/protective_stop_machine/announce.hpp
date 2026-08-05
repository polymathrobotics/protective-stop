// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// MachineAnnouncer — optional periodic check-in to a central fleet console so an
// operator dashboard shows this software machine as RUNNING, exactly like the
// host runner's announce (host/machine_app_runner.c: announce_thread /
// announce_post_once) and the ESP32 machn's fleet check-in. One bearer-
// authenticated POST every interval_s; the console attributes the source IP
// itself and derives `running` / `announce_age_s` from the check-in stream.
//
// Deliberately OUTSIDE the safety loop: it runs on its own thread, links only
// libcurl (already a dependency), and any failure only logs — the pstop safety
// protocol never depends on it. Fully OFF when the URL is empty, so a non-fleet
// deployment is unaffected (default-disabled, opt-in).
//
// The payload builder (build_announce_payload) is header-inline and rclcpp-free
// so it is unit-testable with no network and no ROS runtime, matching the
// repo's operator_policy testable-logic pattern.
#ifndef PROTECTIVE_STOP_MACHINE__ANNOUNCE_HPP_
#define PROTECTIVE_STOP_MACHINE__ANNOUNCE_HPP_

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <thread>

#include "protective_stop_machine/backend.hpp"

namespace protective_stop_machine
{

// Deployment values for the check-in. Prefer the environment for URL + key file
// (PSTOP_ANNOUNCE_URL / PSTOP_ANNOUNCE_KEY_FILE) so secrets stay out of committed
// config — mirrors the host runner's convention. Disabled when url is empty.
struct AnnounceConfig
{
  std::string url;  // http://host[:port]/path — empty = DISABLED
  std::string key_file;  // path whose first line is the bearer token (chmod 600)
  std::string name;  // display name on the console; empty = this host's hostname
  int interval_s{60};  // seconds between check-ins
  double http_timeout_s{10.0};
};

// Escape a string for embedding in a JSON string literal. The console keys on
// `name`, which is operator-supplied, so escape defensively rather than trust it
// (the host runner trusted the hostname; we do not).
inline std::string json_escape(const std::string & in)
{
  std::string out;
  out.reserve(in.size() + 8);
  for (char ch : in) {
    switch (ch) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", ch);
          out += buf;
        } else {
          out += ch;
        }
    }
  }
  return out;
}

// Build the check-in JSON body. The core `{"name","port"}` pair is byte-for-byte
// what the host runner sends (announce_post_once), so a minimal console treats
// the software machine identically. The additional fields are ADDITIVE and
// non-breaking (a console that ignores unknown keys still sees name+port):
//   device_type : "machine" so the console distinguishes machines from remotes
//                 (planned check-in field, docs/MACHINE_ESP32_DESIGN.md §Fleet).
//   machine_id  : this machine's 32-bit pstop id, hex (identity aid).
//   running     : robot cleared to move (armed, no stop).
//   active_remotes + remotes[] : the bonded-remote summary the console's
//                 machines/overview renders per-remote.
inline std::string build_announce_payload(
  const std::string & name, int port, uint32_t machine_id, const MachineSnapshot & snap)
{
  char idbuf[16];
  std::snprintf(idbuf, sizeof(idbuf), "%08x", machine_id);

  std::string out = "{\"name\":\"";
  out += json_escape(name);
  out += "\",\"port\":";
  out += std::to_string(port);
  out += ",\"machine_id\":\"";
  out += idbuf;
  out += "\",\"device_type\":\"machine\",\"running\":";
  out += snap.running ? "true" : "false";
  out += ",\"active_remotes\":";
  out += std::to_string(snap.active_remotes);
  out += ",\"remotes\":[";
  bool first = true;
  for (const auto & remote : snap.remotes) {
    if (!first) {
      out += ',';
    }
    first = false;
    out += "{\"id\":\"";
    out += json_escape(remote.device_id);
    out += "\",\"bond_state\":";
    out += std::to_string(static_cast<int>(remote.bond_state));
    out += ",\"stop_only\":";
    out += remote.stop_only ? "true" : "false";
    out += ",\"in_use\":";
    out += remote.in_use ? "true" : "false";
    out += '}';
  }
  out += "]}";
  return out;
}

// Owns a background thread that POSTs the check-in every interval_s while the
// node is ACTIVE. Construct with the machine identity + a snapshot getter, then
// start()/stop() from the node's activate/deactivate transitions. Idempotent.
class MachineAnnouncer
{
public:
  MachineAnnouncer(
    AnnounceConfig cfg, int port, uint32_t machine_id,
    std::function<MachineSnapshot()> snapshot_fn);
  ~MachineAnnouncer();

  MachineAnnouncer(const MachineAnnouncer &) = delete;
  MachineAnnouncer & operator=(const MachineAnnouncer &) = delete;

  // Launch the check-in thread. Returns true if it started; false (non-fatal —
  // the caller only logs) when disabled (empty url) or the key file is
  // unreadable. Never fails node activation: announce is not on the safety path.
  bool start();

  // Stop the thread and join. Idempotent.
  void stop();

  bool enabled() const
  {
    return enabled_.load();
  }

private:
  void run();
  bool post_once(const std::string & payload, const std::string & bearer_key);

  AnnounceConfig cfg_;
  int port_;
  uint32_t machine_id_;
  std::function<MachineSnapshot()> snapshot_fn_;

  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> enabled_{false};
};

}  // namespace protective_stop_machine

#endif  // PROTECTIVE_STOP_MACHINE__ANNOUNCE_HPP_
