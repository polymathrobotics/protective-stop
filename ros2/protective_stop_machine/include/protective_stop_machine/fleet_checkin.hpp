// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// FleetCheckin — optional periodic DEVICE CHECK-IN to pstop-fleet so a software
// machine registers there EXACTLY like an ESP32 machine does: a
// `device_type:"machine"` device with full metadata, not just the lighter
// [announce] overview ping. It mirrors the ESP32's fleet_ota_checkin()
// (components/microlink/src/ml_app.c): one bearer-authenticated POST to
// `<base>/api/v1/checkin` carrying the ESP32 check-in schema (device_id,
// app_version, idf_version, uptime_s, device_type, check_interval_s, local_ip,
// state, rollback_occurred), every check_interval_s, plus an immediate boot
// check-in. The fleet then parses the software machine identically to a chip.
//
// This is COMPLEMENTARY to MachineAnnouncer, not a replacement: the announcer
// feeds the console's live per-remote overview; the check-in registers the
// device record + advertises cadence/version the same way the firmware does.
//
// Deliberately OUTSIDE the safety loop — own thread, libcurl only (already a
// dependency), and any failure only logs. The pstop safety protocol never
// depends on it, and it is fully OFF when the base URL is empty (opt-in). No
// firmware OTA is attempted: a software machine has no ESP-OTA image, so an
// `update_available` directive from the fleet is logged for the operator and
// otherwise a no-op.
//
// The payload builder (build_checkin_payload) and state mapping (checkin_state)
// are header-inline and rclcpp-free so they are unit-testable with no network
// and no ROS runtime, matching the repo's announce / timing_floors pattern.
#ifndef PROTECTIVE_STOP_MACHINE__FLEET_CHECKIN_HPP_
#define PROTECTIVE_STOP_MACHINE__FLEET_CHECKIN_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <thread>

#include "protective_stop_machine/announce.hpp"  // json_escape + MachineSnapshot
#include "protective_stop_machine/backend.hpp"

namespace protective_stop_machine
{

// Deployment values for the fleet check-in. Prefer the environment for the base
// URL + key file (PSTOP_FLEET_CHECKIN_URL / PSTOP_FLEET_API_KEY_FILE) so the
// proprietary fleet URL/key stay out of committed config. Disabled when the base
// URL is empty. `base_url` is a BASE (e.g. http://fleet.example:8000); the client
// appends `/api/v1/checkin` itself, exactly as the ESP32 hardcodes that path.
struct FleetCheckinConfig
{
  std::string base_url;  // http://host[:port] — empty = DISABLED (opt-in)
  std::string key_file;  // path whose first line is the bearer token (chmod 600)
  std::string app_version;  // ament package version of protective_stop_machine
  std::string idf_version;  // runtime tag, e.g. "ros2-jazzy" (no IDF on a host)
  int interval_s{300};  // seconds between check-ins (ESP32 default 300)
  double http_timeout_s{15.0};  // match the ESP32's 15s check-in timeout
};

// Map the machine snapshot onto the ESP32 check-in `state` enum. The chip sends
// CONNECTED once its uplink is up, CONNECTING while bringing it up, IDLE before.
// The software machine has no VPN uplink of its own, so we report link *to a
// remote* instead: bonded/running -> CONNECTED, backend up but no bond yet ->
// CONNECTING, backend not reachable -> IDLE.
inline const char * checkin_state(const MachineSnapshot & snap)
{
  if (snap.running || snap.active_remotes > 0) {
    return "CONNECTED";
  }
  if (snap.reachable) {
    return "CONNECTING";
  }
  return "IDLE";
}

// Build the check-in JSON body, matching the ESP32 fleet_ota_checkin() schema so
// the fleet parses a software machine identically to a chip. Field mapping:
//   device_id         : this machine's 32-bit pstop id as 8 hex digits. NOTE the
//                       namespace differs from the ESP32's device_id, which is a
//                       12-hex WiFi MAC — a pstop id and a MAC never collide, and
//                       the fleet keys on the string, so mixing widths is safe.
//   app_version       : ament package version of protective_stop_machine.
//   idf_version       : "ros2-<ROS_DISTRO>" — there is no IDF; this tags the
//                       runtime the same slot the chip uses for its IDF version.
//   uptime_s          : process uptime in seconds.
//   device_type       : "machine" (same class token the chip machn sends).
//   check_interval_s  : our own cadence, so the fleet can judge staleness.
//   tailscale_ip      : sent EMPTY — a software host may be on Tailscale but the
//                       node has no handle to resolve its VPN IP; the key is kept
//                       present (not omitted) so the schema stays byte-identical.
//   local_ip          : first non-loopback IPv4 of this host (resolved by run()).
//   state             : checkin_state(snap) (see above).
//   rollback_occurred : always false — a software machine has no ESP-OTA image
//                       and therefore no rollback partition to have reverted.
// free_heap is deliberately OMITTED: it is an ESP heap metric with no meaningful
// analogue for a Linux process (gigabytes of virtual heap); 0 or any value would
// mislead an operator, and the fleet treats absent optional fields as unknown.
// `running` + `active_remotes` are ADDITIVE (mirroring the announce extras).
inline std::string build_checkin_payload(
  uint32_t machine_id,
  const std::string & app_version,
  const std::string & idf_version,
  uint64_t uptime_s,
  int check_interval_s,
  const std::string & local_ip,
  const std::string & tailscale_ip,
  const MachineSnapshot & snap)
{
  char idbuf[16];
  std::snprintf(idbuf, sizeof(idbuf), "%08x", machine_id);

  std::string out = "{\"device_id\":\"";
  out += idbuf;
  out += "\",\"app_version\":\"";
  out += json_escape(app_version);
  out += "\",\"idf_version\":\"";
  out += json_escape(idf_version);
  out += "\",\"uptime_s\":";
  out += std::to_string(uptime_s);
  out += ",\"device_type\":\"machine\",\"check_interval_s\":";
  out += std::to_string(check_interval_s);
  out += ",\"tailscale_ip\":\"";
  out += json_escape(tailscale_ip);
  out += "\",\"local_ip\":\"";
  out += json_escape(local_ip);
  out += "\",\"state\":\"";
  out += checkin_state(snap);
  out += "\",\"rollback_occurred\":false,\"running\":";
  out += snap.running ? "true" : "false";
  out += ",\"active_remotes\":";
  out += std::to_string(snap.active_remotes);
  out += "}";
  return out;
}

// Join a base URL and the fixed check-in path without doubling the slash. The
// ESP32 does `snprintf("%s/api/v1/checkin", backend)`; we mirror that but tolerate
// an operator-supplied trailing slash on the base.
inline std::string checkin_endpoint(const std::string & base_url)
{
  std::string base = base_url;
  while (!base.empty() && base.back() == '/') {
    base.pop_back();
  }
  return base + "/api/v1/checkin";
}

// Owns a background thread that POSTs the check-in every interval_s while the
// node is ACTIVE. Construct with the machine identity + a snapshot getter, then
// start()/stop() from the node's activate/deactivate transitions. Idempotent.
class FleetCheckin
{
public:
  FleetCheckin(
    FleetCheckinConfig cfg, uint32_t machine_id,
    std::function<MachineSnapshot()> snapshot_fn);
  ~FleetCheckin();

  FleetCheckin(const FleetCheckin &) = delete;
  FleetCheckin & operator=(const FleetCheckin &) = delete;

  // Launch the check-in thread. Returns true if it started; false (non-fatal —
  // the caller only logs) when disabled (empty base URL) or the key file is
  // unreadable. Never fails node activation: check-in is not on the safety path.
  bool start();

  // Stop the thread and join. Idempotent.
  void stop();

  bool enabled() const
  {
    return enabled_.load();
  }

private:
  void run();
  // POST the payload; on a 2xx, parse the response for OTA directives (log-only).
  bool post_once(
    const std::string & endpoint, const std::string & payload,
    const std::string & bearer_key);
  void handle_response(const std::string & body);

  FleetCheckinConfig cfg_;
  uint32_t machine_id_;
  std::function<MachineSnapshot()> snapshot_fn_;
  std::chrono::steady_clock::time_point start_time_{std::chrono::steady_clock::now()};

  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> enabled_{false};
};

}  // namespace protective_stop_machine

#endif  // PROTECTIVE_STOP_MACHINE__FLEET_CHECKIN_HPP_
