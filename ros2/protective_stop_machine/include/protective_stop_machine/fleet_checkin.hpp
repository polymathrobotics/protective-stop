// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <thread>

#include "protective_stop_machine/announce.hpp"
#include "protective_stop_machine/backend.hpp"

namespace protective_stop_machine
{

/// @brief Deployment values for the fleet check-in. Disabled when base_url is
/// empty. PSTOP_CHECKIN_URL and PSTOP_CHECKIN_API_KEY_FILE override base_url
/// and key_file.
/// base_url is a base such as http://fleet.example:8000; the client appends
/// /api/v1/checkin itself.
struct FleetCheckinConfig
{
  /// http://host[:port]. Empty disables the check-in.
  std::string base_url;
  /// Path whose first line is the bearer token, chmod 600.
  std::string key_file;
  /// ament package version of protective_stop_machine.
  std::string app_version;
  /// Runtime tag, e.g. "ros2-jazzy".
  std::string idf_version;
  /// Seconds between check-ins. The ESP32 default is 300.
  int interval_s{300};
  /// Matches the ESP32's 15s check-in timeout.
  double http_timeout_s{15.0};
};

/// @brief Maps a snapshot onto the ESP32 check-in state enum.
/// A software machine has no VPN uplink, so the link reported is the one to a
/// remote: bonded or running is CONNECTED, backend up without a bond is
/// CONNECTING, an unreachable backend is IDLE.
inline const char * checkin_state(const MachineSnapshot & snapshot)
{
  if (snapshot.running || snapshot.active_remotes > 0) {
    return "CONNECTED";
  }
  if (snapshot.reachable) {
    return "CONNECTING";
  }
  return "IDLE";
}

/// @brief Builds the check-in JSON body in the ESP32 fleet_ota_checkin() schema.
/// Field mapping:
///   device_id         this machine's 32-bit pstop id, 8 hex digits, where the
///                     chip sends a 12-hex WiFi MAC. The fleet keys on the
///                     string, so the widths may differ.
///   app_version       ament package version of protective_stop_machine.
///   idf_version       "ros2-<ROS_DISTRO>", tagging the runtime in the slot the
///                     chip uses for its IDF version.
///   uptime_s          process uptime in seconds.
///   device_type       "machine", the class token the chip machn sends.
///   check_interval_s  this client's cadence, for judging staleness.
///   tailscale_ip      sent empty; the key stays present so the schema is
///                     byte-identical to the chip's.
///   local_ip          first non-loopback IPv4 of this host.
///   state             checkin_state(snapshot).
///   rollback_occurred always false; there is no ESP-OTA partition.
///   running, active_remotes  additive, mirroring the announce extras.
/// free_heap is omitted: the fleet treats an absent optional field as unknown.
inline std::string build_checkin_payload(
  uint32_t machine_id,
  const std::string & app_version,
  const std::string & idf_version,
  uint64_t uptime_s,
  int check_interval_s,
  const std::string & local_ip,
  const std::string & tailscale_ip,
  const MachineSnapshot & snapshot)
{
  char device_id_text[16];
  std::snprintf(device_id_text, sizeof(device_id_text), "%08x", machine_id);

  std::string payload = "{\"device_id\":\"";
  payload += device_id_text;
  payload += "\",\"app_version\":\"";
  payload += json_escape(app_version);
  payload += "\",\"idf_version\":\"";
  payload += json_escape(idf_version);
  payload += "\",\"uptime_s\":";
  payload += std::to_string(uptime_s);
  payload += ",\"device_type\":\"machine\",\"check_interval_s\":";
  payload += std::to_string(check_interval_s);
  payload += ",\"tailscale_ip\":\"";
  payload += json_escape(tailscale_ip);
  payload += "\",\"local_ip\":\"";
  payload += json_escape(local_ip);
  payload += "\",\"state\":\"";
  payload += checkin_state(snapshot);
  payload += "\",\"rollback_occurred\":false,\"running\":";
  payload += snapshot.running ? "true" : "false";
  payload += ",\"active_remotes\":";
  payload += std::to_string(snapshot.active_remotes);
  payload += "}";
  return payload;
}

/// @brief Joins a base URL and the fixed check-in path.
/// Trailing slashes on the base are trimmed.
inline std::string checkin_endpoint(const std::string & base_url)
{
  std::string trimmed_base = base_url;
  while (!trimmed_base.empty() && trimmed_base.back() == '/') {
    trimmed_base.pop_back();
  }
  return trimmed_base + "/api/v1/checkin";
}

/// @brief Periodically POSTs a device check-in to `<base_url>/api/v1/checkin`.
/// The payload follows the ESP32 check-in schema, so the fleet registers this
/// software machine as a `device_type:"machine"` device. An `update_available`
/// directive in the response is logged only.
/// Runs on its own thread, off the safety path: failures only log.
/// Driven from the node's activate/deactivate transitions, so it runs only
/// while the node is ACTIVE. start() and stop() are idempotent.
class FleetCheckin
{
public:
  /// @param config Deployment values.
  /// @param machine_id Device ID of this machine.
  /// @param snapshot_fn Returns the state to report; called from the check-in thread.
  FleetCheckin(FleetCheckinConfig config, uint32_t machine_id, std::function<MachineSnapshot()> snapshot_fn);
  ~FleetCheckin();

  FleetCheckin(const FleetCheckin &) = delete;
  FleetCheckin & operator=(const FleetCheckin &) = delete;

  /// @brief Launches the check-in thread.
  /// @return False when disabled (empty base_url) or the key file is unreadable.
  /// Non-fatal: check-in is off the safety path and never fails activation.
  bool start();

  /// @brief Stops the thread and joins. Idempotent.
  void stop();

  /// True while the check-in thread is running.
  bool enabled() const
  {
    return enabled_.load();
  }

private:
  void run();
  /// @brief POSTs the payload and, on a 2xx, reads the response for OTA
  /// directives. A directive is logged only.
  bool post_once(const std::string & endpoint, const std::string & payload, const std::string & bearer_key);
  void handle_response(const std::string & body);

  FleetCheckinConfig config_;
  uint32_t machine_id_;
  std::function<MachineSnapshot()> snapshot_fn_;
  std::chrono::steady_clock::time_point start_time_{std::chrono::steady_clock::now()};

  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> enabled_{false};
};

}  // namespace protective_stop_machine
