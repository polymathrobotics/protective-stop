// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <thread>

#include "protective_stop_machine/backend.hpp"

namespace protective_stop_machine
{

/// @brief Deployment values for the check-in. Disabled when url is empty.
/// PSTOP_ANNOUNCE_URL and PSTOP_ANNOUNCE_KEY_FILE override url and key_file.
struct AnnounceConfig
{
  /// http://host[:port]/path. Empty disables the announcer.
  std::string url;
  /// Path whose first line is the bearer token, chmod 600.
  std::string key_file;
  /// Display name on the console. Empty uses this host's hostname.
  std::string name;
  /// Seconds between check-ins.
  int interval_s{60};
  /// HTTP request timeout, in seconds.
  double http_timeout_s{10.0};
};

/// @brief Escapes a string for embedding in a JSON string literal.
/// Control characters below 0x20 become \uXXXX.
inline std::string json_escape(const std::string & text)
{
  std::string escaped;
  escaped.reserve(text.size() + 8);
  for (char character : text) {
    switch (character) {
      case '"':
        escaped += "\\\"";
        break;
      case '\\':
        escaped += "\\\\";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(character) < 0x20) {
          char escape_sequence[8];
          std::snprintf(escape_sequence, sizeof(escape_sequence), "\\u%04x", character);
          escaped += escape_sequence;
        } else {
          escaped += character;
        }
    }
  }
  return escaped;
}

/// @brief Builds the check-in JSON body.
/// The core {"name","port"} pair is byte-for-byte what the host runner's
/// announce_post_once sends; the rest are additive fields:
///   device_type    "machine", distinguishing machines from remotes.
///   machine_id     this machine's 32-bit pstop id, 8 hex digits.
///   running        robot cleared to move (armed, no stop).
///   active_remotes + remotes[]  the per-remote bonded summary.
inline std::string build_announce_payload(
  const std::string & name, int port, uint32_t machine_id, const MachineSnapshot & snapshot)
{
  char machine_id_text[16];
  std::snprintf(machine_id_text, sizeof(machine_id_text), "%08x", machine_id);

  std::string payload = "{\"name\":\"";
  payload += json_escape(name);
  payload += "\",\"port\":";
  payload += std::to_string(port);
  payload += ",\"machine_id\":\"";
  payload += machine_id_text;
  payload += "\",\"device_type\":\"machine\",\"running\":";
  payload += snapshot.running ? "true" : "false";
  payload += ",\"active_remotes\":";
  payload += std::to_string(snapshot.active_remotes);
  payload += ",\"remotes\":[";
  bool first = true;
  for (const auto & remote : snapshot.remotes) {
    if (!first) {
      payload += ',';
    }
    first = false;
    payload += "{\"id\":\"";
    payload += json_escape(remote.device_id);
    payload += "\",\"bond_state\":";
    payload += std::to_string(static_cast<int>(remote.bond_state));
    payload += ",\"stop_only\":";
    payload += remote.stop_only ? "true" : "false";
    payload += ",\"in_use\":";
    payload += remote.in_use ? "true" : "false";
    payload += '}';
  }
  payload += "]}";
  return payload;
}

/// @brief Periodically POSTs a bearer-authenticated RUNNING check-in to a fleet console.
/// Runs on its own thread, off the safety path: failures only log.
/// Driven from the node's activate/deactivate transitions, so it runs only
/// while the node is ACTIVE. start() and stop() are idempotent.
class MachineAnnouncer
{
public:
  /// @param config Deployment values.
  /// @param port UDP port the machine listens on.
  /// @param machine_id Device ID of this machine.
  /// @param snapshot_fn Returns the state to report; called from the check-in thread.
  MachineAnnouncer(AnnounceConfig config, int port, uint32_t machine_id, std::function<MachineSnapshot()> snapshot_fn);
  ~MachineAnnouncer();

  MachineAnnouncer(const MachineAnnouncer &) = delete;
  MachineAnnouncer & operator=(const MachineAnnouncer &) = delete;

  /// @brief Launches the check-in thread.
  /// @return False when disabled (empty url) or the key file is unreadable.
  /// Non-fatal: announce is off the safety path and never fails activation.
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
  bool post_once(const std::string & payload, const std::string & bearer_key);

  AnnounceConfig config_;
  int port_;
  uint32_t machine_id_;
  std::function<MachineSnapshot()> snapshot_fn_;

  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> enabled_{false};
};

}  // namespace protective_stop_machine
