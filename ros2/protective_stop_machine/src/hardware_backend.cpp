// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// HardwareMachineBackend — HTTP client to the ESP32 machn. Polls /state.json for
// state and proxies runtime config to the device admin API. The ESP32 enforces
// STOP independently; this backend only observes and (best-effort) reconfigures.
#include "protective_stop_machine/hardware_backend.hpp"

#include <curl/curl.h>

#include <chrono>
#include <cstdio>
#include <mutex>
#include <string>
#include <utility>

#include "protective_stop_machine/json_lite.hpp"

namespace protective_stop_machine
{

static size_t append_response_body(char * ptr, size_t size, size_t nmemb, void * userdata)
{
  auto * out = static_cast<std::string *>(userdata);
  out->append(ptr, size * nmemb);
  return size * nmemb;
}

HardwareMachineBackend::HardwareMachineBackend(const HardwareConfig & config)
: config_(config)
{}

HardwareMachineBackend::~HardwareMachineBackend()
{
  stop();
}

bool HardwareMachineBackend::start()
{
  if (running_.load()) {
    return true;
  }
  // libcurl global init is process-wide and not thread-safe: do it once before
  // the poll thread starts. No matching cleanup() — as a component we may share
  // the process with other curl users, so we leak it until exit rather than tear
  // it down underneath them.
  static std::once_flag curl_once;
  std::call_once(curl_once, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
  running_ = true;
  poll_thread_ = std::thread([this] { poll_loop(); });
  return true;
}

void HardwareMachineBackend::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  if (poll_thread_.joinable()) {
    poll_thread_.join();
  }
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  // unreachable -> UNSTABLE
  latest_snapshot_ = MachineSnapshot{};
}

// NOLINTNEXTLINE(runtime/int)
bool HardwareMachineBackend::http_get(const std::string & path, std::string & body, long & status)
{
  CURL * curl = curl_easy_init();
  if (!curl) {
    return false;
  }
  const std::string url = config_.device_url + path;
  body.clear();
  status = 0;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append_response_body);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
  // NOLINTNEXTLINE(runtime/int)
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(config_.http_timeout_s * 1000.0));
  if (!config_.admin_pass.empty()) {
    curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
    const std::string user_password = config_.admin_user + ":" + config_.admin_pass;
    curl_easy_setopt(curl, CURLOPT_USERPWD, user_password.c_str());
  }
  CURLcode curl_result = curl_easy_perform(curl);
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_easy_cleanup(curl);
  return curl_result == CURLE_OK && status >= 200 && status < 300;
}

bool HardwareMachineBackend::http_post(
  // NOLINTNEXTLINE(runtime/int)
  const std::string & path,
  const std::string & json,
  std::string & body,
  // NOLINTNEXTLINE(runtime/int)
  long & status)
{
  CURL * curl = curl_easy_init();
  if (!curl) {
    return false;
  }
  const std::string url = config_.device_url + path;
  body.clear();
  status = 0;
  struct curl_slist * headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_COPYPOSTFIELDS, json.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append_response_body);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
  // NOLINTNEXTLINE(runtime/int)
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(config_.http_timeout_s * 1000.0));
  if (!config_.admin_pass.empty()) {
    curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
    const std::string user_password = config_.admin_user + ":" + config_.admin_pass;
    curl_easy_setopt(curl, CURLOPT_USERPWD, user_password.c_str());
  }
  CURLcode curl_result = curl_easy_perform(curl);
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  return curl_result == CURLE_OK && status >= 200 && status < 300;
}

void HardwareMachineBackend::parse_state(const std::string & body, MachineSnapshot & out_snapshot)
{
  jsonlite::Value root;
  if (jsonlite::parse(body, root) && root.is_obj()) {
    out_snapshot.reachable = true;
    const bool relay_stop = root.bool_at("relay_stop", true);
    out_snapshot.running = !relay_stop;
    out_snapshot.relay.applicable = true;
    out_snapshot.relay.run = !relay_stop;
    out_snapshot.relay.relay_stop = relay_stop;
    out_snapshot.relay.fault_a = root.bool_at("relay_fault_a", false);
    out_snapshot.relay.fault_b = root.bool_at("relay_fault_b", false);
    out_snapshot.relay.mismatch = static_cast<uint32_t>(root.num_at("pstop_mismatch", 0));
    out_snapshot.status_reason = out_snapshot.running ? "run (relay closed)" : "stop (relay open)";

    // The machn exposes its bonded remotes as the "bonded_remotes" array;
    // each item has a numeric id (format as hex), state, age_ms, rtt_ms and
    // wg_rtt_ms. active_remotes is the array length.
    const jsonlite::Value * list = root.find("bonded_remotes");
    if (list && list->is_arr()) {
      for (const auto & item : list->arr) {
        if (!item.is_obj()) {
          continue;
        }
        const uint32_t remote_id = static_cast<uint32_t>(item.num_at("id", 0));
        // an id of 0 is an empty/malformed slot
        if (remote_id == 0U) {
          continue;
        }
        RemoteInfo remote;
        char id_text[16];
        std::snprintf(id_text, sizeof(id_text), "%08x", remote_id);
        remote.device_id = id_text;
        remote.bond_state = static_cast<uint8_t>(item.num_at("state", 2));
        remote.reply_age_ms = static_cast<uint32_t>(item.num_at("age_ms", 0));
        remote.loop_rtt_ms = static_cast<uint32_t>(item.num_at("rtt_ms", 0));
        remote.disco_rtt_ms = static_cast<uint32_t>(item.num_at("wg_rtt_ms", 0));
        out_snapshot.remotes.push_back(std::move(remote));
      }
    }
    out_snapshot.active_remotes = static_cast<uint32_t>(out_snapshot.remotes.size());
    out_snapshot.need_stop = relay_stop && out_snapshot.active_remotes > 0;
  } else {
    out_snapshot.reachable = false;
    out_snapshot.status_reason = "state.json parse error";
  }
}

void HardwareMachineBackend::poll_loop()
{
  const double poll_hz = config_.poll_hz > 0.1 ? config_.poll_hz : 5.0;
  const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / poll_hz));
  while (running_.load()) {
    std::string body;
    // NOLINTNEXTLINE(runtime/int)
    long status = 0;
    MachineSnapshot polled_snapshot;
    if (http_get("/state.json", body, status)) {
      parse_state(body, polled_snapshot);
    } else {
      polled_snapshot.reachable = false;
      polled_snapshot.status_reason = "device unreachable (http " + std::to_string(status) + ")";
    }
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      latest_snapshot_ = std::move(polled_snapshot);
    }
    std::this_thread::sleep_for(period);
  }
}

MachineSnapshot HardwareMachineBackend::snapshot() const
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return latest_snapshot_;
}

bool HardwareMachineBackend::configure(const MachineTiming & timing, std::string & error)
{
  // Control proxy: forward the (already safety-validated) timing to the device
  // admin API. The ESP32 does not currently expose a runtime timing endpoint,
  // so this reports the device's response and fails cleanly if unsupported —
  // the seam is here for when the device gains the endpoint.
  char json[160];
  std::snprintf(
    json,
    sizeof(json),
    "{\"heartbeat_ms\":%llu,\"max_missed\":%u,\"min_stop_ms\":%llu}",
    // NOLINTNEXTLINE(runtime/int)
    static_cast<unsigned long long>(timing.heartbeat_ms),
    static_cast<unsigned>(timing.max_missed),
    // NOLINTNEXTLINE(runtime/int)
    static_cast<unsigned long long>(timing.min_stop_ms));
  std::string body;
  // NOLINTNEXTLINE(runtime/int)
  long status = 0;
  if (http_post("/admin/api/pstop_config", json, body, status)) {
    error.clear();
    return true;
  }
  error = "device did not accept timing config (http " + std::to_string(status) +
          "); hardware timing is set via device config";
  return false;
}

}  // namespace protective_stop_machine
