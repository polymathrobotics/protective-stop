// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// HardwareMachineBackend — HTTP client to the ESP32 machn. Polls /state.json for
// state and proxies runtime config to the device admin API. The ESP32 enforces
// STOP independently; this backend only observes and (best-effort) reconfigures.
#include "protective_stop_machine/hardware_backend.hpp"

#include <chrono>

#include <curl/curl.h>

#include "protective_stop_machine/json_lite.hpp"

namespace protective_stop_machine
{

static size_t write_cb(char * ptr, size_t size, size_t nmemb, void * userdata)
{
  auto * out = static_cast<std::string *>(userdata);
  out->append(ptr, size * nmemb);
  return size * nmemb;
}

HardwareMachineBackend::HardwareMachineBackend(const HardwareConfig & cfg)
: cfg_(cfg)
{
  curl_global_init(CURL_GLOBAL_DEFAULT);
}

HardwareMachineBackend::~HardwareMachineBackend()
{
  stop();
  curl_global_cleanup();
}

bool HardwareMachineBackend::start()
{
  if (running_.load()) {return true;}
  running_ = true;
  th_ = std::thread([this] {poll_loop();});
  return true;
}

void HardwareMachineBackend::stop()
{
  if (!running_.exchange(false)) {return;}
  if (th_.joinable()) {th_.join();}
  std::lock_guard<std::mutex> lk(mtx_);
  snap_ = MachineSnapshot{};   // unreachable -> UNSTABLE
}

bool HardwareMachineBackend::http_get(const std::string & path, std::string & body,
  long & status)
{
  CURL * c = curl_easy_init();
  if (!c) {return false;}
  const std::string url = cfg_.device_url + path;
  body.clear();
  status = 0;
  curl_easy_setopt(c, CURLOPT_URL, url.c_str());
  curl_easy_setopt(c, CURLOPT_WRITEFUNCTION, write_cb);
  curl_easy_setopt(c, CURLOPT_WRITEDATA, &body);
  curl_easy_setopt(c, CURLOPT_TIMEOUT_MS,
    static_cast<long>(cfg_.http_timeout_s * 1000.0));
  if (!cfg_.admin_pass.empty()) {
    curl_easy_setopt(c, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
    const std::string up = cfg_.admin_user + ":" + cfg_.admin_pass;
    curl_easy_setopt(c, CURLOPT_USERPWD, up.c_str());
  }
  CURLcode rc = curl_easy_perform(c);
  curl_easy_getinfo(c, CURLINFO_RESPONSE_CODE, &status);
  curl_easy_cleanup(c);
  return rc == CURLE_OK && status >= 200 && status < 300;
}

bool HardwareMachineBackend::http_post(const std::string & path,
  const std::string & json, std::string & body, long & status)
{
  CURL * c = curl_easy_init();
  if (!c) {return false;}
  const std::string url = cfg_.device_url + path;
  body.clear();
  status = 0;
  struct curl_slist * hdrs = nullptr;
  hdrs = curl_slist_append(hdrs, "Content-Type: application/json");
  curl_easy_setopt(c, CURLOPT_URL, url.c_str());
  curl_easy_setopt(c, CURLOPT_POST, 1L);
  curl_easy_setopt(c, CURLOPT_COPYPOSTFIELDS, json.c_str());
  curl_easy_setopt(c, CURLOPT_HTTPHEADER, hdrs);
  curl_easy_setopt(c, CURLOPT_WRITEFUNCTION, write_cb);
  curl_easy_setopt(c, CURLOPT_WRITEDATA, &body);
  curl_easy_setopt(c, CURLOPT_TIMEOUT_MS,
    static_cast<long>(cfg_.http_timeout_s * 1000.0));
  if (!cfg_.admin_pass.empty()) {
    curl_easy_setopt(c, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
    const std::string up = cfg_.admin_user + ":" + cfg_.admin_pass;
    curl_easy_setopt(c, CURLOPT_USERPWD, up.c_str());
  }
  CURLcode rc = curl_easy_perform(c);
  curl_easy_getinfo(c, CURLINFO_RESPONSE_CODE, &status);
  curl_slist_free_all(hdrs);
  curl_easy_cleanup(c);
  return rc == CURLE_OK && status >= 200 && status < 300;
}

void HardwareMachineBackend::poll_loop()
{
  const double hz = cfg_.poll_hz > 0.1 ? cfg_.poll_hz : 5.0;
  const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / hz));
  while (running_.load()) {
    std::string body;
    long status = 0;
    MachineSnapshot s;
    if (http_get("/state.json", body, status)) {
      jsonlite::Value root;
      if (jsonlite::parse(body, root) && root.is_obj()) {
        s.reachable = true;
        const bool relay_stop = root.bool_at("relay_stop", true);
        s.running = !relay_stop;
        s.relay.applicable = true;
        s.relay.run = !relay_stop;
        s.relay.relay_stop = relay_stop;
        s.relay.fault_a = root.bool_at("relay_fault_a", false);
        s.relay.fault_b = root.bool_at("relay_fault_b", false);
        s.relay.mismatch = static_cast<uint32_t>(root.num_at("pstop_mismatch", 0));
        s.active_remotes =
          static_cast<uint32_t>(root.num_at("bonded_remotes", 0));
        s.need_stop = relay_stop && s.active_remotes > 0;
        s.status_reason = s.running ? "run (relay closed)" : "stop (relay open)";

        // Optional per-remote array if the device exposes one.
        for (const char * key : {"remotes", "bonded_remote_list"}) {
          const jsonlite::Value * list = root.find(key);
          if (list && list->is_arr()) {
            for (const auto & item : list->arr) {
              if (!item.is_obj()) {continue;}
              RemoteInfo r;
              r.device_id = item.str_at("id", item.str_at("device_id"));
              r.bond_state = static_cast<uint8_t>(item.num_at("state", 2));
              r.in_use = item.bool_at("in_use", false);
              r.reply_age_ms = static_cast<uint32_t>(item.num_at("reply_age_ms", 0));
              r.loop_rtt_ms = static_cast<uint32_t>(item.num_at("rtt_ms", 0));
              r.rebonds = static_cast<uint32_t>(item.num_at("rebonds", 0));
              s.remotes.push_back(std::move(r));
            }
            break;
          }
        }
      } else {
        s.reachable = false;
        s.status_reason = "state.json parse error";
      }
    } else {
      s.reachable = false;
      s.status_reason = "device unreachable (http " + std::to_string(status) + ")";
    }
    {
      std::lock_guard<std::mutex> lk(mtx_);
      snap_ = std::move(s);
    }
    std::this_thread::sleep_for(period);
  }
}

MachineSnapshot HardwareMachineBackend::snapshot() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return snap_;
}

bool HardwareMachineBackend::configure(const MachineTiming & timing, std::string & error)
{
  // Control proxy: forward the (already safety-validated) timing to the device
  // admin API. The ESP32 does not currently expose a runtime timing endpoint,
  // so this reports the device's response and fails cleanly if unsupported —
  // the seam is here for when the device gains the endpoint.
  char json[160];
  std::snprintf(json, sizeof(json),
    "{\"heartbeat_ms\":%llu,\"max_missed\":%u,\"min_stop_ms\":%llu}",
    static_cast<unsigned long long>(timing.heartbeat_ms),
    static_cast<unsigned>(timing.max_missed),
    static_cast<unsigned long long>(timing.min_stop_ms));
  std::string body;
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
