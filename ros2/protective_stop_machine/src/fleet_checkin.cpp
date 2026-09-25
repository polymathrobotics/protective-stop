// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// FleetCheckin implementation — the network half. Mirrors the ESP32
// fleet_ota_checkin() (components/microlink/src/ml_app.c) but posts via libcurl
// (already linked for the hardware backend) instead of esp_http_client, and
// derives the payload from the software machine's live snapshot. Logs to stderr
// like the announcer; stays rclcpp-free so the payload logic is unit-testable.
#include "protective_stop_machine/fleet_checkin.hpp"

#include <arpa/inet.h>
#include <curl/curl.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <utility>

#include "protective_stop_machine/json_lite.hpp"

namespace protective_stop_machine
{

// Accumulate the response body (capped) so we can read OTA directives. libcurl
// hands data in chunks; append until the cap, then drop the rest.
static size_t append_capped_response_body(char * ptr, size_t size, size_t nmemb, void * userdata)
{
  const size_t chunk_size = size * nmemb;
  auto * out = static_cast<std::string *>(userdata);
  // fleet check-in replies are small JSON
  constexpr size_t kMaxResp = 4096;
  if (out->size() < kMaxResp) {
    out->append(ptr, out->size() + chunk_size > kMaxResp ? kMaxResp - out->size() : chunk_size);
  }
  // always consume all, else libcurl aborts the transfer
  return chunk_size;
}

// First non-loopback IPv4 address of this host, dotted-quad, or "" if none.
// Mirrors the ESP32 reading its WIFI_STA IP: a best-effort local-IP hint for the
// fleet, never load-bearing.
static std::string resolve_local_ipv4()
{
  struct ifaddrs * if_list = nullptr;
  if (getifaddrs(&if_list) != 0) {
    return "";
  }
  std::string result;
  for (struct ifaddrs * iface = if_list; iface != nullptr; iface = iface->ifa_next) {
    if (iface->ifa_addr == nullptr || iface->ifa_addr->sa_family != AF_INET) {
      continue;
    }
    auto * ipv4 = reinterpret_cast<struct sockaddr_in *>(iface->ifa_addr);
    const uint32_t host_addr = ntohl(ipv4->sin_addr.s_addr);
    if ((host_addr >> 24) == 127) {
      // skip 127.0.0.0/8 loopback
      continue;
    }
    char address_text[INET_ADDRSTRLEN] = {0};
    if (inet_ntop(AF_INET, &ipv4->sin_addr, address_text, sizeof(address_text))) {
      result = address_text;
      break;
    }
  }
  freeifaddrs(if_list);
  return result;
}

FleetCheckin::FleetCheckin(
  FleetCheckinConfig config, uint32_t machine_id,
  std::function<MachineSnapshot()> snapshot_fn)
: config_(std::move(config))
  , machine_id_(machine_id)
  , snapshot_fn_(std::move(snapshot_fn))
{}

FleetCheckin::~FleetCheckin()
{
  stop();
}

bool FleetCheckin::start()
{
  if (running_.load()) {
    return true;
  }
  if (config_.base_url.empty()) {
    // disabled — non-fatal, the caller only logs
    return false;
  }
  running_ = true;
  enabled_ = true;
  thread_ = std::thread([this] {run();});
  return true;
}

void FleetCheckin::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  if (thread_.joinable()) {
    thread_.join();
  }
  enabled_ = false;
}

// NOLINTNEXTLINE(runtime/int) — libcurl uses `long` for status
bool FleetCheckin::post_once(
  const std::string & endpoint, const std::string & payload,
  const std::string & bearer_key)
{
  CURL * curl = curl_easy_init();
  if (!curl) {
    return false;
  }
  struct curl_slist * headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  const std::string auth = "Authorization: Bearer " + bearer_key;
  headers = curl_slist_append(headers, auth.c_str());

  std::string response_body;
  curl_easy_setopt(curl, CURLOPT_URL, endpoint.c_str());
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_COPYPOSTFIELDS, payload.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append_capped_response_body);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);
  // NOLINTNEXTLINE(runtime/int)
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(config_.http_timeout_s * 1000.0));

  CURLcode curl_result = curl_easy_perform(curl);
  // NOLINTNEXTLINE(runtime/int)
  long status = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  const bool posted = curl_result == CURLE_OK && status >= 200 && status < 300;
  if (posted) {
    handle_response(response_body);
  }
  return posted;
}

// Parse the 200 JSON for OTA directives. A software machine has NO ESP-OTA image,
// so we never download firmware: if the fleet says an update is available we log
// it (so an operator knows the fleet expects one), otherwise no-op. Never fails —
// a malformed/absent body is tolerated silently (off the safety path).
void FleetCheckin::handle_response(const std::string & body)
{
  if (body.empty()) {
    return;
  }
  jsonlite::Value root;
  if (!jsonlite::parse(body, root) || !root.is_obj()) {
    return;
  }
  if (root.bool_at("update_available", false)) {
    const std::string target = root.str_at("target_version", "?");
    std::fprintf(
      stderr,
      "fleet-checkin: fleet reports update_available (target %s) — software machine has no ESP-OTA, ignoring\n",
      target.c_str());
  }
}

void FleetCheckin::run()
{
  // Load the bearer token from the key file's first line (chmod 600), mirroring
  // the announcer. An empty/absent path yields an empty bearer; an unreadable
  // configured path disables the check-in (non-fatal — log only).
  std::string bearer_key;
  if (!config_.key_file.empty()) {
    FILE * file = std::fopen(config_.key_file.c_str(), "r");
    if (file) {
      char line[256];
      if (std::fgets(line, sizeof(line), file)) {
        line[std::strcspn(line, "\r\n")] = '\0';
        bearer_key = line;
      }
      std::fclose(file);
    } else {
      std::fprintf(stderr, "fleet-checkin: cannot read key file %s — check-in disabled\n",
          config_.key_file.c_str());
      enabled_ = false;
      running_ = false;
      return;
    }
  }

  const std::string endpoint = checkin_endpoint(config_.base_url);
  const std::string local_ip = resolve_local_ipv4();
  // tailscale_ip is intentionally empty (see build_checkin_payload note).
  const std::string tailscale_ip;

  const int interval = config_.interval_s > 0 ? config_.interval_s : 300;
  // -1 unknown, 0 fail, 1 ok — logged only on change
  int last_post_succeeded = -1;
  while (running_.load()) {
    const MachineSnapshot snapshot = snapshot_fn_ ? snapshot_fn_() : MachineSnapshot{};
    const auto uptime =
      std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() -
        start_time_).count();
    const std::string payload = build_checkin_payload(
      machine_id_,
      config_.app_version,
      config_.idf_version,
      static_cast<uint64_t>(uptime),
      interval,
      local_ip,
      tailscale_ip,
      snapshot);
    const bool posted = post_once(endpoint, payload, bearer_key);
    const int post_succeeded = posted ? 1 : 0;
    if (post_succeeded != last_post_succeeded) {
      if (posted) {
        std::fprintf(stderr, "fleet-checkin: OK -> %s\n", endpoint.c_str());
      } else {
        std::fprintf(stderr, "fleet-checkin: FAILED -> %s (will keep retrying)\n",
            endpoint.c_str());
      }
      last_post_succeeded = post_succeeded;
    }
    for (int second = 0; second < interval && running_.load(); ++second) {
      sleep(1);
    }
  }
}

}  // namespace protective_stop_machine
