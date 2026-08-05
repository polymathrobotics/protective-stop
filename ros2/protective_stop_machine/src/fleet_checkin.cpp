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
static size_t append_cb(char * ptr, size_t size, size_t nmemb, void * userdata)
{
  const size_t n = size * nmemb;
  auto * out = static_cast<std::string *>(userdata);
  constexpr size_t kMaxResp = 4096;  // fleet check-in replies are small JSON
  if (out->size() < kMaxResp) {
    out->append(ptr, out->size() + n > kMaxResp ? kMaxResp - out->size() : n);
  }
  return n;  // always consume all, else libcurl aborts the transfer
}

// First non-loopback IPv4 address of this host, dotted-quad, or "" if none.
// Mirrors the ESP32 reading its WIFI_STA IP: a best-effort local-IP hint for the
// fleet, never load-bearing.
static std::string resolve_local_ipv4()
{
  struct ifaddrs * ifaddr = nullptr;
  if (getifaddrs(&ifaddr) != 0) {
    return "";
  }
  std::string result;
  for (struct ifaddrs * ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) {
      continue;
    }
    auto * sin = reinterpret_cast<struct sockaddr_in *>(ifa->ifa_addr);
    const uint32_t host_addr = ntohl(sin->sin_addr.s_addr);
    if ((host_addr >> 24) == 127) {
      continue;  // skip 127.0.0.0/8 loopback
    }
    char buf[INET_ADDRSTRLEN] = {0};
    if (inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf))) {
      result = buf;
      break;
    }
  }
  freeifaddrs(ifaddr);
  return result;
}

FleetCheckin::FleetCheckin(
  FleetCheckinConfig cfg, uint32_t machine_id,
  std::function<MachineSnapshot()> snapshot_fn)
: cfg_(std::move(cfg))
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
  if (cfg_.base_url.empty()) {
    return false;  // disabled — non-fatal, caller only logs
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

  std::string resp;
  curl_easy_setopt(curl, CURLOPT_URL, endpoint.c_str());
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_COPYPOSTFIELDS, payload.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resp);
  // NOLINTNEXTLINE(runtime/int)
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(cfg_.http_timeout_s * 1000.0));

  CURLcode rc = curl_easy_perform(curl);
  // NOLINTNEXTLINE(runtime/int)
  long status = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  const bool ok = rc == CURLE_OK && status >= 200 && status < 300;
  if (ok) {
    handle_response(resp);
  }
  return ok;
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
  if (!cfg_.key_file.empty()) {
    FILE * file = std::fopen(cfg_.key_file.c_str(), "r");
    if (file) {
      char line[256];
      if (std::fgets(line, sizeof(line), file)) {
        line[std::strcspn(line, "\r\n")] = '\0';
        bearer_key = line;
      }
      std::fclose(file);
    } else {
      std::fprintf(stderr, "fleet-checkin: cannot read key file %s — check-in disabled\n",
          cfg_.key_file.c_str());
      enabled_ = false;
      running_ = false;
      return;
    }
  }

  const std::string endpoint = checkin_endpoint(cfg_.base_url);
  const std::string local_ip = resolve_local_ipv4();
  // tailscale_ip is intentionally empty (see build_checkin_payload note).
  const std::string tailscale_ip;

  const int interval = cfg_.interval_s > 0 ? cfg_.interval_s : 300;
  int last_ok = -1;  // -1 unknown, 0 fail, 1 ok — log only on change
  while (running_.load()) {
    const MachineSnapshot snap = snapshot_fn_ ? snapshot_fn_() : MachineSnapshot{};
    const auto uptime =
      std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() -
        start_time_).count();
    const std::string payload = build_checkin_payload(
      machine_id_,
      cfg_.app_version,
      cfg_.idf_version,
      static_cast<uint64_t>(uptime),
      interval,
      local_ip,
      tailscale_ip,
      snap);
    const bool ok = post_once(endpoint, payload, bearer_key);
    const int now_ok = ok ? 1 : 0;
    if (now_ok != last_ok) {
      if (ok) {
        std::fprintf(stderr, "fleet-checkin: OK -> %s\n", endpoint.c_str());
      } else {
        std::fprintf(stderr, "fleet-checkin: FAILED -> %s (will keep retrying)\n",
            endpoint.c_str());
      }
      last_ok = now_ok;
    }
    for (int i = 0; i < interval && running_.load(); ++i) {
      sleep(1);
    }
  }
}

}  // namespace protective_stop_machine
