// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// MachineAnnouncer implementation — the network half. Mirrors the host runner's
// announce_thread/announce_post_once (host/machine_app_runner.c) but posts via
// libcurl (already linked for the hardware backend) instead of a raw socket, and
// feeds the software machine's live snapshot into the payload. Logs to stderr
// like the host runner; stays rclcpp-free so the payload logic is unit-testable.
#include "protective_stop_machine/announce.hpp"

#include <curl/curl.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <utility>

namespace protective_stop_machine
{

// Discard the response body; we only care about the HTTP status.
static size_t discard_cb(char * /*ptr*/, size_t size, size_t nmemb, void * /*userdata*/)
{
  return size * nmemb;
}

MachineAnnouncer::MachineAnnouncer(
  AnnounceConfig cfg, int port, uint32_t machine_id, std::function<MachineSnapshot()> snapshot_fn)
: cfg_(std::move(cfg))
, port_(port)
, machine_id_(machine_id)
, snapshot_fn_(std::move(snapshot_fn))
{}

MachineAnnouncer::~MachineAnnouncer()
{
  stop();
}

bool MachineAnnouncer::start()
{
  if (running_.load()) {
    return true;
  }
  if (cfg_.url.empty()) {
    return false;  // disabled — non-fatal, caller only logs
  }
  running_ = true;
  enabled_ = true;
  thread_ = std::thread([this] { run(); });
  return true;
}

void MachineAnnouncer::stop()
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
bool MachineAnnouncer::post_once(const std::string & payload, const std::string & bearer_key)
{
  CURL * curl = curl_easy_init();
  if (!curl) {
    return false;
  }
  struct curl_slist * headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  const std::string auth = "Authorization: Bearer " + bearer_key;
  headers = curl_slist_append(headers, auth.c_str());

  curl_easy_setopt(curl, CURLOPT_URL, cfg_.url.c_str());
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_COPYPOSTFIELDS, payload.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, discard_cb);
  // NOLINTNEXTLINE(runtime/int)
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(cfg_.http_timeout_s * 1000.0));

  CURLcode rc = curl_easy_perform(curl);
  // NOLINTNEXTLINE(runtime/int)
  long status = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  return rc == CURLE_OK && status >= 200 && status < 300;
}

void MachineAnnouncer::run()
{
  // Load the bearer token from the key file's first line (chmod 600), mirroring
  // the host runner. An empty/absent key file yields an empty bearer — same as
  // the host runner, which posts with whatever it read (possibly empty).
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
      std::fprintf(stderr, "announce: cannot read key file %s — announce disabled\n", cfg_.key_file.c_str());
      enabled_ = false;
      running_ = false;
      return;
    }
  }

  // Resolve the display name: configured value, else this host's hostname.
  std::string name = cfg_.name;
  if (name.empty()) {
    char host[128];
    if (gethostname(host, sizeof(host)) == 0) {
      host[sizeof(host) - 1] = '\0';
      name = host;
    } else {
      name = "pstop-machine";
    }
  }

  const int interval = cfg_.interval_s > 0 ? cfg_.interval_s : 60;
  int last_ok = -1;  // -1 unknown, 0 fail, 1 ok — log only on change, like host
  while (running_.load()) {
    const MachineSnapshot snap = snapshot_fn_ ? snapshot_fn_() : MachineSnapshot{};
    const std::string payload = build_announce_payload(name, port_, machine_id_, snap);
    const bool ok = post_once(payload, bearer_key);
    const int now_ok = ok ? 1 : 0;
    if (now_ok != last_ok) {
      if (ok) {
        std::fprintf(stderr, "announce: OK -> %s (as \"%s\")\n", cfg_.url.c_str(), name.c_str());
      } else {
        std::fprintf(stderr, "announce: FAILED -> %s (will keep retrying)\n", cfg_.url.c_str());
      }
      last_ok = now_ok;
    }
    for (int i = 0; i < interval && running_.load(); ++i) {
      sleep(1);
    }
  }
}

}  // namespace protective_stop_machine
