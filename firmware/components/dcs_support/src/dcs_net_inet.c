// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dcs_net_inet.c
 * @brief Internet-reachability probe (display/telemetry only — never reboots).
 *
 * These units exist to relay pstop over Tailscale, and Tailscale's DERP relays
 * + WireGuard peers all live on the public internet. A device can hold a valid
 * local DHCP lease (link UP, gateway answering) yet have no route to the
 * internet — in which case Tailscale is dead even though every local light is
 * green. This probe is the operator-facing signal for exactly that case: the
 * status LED blinks red whenever the internet is unreachable (see dcs_rgb.c).
 *
 * Target = the two canonical anycast resolvers, 1.1.1.1 (Cloudflare) and
 * 8.8.8.8 (Google), both of which reliably answer ICMP echo. We ping ONE at a
 * time to keep a single esp_ping session resident (RAM is tight on this build);
 * after INET_SWITCH_AFTER_FAILS consecutive timeouts on the current target we
 * switch to the other. Internet is declared DOWN only when NEITHER target has
 * replied for INET_DOWN_AFTER_MS — so one resolver being down or ICMP-filtered
 * never false-alarms.
 *
 * This is DELIBERATELY decoupled from dcs_net_liveness (the lwIP-wedge watchdog
 * that pings the active uplink's GATEWAY and can abort() the chip). Internet
 * loss is a normal, recoverable condition on a robot whose uplink comes and
 * goes — it must light the LED, never reboot the device.
 */

#include <stdatomic.h>

#include "dcs_internal.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/ip_addr.h"
#include "ping/ping_sock.h"

static const char * TAG = "dcs_inet";

#define INET_PING_INTERVAL_MS 4000 /* echo request cadence on current target */
#define INET_PING_TIMEOUT_MS 2000 /* per-request reply window                */
#define INET_POLL_MS 2000 /* supervising-task tick                    */
#define INET_SWITCH_AFTER_FAILS 2u /* consecutive timeouts -> try other target */
#define INET_DOWN_AFTER_MS \
  20000u /* NEITHER target replied this long -> DOWN.
                                         * > switch-time (2 fails ~12 s + first ping
                                         * of the other target) so a single resolver
                                         * dropping never trips the alarm.          */

/* "1.1.1.1" and "8.8.8.8" are byte-palindromes, so host/network order is moot. */
static const char * const INET_TARGETS[2] = {"1.1.1.1", "8.8.8.8"};

static esp_ping_handle_t s_ping = NULL;
static int s_target_idx = 0; /* index into INET_TARGETS    */
static atomic_uint_fast64_t s_last_ok_ms = 0; /* last reply from ANY target  */
static atomic_uint_fast32_t s_fail_streak = 0; /* consecutive timeouts, cur tgt */
static atomic_int s_down = 0; /* published reachability state */
static atomic_uint_fast32_t s_gen = 0; /* session generation (see below) */

bool dcs_net_inet_down(void)
{
  return atomic_load(&s_down) != 0;
}

uint32_t dcs_net_inet_silent_ms(void)
{
  uint64_t last = atomic_load(&s_last_ok_ms);
  uint64_t now = (uint64_t)esp_timer_get_time() / 1000u;
  return (now > last) ? (uint32_t)(now - last) : 0u;
}

/* Ignore callbacks from a superseded session. esp_ping_stop/delete only clear
 * flags — the old ping task may be blocked in recvfrom() and fire one last
 * on_ok/on_timeout AFTER start_session() created the next session and reset the
 * counters. Each session carries its generation in cb_args; a stale callback's
 * generation no longer matches s_gen, so it must not touch the shared state (it
 * would credit/penalise the NEW target with the OLD target's result). */
static inline bool stale(void * args)
{
  return (uint32_t)(uintptr_t)args != atomic_load(&s_gen);
}

static void on_ok(esp_ping_handle_t hdl, void * args)
{
  (void)hdl;
  if (stale(args)) {
    return;
  }
  atomic_store(&s_last_ok_ms, (uint64_t)esp_timer_get_time() / 1000u);
  atomic_store(&s_fail_streak, 0);
}

static void on_timeout(esp_ping_handle_t hdl, void * args)
{
  (void)hdl;
  if (stale(args)) {
    return;
  }
  (void)atomic_fetch_add(&s_fail_streak, 1);
}

/* (Re)point the single ping session at INET_TARGETS[s_target_idx]. */
static void start_session(void)
{
  if (s_ping != NULL) {
    (void)esp_ping_stop(s_ping);
    (void)esp_ping_delete_session(s_ping);
    s_ping = NULL;
  }
  ip_addr_t target = {0};
  if (ipaddr_aton(INET_TARGETS[s_target_idx], &target) == 0) {
    return;
  }

  /* Bump the generation BEFORE creating the new session so any late callback
     * from the previous one is recognised as stale (see stale()). */
  uint32_t gen = atomic_fetch_add(&s_gen, 1) + 1;

  esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
  cfg.target_addr = target;
  cfg.count = 0; /* infinite */
  cfg.interval_ms = INET_PING_INTERVAL_MS;
  cfg.timeout_ms = INET_PING_TIMEOUT_MS;
  cfg.task_stack_size = 4096; /* sockets + our callback; IDF default ~2 KiB is tight */
  esp_ping_callbacks_t cbs = {
    .on_ping_success = on_ok,
    .on_ping_timeout = on_timeout,
    .on_ping_end = NULL,
    .cb_args = (void *)(uintptr_t)gen,
  };
  if ((esp_ping_new_session(&cfg, &cbs, &s_ping) == ESP_OK) && (s_ping != NULL)) {
    atomic_store(&s_fail_streak, 0);
    (void)esp_ping_start(s_ping);
    ESP_LOGI(TAG, "probing %s every %dms", INET_TARGETS[s_target_idx], INET_PING_INTERVAL_MS);
  } else {
    ESP_LOGE(TAG, "esp_ping_new_session failed for %s", INET_TARGETS[s_target_idx]);
    s_ping = NULL;
  }
}

static void inet_probe_task(void * arg)
{
  (void)arg;
  /* Grace period: assume reachable at boot so the LED isn't red for the first
     * INET_DOWN_AFTER_MS while the first echoes are still in flight. */
  atomic_store(&s_last_ok_ms, (uint64_t)esp_timer_get_time() / 1000u);
  start_session();

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(INET_POLL_MS));

    if (s_ping == NULL) {
      start_session();
      continue;
    }

    /* Current target gone quiet — switch to the other one. The healthy path
         * (1.1.1.1 replying) never switches, so the session stays put. */
    if (atomic_load(&s_fail_streak) >= INET_SWITCH_AFTER_FAILS) {
      s_target_idx = (s_target_idx == 0) ? 1 : 0;
      ESP_LOGW(TAG, "no reply from current target — switching to %s", INET_TARGETS[s_target_idx]);
      start_session();
    }

    bool down = dcs_net_inet_silent_ms() > INET_DOWN_AFTER_MS;
    if (down != (atomic_load(&s_down) != 0)) {
      ESP_LOGW(TAG, "internet %s", down ? "UNREACHABLE (LED -> red)" : "reachable");
      atomic_store(&s_down, down ? 1 : 0);
    }
  }
}

void dcs_net_inet_start(void)
{
  /* PSRAM stack: internet probe is non-safety, does no flash/NVS. */
  (void)dcs_task_spawn_psram(inet_probe_task, "net_inet", 2816, NULL, 1, tskNO_AFFINITY);
}
