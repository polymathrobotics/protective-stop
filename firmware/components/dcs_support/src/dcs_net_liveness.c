// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dcs_net_liveness.c
 * @brief Layer-1 safety: ICMP-based liveness watchdog.
 *
 * The TWDT watches CPU/scheduling health — it does NOT detect a deadlocked
 * lwIP stack. We have observed (v14 silent hang) the scheduler still
 * ticking, app_hb still feeding TWDT, but lwIP wedged: no ping replies,
 * no HTTP, no ARP responses. This task uses esp_ping progress as the
 * network-liveness signal — if replies stop for NET_LIVENESS_TIMEOUT_MS
 * after connectivity was established, abort() deliberately (so reset_reason=
 * PANIC captures a backtrace) to REBOOT and recover. The reboot is marked
 * (dcs_safety_mark_liveness_abort) so crash accounting does NOT count this
 * network-class reboot toward the firmware-rollback ladder — a network/
 * upstream/peer condition must never downgrade the image; only a genuine code
 * fault should.
 *
 * W5500-board adaptation (2026-06-21):
 *   - Target is the DEFAULT NETIF'S GATEWAY, not 8.8.8.8. A PoE board on a
 *     wired LAN may have no internet route, so pinging Google would false-
 *     abort. The gateway is reachable whenever the active link is healthy.
 *   - The target is re-resolved as the active interface changes (the
 *     net supervisor may switch Ethernet<->USB<->WiFi at runtime).
 *   - The watchdog ARMS conservatively: the CURRENT uplink's gateway must have
 *     replied since the last failover (re-armed per interface, not sticky), AND
 *     pstop must have bonded at least once (ps_last>0) before an abort can fire.
 *     If we never reach the gateway, or pstop never bonded (gateway blocks ICMP,
 *     isolated LAN, SoftAP setup, down peer, no upstream), it stays DISARMED and
 *     never aborts — the board can sit in provisioning/degraded states
 *     indefinitely. It still catches the "was fully healthy, then lwIP wedged"
 *     failure that motivated this watchdog.
 */

#include <inttypes.h>
#include <stdatomic.h>
#include <stdlib.h>

#include "dcs_internal.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/ip_addr.h"
#include "ml_app.h"
#include "ping/ping_sock.h"

#define NET_LIVENESS_TIMEOUT_MS \
  180000 /* 180 s of silence (once armed) -> abort.
                                          * Deliberately ABOVE the boot-count clear
                                          * window (DCS_SAFETY_CLEAR_AFTER_MS=120s):
                                          * combined with the fact that liveness
                                          * aborts no longer count toward rollback,
                                          * a transient upstream outage can neither
                                          * ladder nor false-fire quickly. A real
                                          * lwIP wedge stays wedged far longer. */
#define NET_LIVENESS_POLL_MS 5000 /* re-check target + health every 5 s   */

/* 64-bit ms so the silence math stays correct past the ~49.7-day uint32 ms
 * wrap — this watchdog can reboot the chip, so its time base must not wrap. */
static atomic_uint_fast64_t s_last_ping_ok_ms = 0;
static atomic_uint_fast32_t s_ping_reply_count = 0;
static atomic_uint_fast32_t s_ping_loss_count = 0;
static atomic_uint_fast32_t s_gw_rtt_ms = 0; /* last gateway ping RTT (ms)   */
static atomic_uint_fast32_t s_gw_rtt_max_ms = 0; /* peak gateway RTT since boot  */
static atomic_int s_armed = 0; /* gateway replied on the CURRENT
                                                      * target since the last retarget.
                                                      * Reset on failover so a new
                                                      * interface must re-earn "armed"
                                                      * (a stale arm from the previous
                                                      * uplink must not fire here).   */
static atomic_uint_fast32_t s_gen = 0; /* ping-session generation: ignore
                                                      * late callbacks from a session
                                                      * superseded by a retarget.     */

/* Telemetry for /state.json: the active uplink's own gateway RTT — a device-side
 * latency figure that does NOT traverse the monitoring host's link. Sub-ms on
 * wired Ethernet (reports 0; esp_ping resolution is 1 ms), so a non-zero value
 * or a rising max/loss flags real uplink degradation. */
void dcs_net_liveness_stats(uint32_t * rtt_ms, uint32_t * rtt_max_ms, uint32_t * replies, uint32_t * losses)
{
  if (rtt_ms != NULL) {
    *rtt_ms = atomic_load(&s_gw_rtt_ms);
  }
  if (rtt_max_ms != NULL) {
    *rtt_max_ms = atomic_load(&s_gw_rtt_max_ms);
  }
  if (replies != NULL) {
    *replies = atomic_load(&s_ping_reply_count);
  }
  if (losses != NULL) {
    *losses = atomic_load(&s_ping_loss_count);
  }
}

/* ms since the active uplink's gateway last answered ICMP, or 0 if it has never
 * answered (disarmed — e.g. a gateway that blocks ICMP, or no upstream yet). The
 * supervisor uses this + a pstop-silence cross-check to demote a uplink whose
 * upstream has died but whose netif still holds a stale DHCP lease. */
uint32_t dcs_net_liveness_gw_silent_ms(void)
{
  if (atomic_load(&s_ping_reply_count) == 0u) {
    return 0u; /* never armed */
  }
  uint64_t last = atomic_load(&s_last_ping_ok_ms);
  uint64_t now = (uint64_t)esp_timer_get_time() / 1000u;
  return (now > last) ? (uint32_t)(now - last) : 0u;
}

static esp_ping_handle_t s_ping_handle = NULL;
static uint32_t s_target_ip = 0; /* current gateway target (network order) */

/* Reject callbacks from a superseded session: esp_ping_delete_session is async,
 * so the old gateway's ping task can fire one last reply/timeout after a
 * retarget. Each session carries its generation in cb_args; a mismatch means the
 * callback belongs to a dead target and must not touch s_armed / s_last_ping_ok_ms
 * (else a stale OLD-gateway reply could re-arm or disarm the NEW target). */
static inline bool ping_stale(void * args)
{
  return (uint32_t)(uintptr_t)args != atomic_load(&s_gen);
}

static void on_ping_success(esp_ping_handle_t hdl, void * args)
{
  if (ping_stale(args)) {
    return;
  }
  uint8_t ttl;
  uint16_t seqno;
  uint32_t elapsed_ms, recv_len;
  ip_addr_t target;
  (void)esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
  (void)esp_ping_get_profile(hdl, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
  (void)esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target, sizeof(target));
  (void)esp_ping_get_profile(hdl, ESP_PING_PROF_SIZE, &recv_len, sizeof(recv_len));
  (void)esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_ms, sizeof(elapsed_ms));
  ESP_LOGI(
    "GW_PING",
    "REPLY seq=%" PRIu16 " ttl=%u %s len=%" PRIu32 " time=%" PRIu32 "ms",
    seqno,
    ttl,
    ipaddr_ntoa(&target),
    recv_len,
    elapsed_ms);
  atomic_store(&s_last_ping_ok_ms, (uint64_t)esp_timer_get_time() / 1000u);
  (void)atomic_fetch_add(&s_ping_reply_count, 1);
  atomic_store(&s_armed, 1); /* the CURRENT uplink's gateway is reachable */
  /* single writer (the ping task) — plain stores are race-free here */
  atomic_store(&s_gw_rtt_ms, elapsed_ms);
  if (elapsed_ms > atomic_load(&s_gw_rtt_max_ms)) {
    atomic_store(&s_gw_rtt_max_ms, elapsed_ms);
  }
}

static void on_ping_timeout(esp_ping_handle_t hdl, void * args)
{
  if (ping_stale(args)) {
    return;
  }
  uint16_t seqno;
  ip_addr_t target;
  (void)esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
  (void)esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target, sizeof(target));
  ESP_LOGW("GW_PING", "TIMEOUT seq=%" PRIu16 " %s", seqno, ipaddr_ntoa(&target));
  (void)atomic_fetch_add(&s_ping_loss_count, 1);
}

/* Gateway IPv4 (network byte order) of the active PHYSICAL uplink, or 0.
 *
 * Deliberately NOT esp_netif_get_default_netif(): when Tailscale is active the
 * default netif flaps between the wired uplink (pinned by dcs_net_supervisor)
 * and the WireGuard overlay netif (whose "gateway" doesn't answer ICMP). This
 * watchdog used to follow the default netif, so under Tailscale it ended up
 * pinging the WG gateway, saw 60 s of silence, and abort()ed the chip — a
 * false reboot that the JTAG backtrace pinned to dcs_net_liveness.c. Pinging
 * the physical uplink's own gateway (lwIP routes it directly via that netif's
 * connected subnet, regardless of the default route) is the true link-liveness
 * signal and is immune to overlay routing churn. */
static uint32_t current_gateway(void)
{
  /* Only the STA-class uplinks have an upstream gateway worth probing; SoftAP
     * or none means no upstream, so the watchdog stays disarmed. */
  dcs_iface_t a = (dcs_iface_t)atomic_load(&g_dcs_active_iface);
  if ((a != DCS_IFACE_ETH) && (a != DCS_IFACE_USB) && (a != DCS_IFACE_WIFI)) {
    return 0u;
  }
  esp_netif_t * n = esp_netif_get_handle_from_ifkey(dcs_iface_ifkey(a));
  if (n == NULL) {
    return 0u;
  }
  esp_netif_ip_info_t ip;
  if (esp_netif_get_ip_info(n, &ip) != ESP_OK) {
    return 0u;
  }
  return ip.gw.addr;
}

/* (Re)point the ping session at `gw`. Tears down any previous session. */
static void retarget_ping(uint32_t gw)
{
  if (s_ping_handle != NULL) {
    (void)esp_ping_stop(s_ping_handle);
    (void)esp_ping_delete_session(s_ping_handle);
    s_ping_handle = NULL;
  }
  s_target_ip = gw;
  /* New interface (or no upstream): DISARM. The watchdog must re-earn "armed"
     * with a fresh reply on the new gateway before it can fire — a stale arm
     * from the previous uplink must never abort against a just-promoted link. */
  atomic_store(&s_armed, 0);
  if (gw == 0u) {
    return; /* no upstream — stay disarmed */
  }

  /* Give the new gateway a fresh silence window. Otherwise the abort timer
     * would judge a just-promoted uplink by the PREVIOUS target's staleness and
     * false-fire before the new gateway's first ping reply (a failover flap). */
  atomic_store(&s_last_ping_ok_ms, (uint64_t)esp_timer_get_time() / 1000u);

  /* Bump the generation BEFORE creating the new session so any late callback
     * from the previous one is recognised as stale (see ping_stale). */
  uint32_t gen = atomic_fetch_add(&s_gen, 1) + 1;

  ip_addr_t target = {0};
  ip_addr_set_ip4_u32_val(target, gw); /* portable across LWIP_IPV6 on/off */

  esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
  cfg.target_addr = target;
  cfg.count = 0; /* infinite */
  cfg.interval_ms = 5000;
  cfg.timeout_ms = 2000;
  /* IDF default (~2 KiB) overflowed at ~33 min uptime because the ping task
     * does sockets + our LOG callback. 4 KiB has headroom. */
  cfg.task_stack_size = 4096;
  esp_ping_callbacks_t cbs = {
    .on_ping_success = on_ping_success,
    .on_ping_timeout = on_ping_timeout,
    .on_ping_end = NULL,
    .cb_args = (void *)(uintptr_t)gen,
  };
  if ((esp_ping_new_session(&cfg, &cbs, &s_ping_handle) == ESP_OK) && (s_ping_handle != NULL)) {
    (void)esp_ping_start(s_ping_handle);
    ESP_LOGI("GW_PING", "session started -> %s every 5s", ipaddr_ntoa(&target));
  } else {
    ESP_LOGE("GW_PING", "esp_ping_new_session failed");
    s_ping_handle = NULL;
  }
}

static void net_liveness_task(void * arg)
{
  (void)arg;
  ESP_LOGI(
    "LIVENESS",
    "arm-on-both(gateway+pstop), timeout=%dms, "
    "recovery-reboot does NOT count toward rollback",
    NET_LIVENESS_TIMEOUT_MS);

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(NET_LIVENESS_POLL_MS));

    /* Follow the active interface's gateway. */
    uint32_t gw = current_gateway();
    if (gw != s_target_ip) {
      ESP_LOGI("LIVENESS", "gateway changed -> retargeting ping");
      retarget_ping(gw);
    }

    /* Disarmed conditions: no upstream gateway, no replies yet, or in
         * SoftAP setup mode. The watchdog only fires for a link that WAS
         * healthy and went silent. */
    if (s_target_ip == 0u) {
      continue;
    }
    if (atomic_load(&s_armed) == 0) {
      continue; /* current uplink not (re)armed yet */
    }
    if ((g_dcs.app != NULL) && ml_app_in_ap_fallback(g_dcs.app)) {
      continue;
    }

    uint64_t last_ok_ms = atomic_load(&s_last_ping_ok_ms);
    uint64_t now_ms = (uint64_t)esp_timer_get_time() / 1000u;
    uint64_t silence_ms = (now_ms >= last_ok_ms) ? (now_ms - last_ok_ms) : 0u;

    if (silence_ms > (uint64_t)NET_LIVENESS_TIMEOUT_MS) {
      /* Cross-check before the (drastic) self-reboot. A gateway not
             * answering ICMP is NOT proof of an lwIP wedge — gateways routinely
             * deprioritise/rate-limit ICMP, especially under load, and that was
             * false-firing this watchdog (JTAG-confirmed abort here) under
             * Tailscale traffic while HTTP and the pstop link were both fine.
             * pstop does continuous UDP send+recv every 100 ms, so a fresh pstop
             * reply is hard proof lwIP is alive. Only a TOTAL loss — gateway
             * silent AND pstop silent — is a genuine wedge worth rebooting for. */
      uint64_t ps_last = atomic_load(&g_dcs_pstop_last_reply_ms);
      /* pstop NEVER bonded -> the system never reached full operational
             * health, so a silent gateway is NOT evidence of an lwIP wedge (it's
             * an isolated/unconfigured uplink or a down peer). Stay our hand.
             * This closes the false-abort hole where a never-bonded pstop
             * (ps_last==0) removed the veto and let a healthy chip reboot. */
      if (ps_last == 0u) {
        ESP_LOGW(
          "LIVENESS",
          "gateway silent %llums but pstop never bonded — not a "
          "wedge, NOT aborting",
          (unsigned long long)silence_ms);
        continue;
      }
      bool pstop_alive = (now_ms >= ps_last) && ((now_ms - ps_last) < (uint64_t)NET_LIVENESS_TIMEOUT_MS);
      if (pstop_alive) {
        ESP_LOGW(
          "LIVENESS",
          "gateway silent %llums but pstop reply is fresh — lwIP "
          "alive, NOT aborting",
          (unsigned long long)silence_ms);
        continue;
      }
      /* Genuine wedge: gateway AND pstop both silent on an armed, leased
             * uplink that WAS fully healthy. Reboot to recover — but mark it so
             * the next boot does NOT count this network-class reboot toward the
             * firmware-rollback ladder (only a bad image should roll back). */
      ESP_LOGE(
        "LIVENESS",
        "no gateway-ping reply for %llums AND pstop silent — lwIP "
        "wedge, rebooting to recover (NOT a rollback trigger)",
        (unsigned long long)silence_ms);
      dcs_safety_mark_liveness_abort();
      vTaskDelay(pdMS_TO_TICKS(50));
      abort();
    }
  }
}

void dcs_net_liveness_start(void)
{
  /* Target is resolved lazily once a default netif has a gateway.
     * 4096 (was 2560): this task calls retarget_ping() -> esp_ping_new_session()
     * inline (socket + netconn setup) on a gateway change, plus ipaddr_ntoa +
     * ESP_LOG — too deep for 2560 when failover churns the gateway repeatedly. */
  /* PSRAM stack: liveness probe is non-safety; its only abort path writes an
     * RTC_NOINIT flag (not flash) before reboot, so a PSRAM stack is safe. */
  (void)dcs_task_spawn_psram(net_liveness_task, "net_live", 4096, NULL, 1, tskNO_AFFINITY);
}
