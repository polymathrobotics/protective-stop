// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dcs_admin_pages.c
 * @brief HTTP handlers for the admin/diagnostic web UI.
 *
 * Routes registered:
 *   GET  /                      Static HTML (k_index_html)
 *   GET  /state.json            Telemetry JSON
 *   POST /api/derp              Toggle DERP TX worker (microlink_pause_derp)
 *   POST /api/wg                Toggle ml_wg_mgr task suspend/resume
 *   POST /api/derp_delay?ms=N   Set DERP loop yield
 *   POST /api/wifi_tx_power?q=N Set WiFi max TX power
 *   POST /api/usb_enable        Flip NVS dcs_app/usb_en flag and reboot
 *   POST /api/ts_boot           Flip NVS dcs_app/ts_boot flag (takes effect next reboot)
 *   POST /api/pstop_peer?ip&port  Set + persist pstop peer target
 *   GET  /api/last_log          Last boot's tail-of-log (from RTC_NOINIT)
 *
 * All handlers are pure HTTP plumbing — telemetry values come from the
 * dcs_internal.h atomics that main.c populates via dcs_publish_*.
 */

#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dcs_admin_html.h"
#include "dcs_internal.h"
#include "dcs_support.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "microlink.h"
#include "ml_app.h"
#include "panic_log.h"
#include "soc/rtc_cntl_reg.h"
#include "wireguardif.h"

static const char * TAG = "dcs_admin";

/* === GET / =============================================================== */
static esp_err_t page_index(httpd_req_t * req)
{
  (void)httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, k_index_html, HTTPD_RESP_USE_STRLEN);
}

/* === GET /state.json ===================================================== */

static int emit_bucket(char * buf, int cap, const dcs_task_bucket_t * b)
{
  int len = snprintf(buf, cap, "[");
  if ((len < 0) || (len >= cap)) {
    return len;
  }
  for (int i = 0; (i < (int)b->n) && (len < cap); i++) {
    const char * sep = (i == 0) ? "" : ",";
    len += snprintf(buf + len, cap - len, "%s[\"%s\",%lu]", sep, b->tasks[i].name, (unsigned long)b->tasks[i].pct);
  }
  if (len < cap) {
    len += snprintf(buf + len, cap - len, "]");
  }
  return len;
}

/* IPv4 of a netif (by if_key) as a raw (network-order) u32 for /state.json,
 * 0 if the netif is absent or has no address. Lets callers see which
 * interfaces are up even when they don't currently own the default route. */
static uint32_t netif_ip_by_key(const char * key)
{
  esp_netif_t * n = esp_netif_get_handle_from_ifkey(key);
  if (!n) return 0;
  esp_netif_ip_info_t ip;
  if (esp_netif_get_ip_info(n, &ip) != ESP_OK) return 0;
  return ip.ip.addr;
}

static esp_err_t page_state(httpd_req_t * req)
{
  dcs_task_breakdown_t snap;
  dcs_telemetry_snapshot(&snap);

  /* microlink_get_rssi_dbm() calls esp_wifi_sta_get_ap_info(), which (like
     * esp_wifi_get_max_tx_power below) NULL-derefs inside current_task_is_wifi_task
     * when the WiFi driver was never esp_wifi_init'd (wired-Ethernet path skips it
     * for RAM). THIS is the call that crash-looped the device on a /state.json
     * poll (HIL coredump: httpd in current_task_is_wifi_task). Guard with the
     * WiFi-up flag; rssi reports 0 on the eth/idle path. */
  int rssi_dbm = dcs_wifi_is_enabled() ? microlink_get_rssi_dbm() : 0;
  int8_t tx_q = 0;
  /* esp_wifi_get_max_tx_power() NULL-derefs inside current_task_is_wifi_task
     * when the WiFi driver was never esp_wifi_init'd — and the wired-Ethernet
     * path deliberately skips esp_wifi_init to save RAM. A /state.json poll on
     * the pure-eth path therefore crashed the httpd task (PANIC), and the admin
     * panel auto-refreshing /state.json turned that into a crash/boot loop
     * (HIL-confirmed via coredump: PC=current_task_is_wifi_task, excvaddr=0xa4).
     * Only query tx power when WiFi is actually up; on eth/idle tx_q stays 0. */
  if (dcs_wifi_is_enabled()) {
    (void)esp_wifi_get_max_tx_power(&tx_q);
  }
  uint32_t free_heap = 0, largest = 0;
  (void)microlink_get_heap_info(&free_heap, &largest);

  int ml_state = -1;
  uint32_t vpn_ip = 0;
  if (g_dcs.ml_handle != NULL) {
    ml_state = (int)microlink_get_state(g_dcs.ml_handle);
    vpn_ip = microlink_get_vpn_ip(g_dcs.ml_handle);
  }

  uint32_t heap_min_internal = atomic_load(&g_dcs_heap_min_internal);
  uint32_t pbuf_fails = wireguardif_pbuf_alloc_fails;

  extern uint32_t ml_wg_mgr_get_dedup_drops(void);
  uint32_t wg_dedup_drops = ml_wg_mgr_get_dedup_drops();

  int wifi_disc = 0, wifi_conn = 0, wifi_idx = -1, wifi_n = 0;
  dcs_wifi_status(&wifi_disc, &wifi_conn, &wifi_idx, &wifi_n);

  /* device-side gateway-ping RTT (active uplink, doesn't traverse the host) */
  uint32_t gw_rtt = 0, gw_rtt_max = 0, gw_ok = 0, gw_loss = 0;
  dcs_net_liveness_stats(&gw_rtt, &gw_rtt_max, &gw_ok, &gw_loss);

  /* crash diagnostics: reset-reason history (newest last) + running OTA state */
  uint8_t rhist[DCS_RST_HIST_LEN];
  int rn = dcs_nvs_read_reset_history(rhist, DCS_RST_HIST_LEN);
  /* Worst case: 16 entries x "255," = 4 chars each + brackets/NUL. Sized
     * for corrupt-NVS values (uint8 max), and rp is clamped after every
     * append — otherwise a would-have-written snprintf return can push rp
     * past the buffer and `sizeof - rp` underflows as size_t (OOB write). */
  char rsth[(4 * DCS_RST_HIST_LEN) + 4];
  int rp = snprintf(rsth, sizeof(rsth), "[");
  for (int i = 0; i < rn; i++) {
    rp += snprintf(rsth + rp, sizeof(rsth) - (size_t)rp, "%s%u", (i != 0) ? "," : "", rhist[i]);
    if (rp > (int)(sizeof(rsth) - 1U)) {
      rp = (int)(sizeof(rsth) - 1U);
    }
  }
  (void)snprintf(rsth + rp, sizeof(rsth) - (size_t)rp, "]");
  esp_ota_img_states_t ota_st = ESP_OTA_IMG_UNDEFINED;
  (void)esp_ota_get_state_partition(esp_ota_get_running_partition(), &ota_st);
  int pstop_num = dcs_nvs_read_pstop_unit_num(); /* 0 = auto (chip-ID derived) */

  /* Build into a PSRAM buffer (off the httpd task stack, and off the
     * internal heap which runs tight on this build). */
  enum
  {
    JSON_CAP = 2048
  };

  char * buf = heap_caps_malloc(JSON_CAP, MALLOC_CAP_SPIRAM);
  if (!buf) buf = malloc(JSON_CAP); /* fall back to internal if no PSRAM */
  if (!buf) {
    (void)httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"oom\"}");
  }
  const int cap = JSON_CAP;
  int n = snprintf(
    buf,
    cap,
    "{\"t0\":%lu,\"t1\":%lu,\"l0\":%d,\"l1\":%d,"
    "\"load0\":%lu,\"load1\":%lu,"
    "\"e_hi0\":%lu,\"e_lo0\":%lu,\"e_hi1\":%lu,\"e_lo1\":%lu,"
    "\"active_iface\":%d,\"eth_link\":%d,"
    "\"eth_en\":%d,\"wifi_en\":%d,\"usbncm_en\":%d,"
    "\"wifi_disc\":%d,\"wifi_conn\":%d,\"wifi_idx\":%d,\"wifi_n\":%d,"
    "\"eth_ip\":%lu,\"usb_ip\":%lu,\"wifi_ip\":%lu,"
    "\"rgb_cycles\":%lu,\"uptime_ms\":%llu,"
    "\"derp_paused\":%d,\"derp_delay_ms\":%d,\"wg_paused\":%d,"
    "\"usb_enabled\":%d,\"ts_boot_en\":%d,\"derp_only\":%d,"
    "\"boot_count\":%u,\"reset_reason\":%u,"
    "\"ml_state\":%d,\"vpn_ip\":%lu,"
    "\"pstop_peer_ip\":%lu,\"pstop_peer_port\":%lu,"
    "\"pstop_sent\":%lu,\"pstop_replies\":%lu,\"pstop_last_msg\":%lu,\"pstop_mismatch\":%lu,"
    "\"pstop_send_fail\":%lu,\"pstop_rebonds\":%lu,"
    "\"pstop_rtt_ms\":%lu,\"pstop_last_reply_ms\":%llu,"
    "\"rssi\":%d,\"tx_q\":%d,\"free_heap\":%lu,\"largest\":%lu,"
    "\"heap_min_int\":%lu,\"wg_pbuf_fails\":%lu,\"wg_dedup_drops\":%lu,"
    "\"gw_rtt_ms\":%lu,\"gw_rtt_max_ms\":%lu,\"gw_ok\":%lu,\"gw_loss\":%lu,"
    "\"inet_down\":%d,\"inet_silent_ms\":%lu,"
    "\"rst_hist\":%s,\"ota_state\":%d,\"pstop_num\":%d,\"tk0\":",
    (unsigned long)atomic_load(&g_dcs_core_tick[0]),
    (unsigned long)atomic_load(&g_dcs_core_tick[1]),
    (int)atomic_load(&g_dcs_core_verdict[0]),
    (int)atomic_load(&g_dcs_core_verdict[1]),
    (unsigned long)atomic_load(&g_dcs_load_pct[0]),
    (unsigned long)atomic_load(&g_dcs_load_pct[1]),
    (unsigned long)atomic_load(&g_dcs_estop_high_ok[0]),
    (unsigned long)atomic_load(&g_dcs_estop_low_ok[0]),
    (unsigned long)atomic_load(&g_dcs_estop_high_ok[1]),
    (unsigned long)atomic_load(&g_dcs_estop_low_ok[1]),
    (int)atomic_load(&g_dcs_active_iface),
    dcs_eth_link_up() ? 1 : 0,
    dcs_eth_is_enabled() ? 1 : 0,
    dcs_wifi_is_enabled() ? 1 : 0,
    dcs_usb_is_enabled() ? 1 : 0,
    wifi_disc,
    wifi_conn,
    wifi_idx,
    wifi_n,
    (unsigned long)netif_ip_by_key("DCS_ETH"),
    (unsigned long)netif_ip_by_key("USB_NCM"),
    (unsigned long)netif_ip_by_key("WIFI_STA_DEF"),
    (unsigned long)atomic_load(&g_dcs_rgb_cycles),
    (unsigned long long)esp_timer_get_time() / 1000ULL,
    microlink_is_derp_paused() ? 1 : 0,
    microlink_get_derp_loop_delay_ms(),
    atomic_load(&g_dcs_wg_paused),
    g_dcs.usb_enabled ? 1 : 0,
    g_dcs.ts_boot_en ? 1 : 0,
    g_dcs.derp_only_mode ? 1 : 0,
    (unsigned int)g_dcs.boot_count,
    (unsigned int)g_dcs.reset_reason,
    ml_state,
    (unsigned long)vpn_ip,
    (unsigned long)atomic_load(&g_dcs_pstop_peer_ip),
    (unsigned long)atomic_load(&g_dcs_pstop_peer_port),
    (unsigned long)atomic_load(&g_dcs_pstop_sent),
    (unsigned long)atomic_load(&g_dcs_pstop_replies),
    (unsigned long)atomic_load(&g_dcs_pstop_last_msg),
    (unsigned long)atomic_load(&g_dcs_pstop_mismatch),
    (unsigned long)atomic_load(&g_dcs_pstop_send_fail),
    (unsigned long)atomic_load(&g_dcs_pstop_rebonds),
    (unsigned long)atomic_load(&g_dcs_pstop_rtt_ms),
    (unsigned long long)atomic_load(&g_dcs_pstop_last_reply_ms),
    rssi_dbm,
    (int)tx_q,
    (unsigned long)free_heap,
    (unsigned long)largest,
    (unsigned long)heap_min_internal,
    (unsigned long)pbuf_fails,
    (unsigned long)wg_dedup_drops,
    (unsigned long)gw_rtt,
    (unsigned long)gw_rtt_max,
    (unsigned long)gw_ok,
    (unsigned long)gw_loss,
    dcs_net_inet_down() ? 1 : 0,
    (unsigned long)dcs_net_inet_silent_ms(),
    rsth,
    (int)ota_st,
    pstop_num);

/* Clamp the running offset after EVERY append so `buf + n` and `cap - n`
     * can never run past the buffer. snprintf() returns the length it WOULD
     * have written, so n can exceed cap on a large task list — leaving
     * `cap - n` to underflow as size_t and the next append to scribble past
     * the buffer. That OOB write was corrupting memory and panicking the chip
     * whenever /state.json grew large enough (boot-loop -> OTA rollback). */
#define CLAMP_N()               \
  do {                          \
    if (n < 0) {                \
      n = 0;                    \
    } else if (n > (cap - 1)) { \
      n = cap - 1;              \
    }                           \
  } while (0)
  CLAMP_N();
  n += emit_bucket(buf + n, cap - n, &snap.b[0]);
  CLAMP_N();
  n += snprintf(buf + n, cap - n, ",\"oth0\":%lu,\"tk1\":", (unsigned long)snap.b[0].other_pct);
  CLAMP_N();
  n += emit_bucket(buf + n, cap - n, &snap.b[1]);
  CLAMP_N();
  n += snprintf(buf + n, cap - n, ",\"oth1\":%lu,\"tks\":", (unsigned long)snap.b[1].other_pct);
  CLAMP_N();
  n += emit_bucket(buf + n, cap - n, &snap.b[2]);
  CLAMP_N();
  n += snprintf(buf + n, cap - n, ",\"oths\":%lu}", (unsigned long)snap.b[2].other_pct);
  CLAMP_N();
#undef CLAMP_N

  (void)httpd_resp_set_type(req, "application/json");
  (void)httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t send_rc = httpd_resp_send(req, buf, n);
  free(buf);
  return send_rc;
}

/* === POST /api/derp ====================================================== */
static esp_err_t api_derp_toggle(httpd_req_t * req)
{
  bool now_paused = !microlink_is_derp_paused();
  microlink_pause_derp(now_paused);
  char buf[48];
  int n = snprintf(buf, sizeof(buf), "{\"ok\":true,\"paused\":%d}", now_paused ? 1 : 0);
  (void)httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, buf, n);
}

/* === POST /api/wg ======================================================== */
static esp_err_t api_wg_toggle(httpd_req_t * req)
{
  if (!g_dcs_wg_handle) {
    (void)httpd_resp_set_status(req, "503 Service Unavailable");
    (void)httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"no ml_wg_mgr handle\"}");
  }
  int paused = atomic_load(&g_dcs_wg_paused);
  if (paused != 0) {
    vTaskResume(g_dcs_wg_handle);
    atomic_store(&g_dcs_wg_paused, 0);
  } else {
    vTaskSuspend(g_dcs_wg_handle);
    atomic_store(&g_dcs_wg_paused, 1);
  }
  char buf[48];
  int n = snprintf(buf, sizeof(buf), "{\"ok\":true,\"paused\":%d}", atomic_load(&g_dcs_wg_paused));
  (void)httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, buf, n);
}

/* === POST /api/wifi_tx_power?q=N ========================================= */
static esp_err_t api_wifi_tx_power(httpd_req_t * req)
{
  char query[64];
  char val[8];
  if (
    (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) ||
    (httpd_query_key_value(query, "q", val, sizeof(val)) != ESP_OK))
  {
    (void)httpd_resp_set_status(req, "400 Bad Request");
    (void)httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing ?q=N\"}");
  }
  int q = (int)strtol(val, NULL, 10);
  if ((q < 8) || (q > 84)) {
    (void)httpd_resp_set_status(req, "400 Bad Request");
    (void)httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"q must be 8..84 (0.25 dBm units)\"}");
  }
  /* Same WiFi-not-init NULL-deref guard as /state.json: never call esp_wifi_*
     * unless the driver is up (wired/idle path skips esp_wifi_init). */
  if (!dcs_wifi_is_enabled()) {
    (void)httpd_resp_set_status(req, "409 Conflict");
    (void)httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"WiFi not active (wired/idle) — tx power unavailable\"}");
  }
  esp_err_t err = esp_wifi_set_max_tx_power((int8_t)q);
  if (err != ESP_OK) {
    (void)httpd_resp_set_status(req, "500 Internal Server Error");
    (void)httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"esp_wifi_set_max_tx_power failed\"}");
  }
  int8_t now_q = 0;
  (void)esp_wifi_get_max_tx_power(&now_q);
  char buf[48];
  int n = snprintf(buf, sizeof(buf), "{\"ok\":true,\"q\":%d}", now_q);
  (void)httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, buf, n);
}

/* === POST /api/derp_delay?ms=N =========================================== */
static esp_err_t api_derp_delay(httpd_req_t * req)
{
  char query[64];
  char val[8];
  if (
    (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) ||
    (httpd_query_key_value(query, "ms", val, sizeof(val)) != ESP_OK))
  {
    (void)httpd_resp_set_status(req, "400 Bad Request");
    (void)httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing ?ms=N\"}");
  }
  int ms = (int)strtol(val, NULL, 10);
  if (microlink_set_derp_loop_delay_ms(ms) != ESP_OK) {
    (void)httpd_resp_set_status(req, "400 Bad Request");
    (void)httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"ms must be 1..100\"}");
  }
  char buf[48];
  int n = snprintf(buf, sizeof(buf), "{\"ok\":true,\"ms\":%d}", microlink_get_derp_loop_delay_ms());
  (void)httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, buf, n);
}

/* === POST /api/pstop_peer?ip=A.B.C.D&port=N ============================== */
static esp_err_t api_pstop_peer(httpd_req_t * req)
{
  char query[96];
  char ipstr[20];
  char portstr[8];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
    (void)httpd_resp_set_status(req, "400 Bad Request");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing ?ip=A.B.C.D&port=N\"}");
  }
  if (
    (httpd_query_key_value(query, "ip", ipstr, sizeof(ipstr)) != ESP_OK) ||
    (httpd_query_key_value(query, "port", portstr, sizeof(portstr)) != ESP_OK))
  {
    (void)httpd_resp_set_status(req, "400 Bad Request");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"need both ip and port\"}");
  }
  unsigned int a, b, c, d;
  if ((sscanf(ipstr, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) || (a > 255U) || (b > 255U) || (c > 255U) || (d > 255U)) {
    (void)httpd_resp_set_status(req, "400 Bad Request");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"bad ip\"}");
  }
  int port = (int)strtol(portstr, NULL, 10);
  if ((port < 1) || (port > 65535)) {
    (void)httpd_resp_set_status(req, "400 Bad Request");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"bad port\"}");
  }
  uint32_t ip = ((uint32_t)a << 24) | ((uint32_t)b << 16) | ((uint32_t)c << 8) | (uint32_t)d;
  dcs_publish_pstop_peer(ip, (uint16_t)port); /* also persists to NVS */

  char resp[96];
  int n = snprintf(resp, sizeof(resp), "{\"ok\":true,\"ip\":\"%u.%u.%u.%u\",\"port\":%d}", a, b, c, d, port);
  (void)httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, resp, n);
}

/* === POST /api/ts_boot =================================================== */
static esp_err_t api_ts_boot_toggle(httpd_req_t * req)
{
  bool new_state = !g_dcs.ts_boot_en;
  esp_err_t r = dcs_nvs_write_ts_boot_en(new_state);
  (void)httpd_resp_set_type(req, "application/json");
  if (r != ESP_OK) {
    (void)httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"NVS write failed\"}");
  }
  g_dcs.ts_boot_en = new_state;
  char buf[96];
  int n = snprintf(
    buf, sizeof(buf), "{\"ok\":true,\"ts_boot_en\":%d,\"message\":\"takes effect on next reboot\"}", new_state ? 1 : 0);
  return httpd_resp_send(req, buf, n);
}

/* === POST /api/usb_enable ================================================ */
static esp_err_t api_usb_enable_toggle(httpd_req_t * req)
{
  bool new_state = !g_dcs.usb_enabled;
  esp_err_t r = dcs_nvs_write_usb_enabled(new_state);
  (void)httpd_resp_set_type(req, "application/json");
  if (r != ESP_OK) {
    (void)httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"NVS write failed\"}");
  }
  char buf[96];
  int n = snprintf(
    buf, sizeof(buf), "{\"ok\":true,\"usb_enabled\":%d,\"message\":\"rebooting in 500 ms\"}", new_state ? 1 : 0);
  (void)httpd_resp_send(req, buf, n);

  /* v15.27: pre-restart quiesce. Pause DERP and suspend the WG manager
     * BEFORE the 500 ms delay so no Tailscale packet processing is in
     * flight when esp_restart() fires. Mitigates a rare race observed
     * once on the WiFi→USB transition where the chip panic-rebooted
     * during the 500 ms window (backtrace lost — no serial cable on
     * the test rig). Same applies to admin restart and the safety-
     * ladder auto-restart path via the shutdown hook registered in
     * dcs_support_init(). */
  microlink_pause_derp(true);
  if (g_dcs_wg_handle != NULL) {
    vTaskSuspend(g_dcs_wg_handle);
    atomic_store(&g_dcs_wg_paused, 1);
  }
  vTaskDelay(pdMS_TO_TICKS(500));
  esp_restart();
  return ESP_OK;
}

/* === GET /api/last_log =================================================== */
static esp_err_t api_last_log(httpd_req_t * req)
{
  (void)httpd_resp_set_type(req, "text/plain");
  if (!panic_log_has_snapshot()) {
    return httpd_resp_sendstr(req, "");
  }
  size_t cap = 4096;
  char * buf = malloc(cap);
  if (!buf) {
    (void)httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "malloc failed");
  }
  size_t n = panic_log_copy_snapshot(buf, cap);
  esp_err_t r = httpd_resp_send(req, buf, n);
  free(buf);
  return r;
}

/* === POST /api/iface/{eth,wifi,usb} — toggle an interface up/down ========= *
 * Live control for testing failover and (for WiFi) on-demand bring-up. Each
 * flips the interface's driver; dcs_net_supervisor then re-picks the highest-
 * priority usable netif on its next 1 Hz tick. */
static esp_err_t iface_toggle_reply(httpd_req_t * req, esp_err_t r, bool now_on)
{
  char buf[80];
  int n = snprintf(
    buf,
    sizeof(buf),
    "{\"ok\":%s,\"enabled\":%d,\"err\":\"%s\"}",
    (r == ESP_OK) ? "true" : "false",
    now_on ? 1 : 0,
    esp_err_to_name(r));
  (void)httpd_resp_set_type(req, "application/json");
  if (r != ESP_OK) {
    (void)httpd_resp_set_status(req, "500 Internal Server Error");
  }
  return httpd_resp_send(req, buf, n);
}

static esp_err_t api_iface_eth(httpd_req_t * req)
{
  bool on = !dcs_eth_is_enabled();
  esp_err_t r = dcs_eth_set_enabled(on);
  return iface_toggle_reply(req, r, (r == ESP_OK) ? on : dcs_eth_is_enabled());
}

static esp_err_t api_iface_wifi(httpd_req_t * req)
{
  bool on = !dcs_wifi_is_enabled();
  esp_err_t r = dcs_wifi_set_enabled(on);
  return iface_toggle_reply(req, r, (r == ESP_OK) ? on : dcs_wifi_is_enabled());
}

static esp_err_t api_iface_usb(httpd_req_t * req)
{
  bool on = !dcs_usb_is_enabled();
  esp_err_t r = dcs_usb_set_enabled(on);
  /* Report the ACTUAL resulting state on failure, matching the eth/wifi
     * twins — the reply must not claim a state change that didn't happen. */
  return iface_toggle_reply(req, r, (r == ESP_OK) ? on : dcs_usb_is_enabled());
}

/* === POST /api/pstop_num?n=N — set the USB "PSTOPxx" unit number (0=auto). === *
 * Persisted in NVS; takes effect on the next reboot / USB re-enumerate, since
 * the USB descriptor is fixed at TinyUSB install. */
/* === POST /api/enter_download?confirm=1 ==================================
 * Headless recovery: force the ROM into USB download mode on the next boot,
 * then reset. Lets a field/bench unit be fully reflashed (bootloader +
 * partition table + app — e.g. a partition-layout change, which OTA cannot
 * deliver) WITHOUT physical BOOT+RST access. The forced-download flag is
 * one-shot in the RTC domain; a subsequent full flash clears it, and even
 * if left alone the unit simply waits in download mode (recoverable), so
 * this can strand the device offline until reflashed — hence admin auth
 * AND the required ?confirm=1 and the loud log. */
static esp_err_t api_enter_download(httpd_req_t * req)
{
  if (!ml_app_check_admin_auth(req)) {
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"pstop admin\"");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"admin auth required\"}");
  }
  char query[24], c[4] = {0};
  if (
    httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
    httpd_query_key_value(query, "confirm", c, sizeof(c)) != ESP_OK || strcmp(c, "1") != 0)
  {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(
      req,
      "{\"ok\":false,\"error\":\"needs ?confirm=1 — forces download mode, "
      "device offline until reflashed\"}");
  }
  ESP_LOGW(
    TAG,
    "ENTER DOWNLOAD MODE requested — device will wait for a full "
    "flash after reset (headless recovery)");
  httpd_resp_set_type(req, "application/json");
  (void)httpd_resp_sendstr(req, "{\"ok\":true,\"message\":\"Entering download mode. Full-flash to recover.\"}");
  vTaskDelay(pdMS_TO_TICKS(200)); /* let the response flush */
  REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
  esp_rom_software_reset_system();
  return ESP_OK; /* not reached */
}

static esp_err_t api_pstop_num(httpd_req_t * req)
{
  char query[48], val[8];
  if (
    (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) ||
    (httpd_query_key_value(query, "n", val, sizeof(val)) != ESP_OK))
  {
    (void)httpd_resp_set_status(req, "400 Bad Request");
    (void)httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing ?n=0..99\"}");
  }
  int n = (int)strtol(val, NULL, 10);
  if ((n < 0) || (n > 99)) {
    (void)httpd_resp_set_status(req, "400 Bad Request");
    (void)httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"n must be 0..99 (0=auto)\"}");
  }
  esp_err_t r = dcs_nvs_write_pstop_unit_num((uint8_t)n);
  char buf[112];
  int len = snprintf(
    buf,
    sizeof(buf),
    "{\"ok\":%s,\"pstop_num\":%d,\"note\":\"effective on next reboot\"}",
    (r == ESP_OK) ? "true" : "false",
    n);
  (void)httpd_resp_set_type(req, "application/json");
  if (r != ESP_OK) {
    (void)httpd_resp_set_status(req, "500 Internal Server Error");
  }
  return httpd_resp_send(req, buf, len);
}

/* === Public registration =================================================== */
void dcs_admin_pages_register(ml_app_t * app)
{
  if (!app) {
    ESP_LOGE(TAG, "dcs_admin_pages_register: app is NULL");
    return;
  }
  (void)ml_app_add_page(app, "/", HTTP_GET, page_index);
  (void)ml_app_add_page(app, "/state.json", HTTP_GET, page_state);
  (void)ml_app_add_page(app, "/api/derp", HTTP_POST, api_derp_toggle);
  (void)ml_app_add_page(app, "/api/derp_delay", HTTP_POST, api_derp_delay);
  (void)ml_app_add_page(app, "/api/wg", HTTP_POST, api_wg_toggle);
  (void)ml_app_add_page(app, "/api/wifi_tx_power", HTTP_POST, api_wifi_tx_power);
  (void)ml_app_add_page(app, "/api/usb_enable", HTTP_POST, api_usb_enable_toggle);
  (void)ml_app_add_page(app, "/api/ts_boot", HTTP_POST, api_ts_boot_toggle);
  (void)ml_app_add_page(app, "/api/pstop_peer", HTTP_POST, api_pstop_peer);
  (void)ml_app_add_page(app, "/api/last_log", HTTP_GET, api_last_log);
  (void)ml_app_add_page(app, "/api/iface/eth", HTTP_POST, api_iface_eth);
  (void)ml_app_add_page(app, "/api/iface/wifi", HTTP_POST, api_iface_wifi);
  (void)ml_app_add_page(app, "/api/iface/usb", HTTP_POST, api_iface_usb);
  (void)ml_app_add_page(app, "/api/pstop_num", HTTP_POST, api_pstop_num);
  (void)ml_app_add_page(app, "/api/enter_download", HTTP_POST, api_enter_download);
}
