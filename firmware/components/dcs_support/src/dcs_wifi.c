// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dcs_wifi.c
 * @brief Runtime WiFi-STA enable/disable (admin toggle + lazy failover).
 *
 * On the wired-Ethernet boot path the WiFi driver is intentionally NOT brought
 * up (esp_wifi_init costs RAM the WG/TLS stack needs). This module brings WiFi
 * up on demand — from the admin toggle, or as a runtime failover — and tears
 * it fully back down (deinit + destroy netif) on disable so Tailscale gets the
 * RAM back. A minimal WiFi init config keeps it ~18 KB (vs ~40 KB default) so
 * it can coexist with Tailscale.
 *
 * It cycles the configured WiFi list (ml_config) on association failure, like
 * ml_app does, so it connects to whichever configured SSID is actually present
 * — no hard-coded network.
 *
 * dcs_net_supervisor needs no changes: once WIFI_STA_DEF gets a lease it's a
 * usable candidate; stopped, it drops out.
 *
 * NOTE: owns the WiFi lifecycle only on the wired/USB boot paths (where ml_app
 * skipped wifi_init). Don't use when ml_app brought WiFi up itself.
 */

#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#include "dcs_internal.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "ml_config_httpd.h"
#include "sdkconfig.h"

static const char * TAG = "dcs_wifi";

/* Reconnect backoff: pace retries so "WiFi enabled but no configured AP in
 * range" is a calm idle state, not a scan-thrash loop. Inline reconnect on
 * every STA_DISCONNECTED kept the WiFi task scanning continuously, which
 * starved the Ethernet pstop path (climbing rebonds) and the internal heap. */
#define WIFI_BACKOFF_MIN_MS 2000u
#define WIFI_BACKOFF_MAX_MS 30000u

static atomic_bool s_enabled = false; /* driver running (gates on_wifi_evt) */
static atomic_bool s_want_up = false; /* admin intent — survives a NO_MEM   */
static bool s_inited = false; /* driver + netif created by us */
static esp_netif_t * s_sta = NULL;
static ml_config_wifi_list_t * s_list = NULL; /* configured networks (PSRAM) */
static int s_idx = 0; /* which list entry we're trying */
static atomic_int s_disc_reason = 0; /* last STA_DISCONNECTED reason  */
static atomic_int s_connected = 0; /* associated (pre-DHCP) flag    */
static esp_timer_handle_t s_reconnect_timer = NULL; /* one-shot paced retry */
static esp_timer_handle_t s_init_retry = NULL; /* re-attempt bring-up after burst */
static uint32_t s_backoff_ms = WIFI_BACKOFF_MIN_MS; /* event-task only */

/* Diagnostics for /state.json: why isn't WiFi connecting? */
void dcs_wifi_status(int * reason, int * connected, int * idx, int * count)
{
  if (reason != NULL) {
    *reason = atomic_load(&s_disc_reason);
  }
  if (connected != NULL) {
    *connected = atomic_load(&s_connected);
  }
  if (count != NULL) {
    *count = (s_list != NULL) ? s_list->count : 0;
  }
  if (idx != NULL) {
    *idx = ((s_list != NULL) && (s_list->count != 0)) ? (s_idx % s_list->count) : -1;
  }
}

/* Point the STA at the current list entry (or the compiled default if the list
 * is empty) and (re)connect. */
static void apply_creds_and_connect(void)
{
  char ssid[33] = {0}, pass[65] = {0};
  if ((s_list != NULL) && (s_list->count > 0)) {
    int i = s_idx % s_list->count;
    (void)snprintf(ssid, sizeof(ssid), "%s", s_list->entries[i].ssid);
    (void)snprintf(pass, sizeof(pass), "%s", s_list->entries[i].pass);
  } else {
    (void)snprintf(ssid, sizeof(ssid), "%s", CONFIG_ML_WIFI_SSID);
    (void)snprintf(pass, sizeof(pass), "%s", CONFIG_ML_WIFI_PASSWORD);
  }
  wifi_config_t wc = {0};
  (void)memcpy(wc.sta.ssid, ssid, sizeof(wc.sta.ssid));
  (void)memcpy(wc.sta.password, pass, sizeof(wc.sta.password));
  /* Negotiate PMF + WPA3-SAE so we can join WPA2/WPA3 transition-mode APs
     * (common on corporate SSIDs). Without this the 4-way handshake on a
     * mixed-mode AP times out even with the right password. */
  wc.sta.pmf_cfg.capable = true;
  wc.sta.pmf_cfg.required = false;
  wc.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
  esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wc);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_set_config: %s", esp_err_to_name(err));
  }
  ESP_LOGI(TAG, "WiFi trying SSID '%s'", ssid);
  err = esp_wifi_connect();
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "esp_wifi_connect: %s", esp_err_to_name(err));
  }
}

/* Timer-deferred reconnect: fires on the esp_timer task after the backoff. */
static void reconnect_timer_cb(void * arg)
{
  (void)arg;
  if (atomic_load(&s_enabled)) {
    apply_creds_and_connect();
  }
}

/* Re-attempt bring-up after a NO_MEM. The Tailscale bring-up / DERP-TLS burst
 * transiently consumes the internal RAM esp_wifi_init needs, so enabling WiFi
 * during that window fails. Retry on a timer until the heap settles — unless
 * the user disabled in the meantime (s_want_up cleared). */
static void init_retry_cb(void * arg)
{
  (void)arg;
  if (atomic_load(&s_want_up) && (!s_inited)) {
    (void)dcs_wifi_set_enabled(true);
  }
}

/* Arm the one-shot reconnect after the current backoff, then escalate it
 * (2 s → 4 → 8 … capped at 30 s). Coalesces duplicate STA_DISCONNECTED. */
static void schedule_reconnect(void)
{
  if (s_reconnect_timer == NULL) {
    return;
  }
  (void)esp_timer_stop(s_reconnect_timer);
  (void)esp_timer_start_once(s_reconnect_timer, (uint64_t)s_backoff_ms * 1000u);
  s_backoff_ms = (s_backoff_ms < (WIFI_BACKOFF_MAX_MS / 2u)) ? (s_backoff_ms * 2u) : WIFI_BACKOFF_MAX_MS;
}

/* STA_START → first connect. STA_DISCONNECTED (failed/dropped) → advance to the
 * next configured SSID and arm a backed-off retry (NOT an inline reconnect —
 * that scan-thrashes when no AP is in range). STA_CONNECTED → reset backoff. */
static void on_wifi_evt(void * arg, esp_event_base_t base, int32_t id, void * data)
{
  (void)arg;
  (void)base;
  (void)data;
  if (id == (int32_t)WIFI_EVENT_STA_CONNECTED) {
    atomic_store(&s_connected, 1);
    s_backoff_ms = WIFI_BACKOFF_MIN_MS; /* associated → retry fast next time */
    return;
  }
  if (!atomic_load(&s_enabled)) {
    return;
  }
  if (id == (int32_t)WIFI_EVENT_STA_START) {
    s_backoff_ms = WIFI_BACKOFF_MIN_MS;
    apply_creds_and_connect();
  } else if (id == (int32_t)WIFI_EVENT_STA_DISCONNECTED) {
    atomic_store(&s_connected, 0);
    if (data != NULL) {
      atomic_store(&s_disc_reason, ((wifi_event_sta_disconnected_t *)data)->reason);
    }
    if ((s_list != NULL) && (s_list->count > 1)) {
      s_idx++; /* try the next network */
    }
    schedule_reconnect(); /* paced retry, not thrash */
  } else {
    /* other WiFi events are not of interest */
  }
}

bool dcs_wifi_is_enabled(void)
{
  return atomic_load(&s_enabled);
}

static esp_err_t wifi_set_enabled_locked(bool on)
{
  atomic_store(&s_want_up, on); /* record intent before any early return */
  if ((!on) && (s_init_retry != NULL)) {
    (void)esp_timer_stop(s_init_retry); /* cancel pending bring-up */
  }
  if (on == atomic_load(&s_enabled)) {
    return ESP_OK;
  }

  if (on) {
    if (!s_inited) {
      /* The driver may already be inited by dcs_boot_wifi_idle_init() on
             * the USB-won-at-boot path. Detect that (esp_wifi_get_mode succeeds
             * only once inited) and reuse it rather than double-initialising. */
      wifi_mode_t mode;
      bool pre_inited = (esp_wifi_get_mode(&mode) == ESP_OK);
      if (pre_inited) {
        s_sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        ESP_LOGI(TAG, "WiFi driver pre-inited (USB boot path) — reusing");
      } else {
        s_sta = esp_netif_create_default_wifi_sta();
        /* Default WiFi buffers. An earlier build shrank these to ~18 KB to
                 * fit alongside the WG/TLS stack, but the starved RX ring dropped
                 * EAPOL frames and the WPA2 4-way handshake timed out even at good
                 * signal. Now that dynamic mbedTLS buffers freed ~50 KB internal,
                 * the stock buffers (with AMPDU) fit — and the handshake completes.
                 * Only nvs_enable is overridden (we use WIFI_STORAGE_RAM). */
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        cfg.nvs_enable = 0;
        esp_err_t r = esp_wifi_init(&cfg);
        if (r != ESP_OK) {
          ESP_LOGE(
            TAG,
            "esp_wifi_init: %s — internal RAM tight (likely the "
            "Tailscale bring-up burst); will retry",
            esp_err_to_name(r));
          if (s_sta != NULL) {
            esp_netif_destroy_default_wifi(s_sta);
            s_sta = NULL;
          }
          /* Transient: the WG/DERP-TLS burst grabbed the internal RAM. Arm a
                     * deferred retry so the toggle/failover succeeds once it settles. */
          if (s_init_retry == NULL) {
            esp_timer_create_args_t rt = {.callback = init_retry_cb, .name = "dcs_wifi_ir"};
            (void)esp_timer_create(&rt, &s_init_retry);
          }
          if ((s_init_retry != NULL) && atomic_load(&s_want_up)) {
            (void)esp_timer_start_once(s_init_retry, 4000000u); /* 4 s */
          }
          return r;
        }
        (void)esp_wifi_set_storage(WIFI_STORAGE_RAM);
      }
      (void)esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_evt, NULL);
      esp_timer_create_args_t ta = {.callback = reconnect_timer_cb, .name = "dcs_wifi_rc"};
      (void)esp_timer_create(&ta, &s_reconnect_timer);
      s_inited = true;
    }
    s_backoff_ms = WIFI_BACKOFF_MIN_MS;

    /* Load the configured network list (PSRAM; ~1.5 KB). NULL/empty → the
         * compiled default is used by apply_creds_and_connect(). */
    if (s_list == NULL) {
      s_list = heap_caps_malloc(sizeof(*s_list), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if ((s_list != NULL) && (!ml_config_get_wifi_list(s_list))) {
      s_list->count = 0;
    }
    s_idx = 0;

    esp_err_t merr = esp_wifi_set_mode(WIFI_MODE_STA);
    if (merr != ESP_OK) {
      ESP_LOGE(TAG, "esp_wifi_set_mode: %s", esp_err_to_name(merr));
    }
    atomic_store(&s_enabled, true); /* before start so STA_START connects */
    esp_err_t r = esp_wifi_start(); /* → STA_START → apply_creds_and_connect */
    if (r != ESP_OK) {
      ESP_LOGE(TAG, "esp_wifi_start: %s", esp_err_to_name(r));
      atomic_store(&s_enabled, false);
      return r;
    }
    /* Modem power-save MUST be off for the 10 Hz protective-stop link.
         * The boot-time ml_app WiFi path already does this; this runtime
         * enable path didn't — with default WIFI_PS_MIN_MODEM the radio
         * sleeps between DTIM beacons and drops most unicast RX: measured
         * 2026-07-20 as a re-bond storm (60+ rebonds in minutes — chip TX
         * fine, machine replies never heard, ack-lag > max_lost every ~2 s). */
    esp_err_t perr = esp_wifi_set_ps(WIFI_PS_NONE);
    if (perr != ESP_OK) {
      ESP_LOGE(TAG, "esp_wifi_set_ps: %s", esp_err_to_name(perr));
    }
    ESP_LOGW(
      TAG, "WiFi ENABLED (power-save OFF) — cycling %d configured network(s)", (s_list != NULL) ? s_list->count : 0);
    return ESP_OK;
  }

  /* Disable: stop and FULLY deinit so the ~18 KB internal RAM returns. */
  atomic_store(&s_enabled, false); /* clear first so on_wifi_evt won't reconnect */
  if (s_reconnect_timer != NULL) {
    (void)esp_timer_stop(s_reconnect_timer); /* no pending retry */
  }
  (void)esp_wifi_disconnect();
  (void)esp_wifi_stop();
  if (s_inited) {
    if (s_reconnect_timer != NULL) {
      (void)esp_timer_delete(s_reconnect_timer);
      s_reconnect_timer = NULL;
    }
    (void)esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_evt);
    (void)esp_wifi_deinit();
    if (s_sta != NULL) {
      esp_netif_destroy_default_wifi(s_sta);
      s_sta = NULL;
    }
    s_inited = false;
  }
  /* Keep the PSRAM list buffer allocated for the process lifetime and just
     * mark it empty, rather than free()ing it. dcs_wifi_status() reads
     * s_list->count from the httpd thread without a lock; freeing it here
     * (WiFi task) was a use-after-free race. The buffer is ~1.6 KB PSRAM —
     * not the ~18 KB of INTERNAL RAM this disable path exists to reclaim —
     * so holding it costs nothing that matters, and a re-enable repopulates
     * it in place (see the alloc-once guard above). */
  if (s_list != NULL) {
    s_list->count = 0;
  }
  ESP_LOGW(TAG, "WiFi DISABLED — driver deinit, internal RAM freed");
  return ESP_OK;
}

/* Serialize bring-up/tear-down: both the admin toggle (httpd thread) and the
 * net supervisor (auto-failover) call this, and the esp_wifi_init/deinit pair
 * must not overlap. A concurrent caller gets ESP_ERR_INVALID_STATE and retries
 * (the supervisor next tick; the toggle on the next click). */
esp_err_t dcs_wifi_set_enabled(bool on)
{
  static atomic_bool s_busy = false;
  bool expected = false;
  if (!atomic_compare_exchange_strong(&s_busy, &expected, true)) {
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t r = wifi_set_enabled_locked(on);
  atomic_store(&s_busy, false);
  return r;
}
