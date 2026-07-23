// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dcs_boot.c
 * @brief Boot-path glue: the USB-tether-or-WiFi decision and the WiFi
 * driver idle init for the USB-tether path.
 */

#include "dcs_internal.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "ml_dev_tether.h"

static const char * TAG = "dcs_boot";

/* How long to wait at boot for the W5500 link + DHCP lease before falling
 * through to the next-priority interface. The Ethernet netif is left running
 * regardless, so a cable plugged in later is still promoted to the default
 * route by dcs_net_supervisor — this only bounds how long boot blocks. */
#define DCS_ETH_BOOT_WAIT_MS 6000

/* True once an alt-network path (Ethernet or USB-NCM) took the default route
 * and ml_app therefore skipped its blocking WiFi connect. dcs_support reads
 * this to decide whether to bring the WiFi driver up in idle mode. */
static bool s_alt_network_won = false;

bool dcs_boot_alt_network_won(void)
{
  return s_alt_network_won;
}

/* ml_app's pre-WiFi network hook. Priority order for this board:
 *   1. W5500 Ethernet  2. USB-CDC-NCM tether  3. (fall through) WiFi
 *
 * Returning ESP_OK tells ml_app an alternative network is up, so it does NOT
 * run its blocking WiFi connect + AP fallback. Returning non-OK lets ml_app
 * proceed to WiFi (which itself falls back to a setup SoftAP). */
esp_err_t dcs_boot_try_tether_or_skip(uint32_t timeout_ms)
{
  /* 1. Wired Ethernet — always brought up (kept alive for runtime hot-plug),
     *    highest priority. */
  esp_err_t e = dcs_eth_start();
  if (e == ESP_OK) {
    if (dcs_eth_wait_for_ip(DCS_ETH_BOOT_WAIT_MS)) {
      ESP_LOGI(
        TAG,
        "W5500 Ethernet up with DHCP lease — preferring wired, "
        "skipping WiFi");
      s_alt_network_won = true;
      return ESP_OK;
    }
    ESP_LOGW(
      TAG,
      "W5500 no link/lease in %dms — trying next interface "
      "(eth stays live for runtime promotion)",
      DCS_ETH_BOOT_WAIT_MS);
  } else {
    ESP_LOGE(TAG, "dcs_eth_start failed: %s — continuing without Ethernet", esp_err_to_name(e));
  }

  /* 2. USB-CDC-NCM tether (opt-in via NVS). */
  if (g_dcs.usb_enabled) {
    esp_err_t r = ml_dev_tether_try_start(timeout_ms);
    ESP_LOGI(TAG, "USB tether: %s (used_alt_network=%d)", esp_err_to_name(r), (int)(r == ESP_OK));
    if (r == ESP_OK) {
      s_alt_network_won = true;
      return ESP_OK; /* ml_app skips wifi_init */
    }
  } else {
    ESP_LOGW(TAG, "USB tether disabled via NVS");
  }

  /* 3. Fall through — ml_app brings up WiFi STA, then SoftAP setup mode. */
  ESP_LOGW(TAG, "No wired/USB network — falling through to WiFi");
  return ESP_ERR_NOT_SUPPORTED;
}

/* Bring up the WiFi driver in idle mode so admin endpoints that touch
 * esp_wifi_* APIs (monitor, status) don't NULL-deref the uninitialized
 * driver. Called only when USB tether took the "ESP_OK skip WiFi" path.
 *
 * Deliberately NOT calling esp_wifi_set_mode/esp_wifi_start. Admin API
 * handlers tolerate ESP_ERR_WIFI_NOT_STARTED but cannot tolerate the
 * scan-spam interrupt storm caused by starting WiFi with no SSID. */
void dcs_boot_wifi_idle_init(void)
{
  if (esp_netif_create_default_wifi_sta() == NULL) {
    ESP_LOGW(TAG, "esp_netif_create_default_wifi_sta returned NULL");
  }
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_err_t r = esp_wifi_init(&cfg);
  if (r != ESP_OK) {
    ESP_LOGW(TAG, "esp_wifi_init: %s", esp_err_to_name(r));
    return;
  }
  r = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (r != ESP_OK) {
    ESP_LOGW(TAG, "esp_wifi_set_storage: %s", esp_err_to_name(r));
  }
  ESP_LOGI(TAG, "WiFi driver inited (not started) — radio quiet");
}

/* === USB-CDC-NCM tether admin toggle ====================================== */

static atomic_bool s_usb_enabled = false;
/* Serialises USB-NCM bring-up/tear-down. BOTH the net supervisor (auto-failover)
 * and the admin toggle (httpd thread) call dcs_usb_set_enabled, and
 * ml_dev_tether_try_start()/ml_dev_tether_stop() must NOT overlap — a concurrent
 * esp_netif_new() (bring-up) racing esp_netif_destroy() (tear-down) on the same
 * netif is a use-after-free/double-free panic (HIL-confirmed). Held across the
 * async bring-up task; a loser gets ESP_ERR_INVALID_STATE and retries (the
 * supervisor next tick, the toggle on the next click). Mirrors dcs_wifi. */
static atomic_bool s_usb_busy = false;

bool dcs_usb_is_enabled(void)
{
  return atomic_load(&s_usb_enabled);
}

/* ml_dev_tether_try_start() installs TinyUSB and blocks up to timeout_ms for a
 * DHCP lease, so run it off the httpd task. NB: bringing USB-NCM up replaces
 * the native USB-Serial-JTAG with a TinyUSB composite — flashing then needs
 * OTA until the next reboot. Releases s_usb_busy when the bring-up completes. */
static void usb_enable_task(void * arg)
{
  (void)arg;
  esp_err_t r = ml_dev_tether_try_start(15000);
  ESP_LOGW(TAG, "USB-NCM bring-up: %s", esp_err_to_name(r));
  atomic_store(&s_usb_busy, false);
  vTaskDelete(NULL);
}

esp_err_t dcs_usb_set_enabled(bool on)
{
  if (on == atomic_load(&s_usb_enabled)) return ESP_OK;
  /* Claim the bring-up/tear-down lock; a concurrent caller backs off + retries. */
  bool expected = false;
  if (!atomic_compare_exchange_strong(&s_usb_busy, &expected, true)) {
    return ESP_ERR_INVALID_STATE;
  }
  atomic_store(&s_usb_enabled, on);
  if (on) {
    ESP_LOGW(TAG, "USB-NCM ENABLED (installs TinyUSB)");
    if (xTaskCreate(usb_enable_task, "usb_en", 4096, NULL, 5, NULL) != pdPASS) {
      /* spawn failed (internal heap) — roll back state + release the lock */
      atomic_store(&s_usb_enabled, false);
      atomic_store(&s_usb_busy, false);
      ESP_LOGE(TAG, "USB-NCM enable: usb_en task create failed");
      return ESP_ERR_NO_MEM;
    }
    /* s_usb_busy released by usb_enable_task once bring-up finishes */
  } else {
    ESP_LOGW(TAG, "USB-NCM DISABLED");
    ml_dev_tether_stop(); /* netif down → supervisor demotes USB */
    atomic_store(&s_usb_busy, false);
  }
  return ESP_OK;
}
