// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ml_app.h
 * @brief MicroLink Application Framework
 *
 * High-level convenience layer that absorbs all ESP32 boilerplate:
 * NVS, WiFi, Tailscale connection, OTA with rollback, admin web UI.
 *
 * User code is typically ~15 lines:
 *   ml_app_start() → ml_app_add_page() → done.
 *
 * Admin panel is served at /admin/ with HTTP Basic Auth.
 * User pages are served at / with no auth.
 */

#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "microlink.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct ml_app ml_app_t;

  typedef void (*ml_app_connected_cb_t)(ml_app_t * app, void * user_data);

  /* Optional pre-WiFi network bring-up hook. Called by ml_app_start() before
 * wifi_init(); if it returns ESP_OK, WiFi is left untouched and ml_app
 * proceeds on whatever default netif the hook brought up. Anything else
 * (e.g. ESP_ERR_TIMEOUT, ESP_ERR_NOT_FOUND) means "fall through to WiFi".
 * Used by ml_dev_tether to bring up USB-CDC-NCM and prefer host-routed
 * internet when a USB host is present. NULL = skip this step entirely
 * (default behaviour). */
  typedef esp_err_t (*ml_app_alt_network_fn_t)(uint32_t timeout_ms);

  typedef struct
  {
    int8_t wifi_tx_power_dbm; /* 0 = default */
    uint8_t max_peers; /* 0 = Kconfig default */
    bool enable_derp;
    bool enable_stun;
    bool enable_disco;
    uint16_t http_port; /* default 80 */
    uint8_t max_user_uri_handlers; /* extra slots for user handlers, default 8 */
    uint8_t http_max_sockets; /* concurrent httpd connections; 0 = IDF
                                       default (7). Lower this on no-PSRAM
                                       targets: each open socket can queue
                                       TCP_SND_BUF bytes of send buffer, and
                                       a browser opens ~6 connections at
                                       once — measured OOM at 7×18 KB. */
    uint8_t http_priority; /* FreeRTOS task priority for httpd
                                       (0 = use default 10). esp_http_server's
                                       built-in default is tskIDLE_PRIORITY+5 which
                                       ties with most user/microlink tasks; that
                                       lets HTTP requests stall behind unrelated
                                       work. 10 keeps the admin UI responsive. */
    ml_app_alt_network_fn_t try_alt_network; /* Optional. See typedef above. */
    uint32_t alt_network_timeout_ms; /* Timeout passed to try_alt_network.
                                                 0 = use ml_app default (5000 ms). */
    const char * device_name; /* Tailscale node hostname. NULL/""
                                                 = Kconfig CONFIG_ML_DEVICE_NAME.
                                                 Must outlive ml_app_start (point
                                                 at a static/const buffer). */
  } ml_app_config_t;

#define ML_APP_CONFIG_DEFAULT()                                            \
  {                                                                        \
    .wifi_tx_power_dbm = 20, /* max within ESP_PHY_MAX_WIFI_TX_POWER=20 */ \
    .max_peers = 0,                                                        \
    .enable_derp = true,                                                   \
    .enable_stun = true,                                                   \
    .enable_disco = true,                                                  \
    .http_port = 80,                                                       \
    .max_user_uri_handlers = 8,                                            \
    .http_priority = 10,                                                   \
    .try_alt_network = NULL,                                               \
    .alt_network_timeout_ms = 0,                                           \
    .device_name = NULL,                                                   \
  }

  /**
 * @brief Start the MicroLink application framework
 *
 * Performs all initialization:
 *   1. NVS init
 *   2. WiFi connect (blocks until connected)
 *   3. HTTP server start (user pages accessible on local WiFi)
 *   4. MicroLink init + start (Tailscale connects in background)
 *   5. Admin routes at /admin/ with Basic Auth
 *   6. OTA rollback confirmed when Tailscale connects
 *
 * Returns immediately after WiFi connects. User pages are accessible
 * on the local WiFi IP before Tailscale finishes connecting.
 */
  ml_app_t * ml_app_start(const ml_app_config_t * cfg);

  /**
 * @brief Register a user HTTP handler (no auth required)
 */
  esp_err_t ml_app_add_page(
    ml_app_t * app, const char * uri, httpd_method_t method, esp_err_t (*handler)(httpd_req_t *));

  /**
 * @brief Register a callback for when Tailscale VPN connects
 */
  esp_err_t ml_app_on_connected(ml_app_t * app, ml_app_connected_cb_t cb, void * user_data);

  /**
 * @brief Get the raw httpd handle for advanced use
 */
  httpd_handle_t ml_app_get_httpd(const ml_app_t * app);

  /**
 * @brief Get the raw microlink handle for advanced use
 */
  microlink_t * ml_app_get_microlink(const ml_app_t * app);

  /**
 * @brief True when the framework gave up on saved WiFi and is hosting its
 *        own SoftAP. User apps can use this to render a "configure WiFi
 *        via /admin/" banner. See CONFIG_ML_AP_FALLBACK_* for tuning.
 */
  bool ml_app_in_ap_fallback(const ml_app_t * app);

  /**
 * @brief SSID of the fallback AP if active, NULL otherwise.
 *        Pointer is owned by ml_app — copy if you need it longer than
 *        the next event.
 */
  const char * ml_app_ap_ssid(const ml_app_t * app);

  /**
 * @brief Check HTTP Basic admin auth on a request, using the same
 *        admin:CONFIG_ML_ADMIN_PASSWORD credential that guards the
 *        built-in /admin/ routes. Lets an app protect its own
 *        sensitive user-registered pages (which otherwise have no auth).
 * @return true if authorized (or if no admin password is configured, in
 *         which case the whole admin surface is open by design); false
 *         means the caller should return 401.
 */
  bool ml_app_check_admin_auth(httpd_req_t * req);

#ifdef __cplusplus
}
#endif
