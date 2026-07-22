// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ml_app.c
 * @brief MicroLink Application Framework — absorbs all ESP32 boilerplate
 */

#include "ml_app.h"

#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/base64.h"
#include "mbedtls/sha256.h"
#include "microlink.h"
#include "microlink_internal.h"
#include "ml_config_httpd.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char * TAG = "ml_app";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_MAX_RETRIES 3

/* ============================================================================
 * App Context
 * ========================================================================== */

struct ml_app
{
  microlink_t * ml;
  httpd_handle_t httpd;
  EventGroupHandle_t wifi_events;

  ml_app_connected_cb_t connected_cb;
  void * connected_cb_data;

  /* WiFi state */
  ml_config_wifi_list_t wifi_list;
  int wifi_list_count;
  int current_wifi_idx;
  int wifi_retry_count;
  char wifi_ssid[33];
  char wifi_password[65];

  /* AP fallback state */
  int64_t wifi_start_us; /* when STA mode first armed */
  volatile bool ap_fallback_active;
  char ap_ssid[33]; /* "<prefix>-XXYYZZ" */

  /* Fleet OTA */
  volatile bool tailscale_connected;
  volatile bool auto_update;
  volatile int check_interval_s;
  volatile int ota_poll_state; /* 0=idle, 1=checking, 2=update found, 3=downloading, 4=rebooting, -1=error */
  char ota_poll_msg[64];
  volatile bool ota_check_now; /* set by manual check button */
  volatile int ota_dl_percent; /* 0-100 download progress */
  volatile bool verbose_log;
};

/* File-static app pointer for WiFi event handler */
static ml_app_t * s_app = NULL;

/* ============================================================================
 * HTTP Basic Auth
 * ========================================================================== */

#ifdef CONFIG_ML_ADMIN_PASSWORD
static bool admin_auth_check(httpd_req_t * req)
{
  const char * password = CONFIG_ML_ADMIN_PASSWORD;
  if (!password || password[0] == '\0') return true;

  char auth_hdr[256];
  if (httpd_req_get_hdr_value_str(req, "Authorization", auth_hdr, sizeof(auth_hdr)) != ESP_OK) {
    return false;
  }
  if (strncmp(auth_hdr, "Basic ", 6) != 0) return false;

  unsigned char decoded[128];
  size_t decoded_len = 0;
  if (
    mbedtls_base64_decode(
      decoded, sizeof(decoded) - 1, &decoded_len, (const unsigned char *)(auth_hdr + 6), strlen(auth_hdr + 6)) != 0)
  {
    return false;
  }
  decoded[decoded_len] = '\0';

  char expected[160];
  snprintf(expected, sizeof(expected), "admin:%s", password);
  return strcmp((char *)decoded, expected) == 0;
}
#endif

bool ml_app_check_admin_auth(httpd_req_t * req)
{
#ifdef CONFIG_ML_ADMIN_PASSWORD
  return admin_auth_check(req);
#else
  (void)req;
  return true; /* no admin password configured → admin surface is open */
#endif
}

/* ============================================================================
 * WiFi Setup
 * ========================================================================== */

static void wifi_try_next(ml_app_t * app)
{
  if (app->wifi_list_count <= 1) {
    esp_wifi_connect();
    return;
  }

  app->wifi_retry_count++;
  if (app->wifi_retry_count >= WIFI_MAX_RETRIES) {
    app->wifi_retry_count = 0;
    app->current_wifi_idx = (app->current_wifi_idx + 1) % app->wifi_list_count;
  }

  ml_config_wifi_entry_t * e = &app->wifi_list.entries[app->current_wifi_idx];
  wifi_config_t wifi_config = {
    .sta = {.threshold.authmode = WIFI_AUTH_WPA2_PSK},
  };
  strncpy((char *)wifi_config.sta.ssid, e->ssid, sizeof(wifi_config.sta.ssid) - 1);
  strncpy((char *)wifi_config.sta.password, e->pass, sizeof(wifi_config.sta.password) - 1);

  ESP_LOGI(TAG, "WiFi trying #%d/%d: %s", app->current_wifi_idx + 1, app->wifi_list_count, e->ssid);

  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  esp_wifi_connect();
}

#if CONFIG_ML_AP_FALLBACK_ENABLE
static void ap_fallback_start(ml_app_t * app)
{
  if (app->ap_fallback_active) return;

  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  snprintf(app->ap_ssid, sizeof(app->ap_ssid), "%s-%02x%02x%02x", CONFIG_ML_AP_SSID_PREFIX, mac[3], mac[4], mac[5]);

  const char * password = CONFIG_ML_AP_PASSWORD;
  bool open = (strlen(password) < 8);

  ESP_LOGW(
    TAG,
    "AP fallback: no STA connection after %d s — hosting %s (%s)",
    CONFIG_ML_AP_FALLBACK_TIMEOUT_S,
    app->ap_ssid,
    open ? "open" : "WPA2");

  wifi_config_t ap_cfg = {
    .ap =
      {
        .channel = 1,
        .authmode = open ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK,
        .max_connection = 4,
        .pmf_cfg = {.required = false},
      },
  };
  strncpy((char *)ap_cfg.ap.ssid, app->ap_ssid, sizeof(ap_cfg.ap.ssid) - 1);
  ap_cfg.ap.ssid_len = strlen(app->ap_ssid);
  if (!open) {
    strncpy((char *)ap_cfg.ap.password, password, sizeof(ap_cfg.ap.password) - 1);
  }

  esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "set_mode(APSTA) failed: %s", esp_err_to_name(err));
    return;
  }
  err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "set_config(AP) failed: %s", esp_err_to_name(err));
    return;
  }

  /* Mark the running firmware valid so the bootloader doesn't roll back —
     * STA never came up but we're alive and serving on the AP, so this build
     * is functional. */
  esp_ota_mark_app_valid_cancel_rollback();
  ESP_LOGI(TAG, "OTA: firmware confirmed valid via AP fallback");

  app->ap_fallback_active = true;

  /* Unblock ml_app_start so httpd binds to the now-up AP netif and the
     * user's app proceeds. */
  xEventGroupSetBits(app->wifi_events, WIFI_CONNECTED_BIT);
}

static void ap_fallback_stop(ml_app_t * app)
{
  if (!app->ap_fallback_active) return;
  ESP_LOGI(TAG, "AP fallback: STA up, shutting down %s", app->ap_ssid);
  esp_wifi_set_mode(WIFI_MODE_STA);
  app->ap_fallback_active = false;
}

bool ml_app_in_ap_fallback(const ml_app_t * app)
{
  return app && app->ap_fallback_active;
}

const char * ml_app_ap_ssid(const ml_app_t * app)
{
  return (app && app->ap_fallback_active) ? app->ap_ssid : NULL;
}
#else
bool ml_app_in_ap_fallback(const ml_app_t * app)
{
  (void)app;
  return false;
}

const char * ml_app_ap_ssid(const ml_app_t * app)
{
  (void)app;
  return NULL;
}
#endif

static void wifi_event_handler(void * arg, esp_event_base_t event_base, int32_t event_id, void * event_data)
{
  ml_app_t * app = s_app;
  if (!app) return;

  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    wifi_event_sta_disconnected_t * disc = (wifi_event_sta_disconnected_t *)event_data;
    ESP_LOGW(TAG, "WiFi disconnected, reason=%d", disc->reason);
#if CONFIG_ML_AP_FALLBACK_ENABLE
    if (!app->ap_fallback_active) {
      int64_t elapsed_us = esp_timer_get_time() - app->wifi_start_us;
      if (elapsed_us > (int64_t)CONFIG_ML_AP_FALLBACK_TIMEOUT_S * 1000000LL) {
        ap_fallback_start(app);
      }
    }
#endif
    wifi_try_next(app);
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t * event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "WiFi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
    app->wifi_retry_count = 0;
#if CONFIG_ML_AP_FALLBACK_ENABLE
    /* Restart the AP-fallback timeout on each successful DHCP — a flaky
         * AP that boots us, drops us, and reconnects within seconds shouldn't
         * push the device into AP fallback just because the *first* boot
         * took >60 s to complete. */
    app->wifi_start_us = esp_timer_get_time();
    if (app->ap_fallback_active) ap_fallback_stop(app);
#endif
    xEventGroupSetBits(app->wifi_events, WIFI_CONNECTED_BIT);
  }
}

static void wifi_init(ml_app_t * app)
{
  app->wifi_events = xEventGroupCreate();

  /* esp_netif_init() and esp_event_loop_create_default() are now hoisted
     * to ml_app_start() so they happen before any try_alt_network hook can
     * run. Both are idempotent — ESP_ERR_INVALID_STATE means already
     * initialised (e.g. by ml_dev_tether), which is fine. */
  esp_err_t e1 = esp_netif_init();
  if (e1 != ESP_OK && e1 != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(e1);
  esp_err_t e2 = esp_event_loop_create_default();
  if (e2 != ESP_OK && e2 != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(e2);
  esp_netif_create_default_wifi_sta();
#if CONFIG_ML_AP_FALLBACK_ENABLE
  /* Pre-create the AP netif so we can flip to APSTA later without touching
     * the netif machinery from inside the event handler. Dormant until mode
     * actually includes AP. */
  esp_netif_create_default_wifi_ap();
#endif

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

  wifi_config_t wifi_config = {
    .sta = {.threshold.authmode = WIFI_AUTH_WPA2_PSK},
  };
  strncpy((char *)wifi_config.sta.ssid, app->wifi_ssid, sizeof(wifi_config.sta.ssid) - 1);
  strncpy((char *)wifi_config.sta.password, app->wifi_password, sizeof(wifi_config.sta.password) - 1);

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

  app->wifi_start_us = esp_timer_get_time();
  ESP_LOGI(TAG, "WiFi init complete, connecting to %s...", app->wifi_ssid);
}

/* ============================================================================
 * MicroLink Callbacks
 * ========================================================================== */

static void ml_state_callback(microlink_t * ml, microlink_state_t state, void * user_data)
{
  ml_app_t * app = (ml_app_t *)user_data;
  const char * names[] = {"IDLE", "WIFI_WAIT", "CONNECTING", "REGISTERING", "CONNECTED", "RECONNECTING", "ERROR"};
  const char * name = (state < sizeof(names) / sizeof(names[0])) ? names[state] : "UNKNOWN";
  ESP_LOGI(TAG, "MicroLink state: %s", name);

  if (state == ML_STATE_CONNECTED) {
    uint32_t ip = microlink_get_vpn_ip(ml);
    char ip_str[16];
    microlink_ip_to_str(ip, ip_str);
    ESP_LOGI(TAG, "Tailscale connected! VPN IP: %s", ip_str);

    app->tailscale_connected = true;

    esp_ota_mark_app_valid_cancel_rollback();
    ESP_LOGI(TAG, "OTA: firmware confirmed valid");

    if (app->connected_cb) {
      app->connected_cb(app, app->connected_cb_data);
    }
  }
}

static void ml_peer_callback(microlink_t * ml, const microlink_peer_info_t * peer, void * user_data)
{
  char ip_str[16];
  microlink_ip_to_str(peer->vpn_ip, ip_str);
  ESP_LOGI(TAG, "Peer: %s (%s) online=%d direct=%d", peer->hostname, ip_str, peer->online, peer->direct_path);
}

/* ============================================================================
 * Fleet OTA Poll Task
 * ========================================================================== */

#if defined(CONFIG_ML_OTA_BACKEND_URL) && defined(CONFIG_ML_OTA_API_KEY)

  #define OTA_STATE_IDLE 0
  #define OTA_STATE_CHECKING 1
  #define OTA_STATE_FOUND 2
  #define OTA_STATE_DOWNLOAD 3
  #define OTA_STATE_REBOOT 4
  #define OTA_STATE_ERROR -1

static void ota_set_state(ml_app_t * app, int state, const char * msg)
{
  app->ota_poll_state = state;
  if (msg) {
    strncpy((char *)app->ota_poll_msg, msg, sizeof(app->ota_poll_msg) - 1);
    ((char *)app->ota_poll_msg)[sizeof(app->ota_poll_msg) - 1] = '\0';
  } else {
    app->ota_poll_msg[0] = '\0';
  }
}

/* Response buffer for check-in JSON */
typedef struct
{
  char * buf;
  int len;
  int cap;
} http_buf_t;

static esp_err_t http_event_handler(esp_http_client_event_t * evt)
{
  http_buf_t * rb = (http_buf_t *)evt->user_data;
  if (evt->event_id == HTTP_EVENT_ON_DATA && rb) {
    if (rb->len + evt->data_len < rb->cap) {
      memcpy(rb->buf + rb->len, evt->data, evt->data_len);
      rb->len += evt->data_len;
      rb->buf[rb->len] = '\0';
    }
  }
  return ESP_OK;
}

static bool fleet_ota_checkin(
  ml_app_t * app, char * out_url, int url_size, char * out_sha256, int * out_size, bool * out_reached)
{
  if (out_reached) *out_reached = false;
  const char * backend = CONFIG_ML_OTA_BACKEND_URL;
  const char * api_key = CONFIG_ML_OTA_API_KEY;
  if (!backend[0] || !api_key[0]) return false;

  /* Build check-in JSON */
  const esp_app_desc_t * desc = esp_app_get_description();
  const esp_partition_t * invalid = esp_ota_get_last_invalid_partition();

  char url[200];
  snprintf(url, sizeof(url), "%s/api/v1/checkin", backend);

  /* Get device MAC for ID */
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char device_id[13];
  snprintf(device_id, sizeof(device_id), "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Get IPs */
  char tailscale_ip[16] = "";
  char local_ip[16] = "";
  if (app->ml && app->tailscale_connected) {
    uint32_t vpn = microlink_get_vpn_ip(app->ml);
    if (vpn) microlink_ip_to_str(vpn, tailscale_ip);
  }
  esp_netif_t * netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (netif) {
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr) {
      snprintf(local_ip, sizeof(local_ip), IPSTR, IP2STR(&ip_info.ip));
    }
  }

  cJSON * body = cJSON_CreateObject();
  cJSON_AddStringToObject(body, "device_id", device_id);
  cJSON_AddStringToObject(body, "app_version", desc ? desc->version : "");
  cJSON_AddStringToObject(body, "idf_version", desc ? desc->idf_ver : "");
  cJSON_AddNumberToObject(body, "free_heap", esp_get_free_heap_size());
  cJSON_AddNumberToObject(body, "uptime_s", (double)(esp_timer_get_time() / 1000000));
  cJSON_AddStringToObject(body, "tailscale_ip", tailscale_ip);
  cJSON_AddStringToObject(body, "local_ip", local_ip);
  cJSON_AddStringToObject(body, "state", app->tailscale_connected ? "CONNECTED" : (app->ml ? "CONNECTING" : "IDLE"));
  cJSON_AddBoolToObject(body, "rollback_occurred", invalid != NULL);

  char * json_str = cJSON_PrintUnformatted(body);
  cJSON_Delete(body);
  if (!json_str) return false;

  /* HTTP POST */
  char resp_buf[512];
  http_buf_t rb = {.buf = resp_buf, .len = 0, .cap = sizeof(resp_buf) - 1};

  char auth_hdr[100];
  snprintf(auth_hdr, sizeof(auth_hdr), "Bearer %s", api_key);

  esp_http_client_config_t http_cfg = {
    .url = url,
    .method = HTTP_METHOD_POST,
    .event_handler = http_event_handler,
    .user_data = &rb,
    .timeout_ms = 15000,
    .crt_bundle_attach = esp_crt_bundle_attach,
  };

  esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
  esp_http_client_set_header(client, "Content-Type", "application/json");
  esp_http_client_set_header(client, "Authorization", auth_hdr);
  esp_http_client_set_post_field(client, json_str, strlen(json_str));

  esp_err_t err = esp_http_client_perform(client);
  int status = esp_http_client_get_status_code(client);
  esp_http_client_cleanup(client);
  free(json_str);

  if (err != ESP_OK || status != 200) {
    ESP_LOGW(TAG, "OTA check-in failed: err=%s status=%d", esp_err_to_name(err), status);
    /* Couldn't reach the fleet — flag the link unhealthy so the fleet
         * peer's disco-first wake forces a fresh handshake (e.g. a stale
         * post-reboot session), and let the caller surface the real state. */
    microlink_notify_fleet_health(app->ml, false);
    return false;
  }
  /* Reached the fleet (HTTP 200). */
  if (out_reached) *out_reached = true;
  microlink_notify_fleet_health(app->ml, true);

  /* Parse response */
  cJSON * resp = cJSON_Parse(resp_buf);
  if (!resp) return false;

  /* Sync auto_update from backend (single source of truth) */
  cJSON * au = cJSON_GetObjectItem(resp, "auto_update");
  if (au && cJSON_IsBool(au)) {
    bool backend_val = cJSON_IsTrue(au);
    if (backend_val != app->auto_update) {
      app->auto_update = backend_val;
      ESP_LOGI(TAG, "OTA: auto_update synced from backend: %s", backend_val ? "ON" : "OFF");
    }
  }

  cJSON * avail = cJSON_GetObjectItem(resp, "update_available");
  if (!avail || !cJSON_IsTrue(avail)) {
    cJSON_Delete(resp);
    return false;
  }

  cJSON * fw_url = cJSON_GetObjectItem(resp, "firmware_url");
  cJSON * fw_sha = cJSON_GetObjectItem(resp, "firmware_sha256");
  cJSON * fw_size = cJSON_GetObjectItem(resp, "firmware_size");
  cJSON * fw_ver = cJSON_GetObjectItem(resp, "target_version");

  if (fw_url && cJSON_IsString(fw_url)) strncpy(out_url, fw_url->valuestring, url_size - 1);
  if (fw_sha && cJSON_IsString(fw_sha)) strncpy(out_sha256, fw_sha->valuestring, 64);
  if (fw_size && cJSON_IsNumber(fw_size)) *out_size = (int)fw_size->valuedouble;

  ESP_LOGI(
    TAG,
    "OTA: update available — version %s (%d bytes)",
    fw_ver && cJSON_IsString(fw_ver) ? fw_ver->valuestring : "?",
    *out_size);

  cJSON_Delete(resp);
  return true;
}

static bool fleet_ota_download_and_apply(
  ml_app_t * app, const char * fw_url, const char * expected_sha256, int expected_size)
{
  ESP_LOGI(TAG, "OTA: downloading firmware from %s", fw_url);

  const esp_partition_t * update = esp_ota_get_next_update_partition(NULL);
  if (!update) {
    ESP_LOGE(TAG, "OTA: no update partition found");
    return false;
  }

  ESP_LOGI(TAG, "OTA: target partition '%s' (%lu KB)", update->label, (unsigned long)update->size / 1024);

  esp_ota_handle_t ota_handle;
  esp_err_t err = esp_ota_begin(update, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "OTA: begin failed: %s", esp_err_to_name(err));
    return false;
  }

  /* SHA-256 context for verification */
  mbedtls_sha256_context sha_ctx;
  mbedtls_sha256_init(&sha_ctx);
  mbedtls_sha256_starts(&sha_ctx, 0);

  char auth_hdr[100];
  snprintf(auth_hdr, sizeof(auth_hdr), "Bearer %s", CONFIG_ML_OTA_API_KEY);

  esp_http_client_config_t http_cfg = {
    .url = fw_url,
    .timeout_ms = 30000,
    .buffer_size = 4096,
    .crt_bundle_attach = esp_crt_bundle_attach,
  };

  esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
  esp_http_client_set_header(client, "Authorization", auth_hdr);

  err = esp_http_client_open(client, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "OTA: HTTP open failed: %s", esp_err_to_name(err));
    esp_ota_abort(ota_handle);
    esp_http_client_cleanup(client);
    mbedtls_sha256_free(&sha_ctx);
    return false;
  }

  int content_len = esp_http_client_fetch_headers(client);
  if (content_len <= 0) {
    ESP_LOGE(TAG, "OTA: no content (status=%d)", esp_http_client_get_status_code(client));
    esp_ota_abort(ota_handle);
    esp_http_client_cleanup(client);
    mbedtls_sha256_free(&sha_ctx);
    return false;
  }

  ESP_LOGI(TAG, "OTA: receiving %d bytes...", content_len);

  char * buf = malloc(4096);
  if (!buf) {
    esp_ota_abort(ota_handle);
    esp_http_client_cleanup(client);
    mbedtls_sha256_free(&sha_ctx);
    return false;
  }

  int total = 0;
  int last_pct = -1;
  bool success = true;

  while (true) {
    int read_len = esp_http_client_read(client, buf, 4096);
    if (read_len < 0) {
      ESP_LOGE(TAG, "OTA: read error at %d bytes", total);
      success = false;
      break;
    }
    if (read_len == 0) break;

    err = esp_ota_write(ota_handle, buf, read_len);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "OTA: write failed at %d bytes: %s", total, esp_err_to_name(err));
      success = false;
      break;
    }

    mbedtls_sha256_update(&sha_ctx, (const unsigned char *)buf, read_len);
    total += read_len;

    int pct = content_len > 0 ? (total * 100 / content_len) : 0;
    app->ota_dl_percent = pct;
    if (pct / 10 != last_pct / 10) {
      last_pct = pct;
      ESP_LOGI(TAG, "OTA: %3d%%  (%d / %d bytes)", pct, total, content_len);
    }
  }

  free(buf);
  esp_http_client_close(client);
  esp_http_client_cleanup(client);

  if (!success) {
    esp_ota_abort(ota_handle);
    mbedtls_sha256_free(&sha_ctx);
    return false;
  }

  /* Verify SHA-256 */
  unsigned char hash[32];
  mbedtls_sha256_finish(&sha_ctx, hash);
  mbedtls_sha256_free(&sha_ctx);

  char hash_hex[65];
  for (int i = 0; i < 32; i++) snprintf(hash_hex + i * 2, 3, "%02x", hash[i]);

  if (expected_sha256[0] && strcmp(hash_hex, expected_sha256) != 0) {
    ESP_LOGE(TAG, "OTA: SHA-256 mismatch!");
    ESP_LOGE(TAG, "  expected: %s", expected_sha256);
    ESP_LOGE(TAG, "  got:      %s", hash_hex);
    esp_ota_abort(ota_handle);
    return false;
  }
  ESP_LOGI(TAG, "OTA: SHA-256 verified OK");

  /* Finalize */
  err = esp_ota_end(ota_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "OTA: image validation failed: %s", esp_err_to_name(err));
    return false;
  }

  err = esp_ota_set_boot_partition(update);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "OTA: set boot partition failed: %s", esp_err_to_name(err));
    return false;
  }

  ota_set_state(app, OTA_STATE_REBOOT, "Update successful, rebooting...");
  ESP_LOGI(TAG, "========== FLEET OTA UPDATE SUCCESSFUL ==========");
  ESP_LOGI(TAG, "OTA: %d bytes written to '%s', SHA-256 verified", total, update->label);
  ESP_LOGW(TAG, "OTA: rebooting in 3 seconds...");
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGW(TAG, "OTA: rebooting in 2 seconds...");
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGW(TAG, "OTA: rebooting in 1 second...");
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGW(TAG, "OTA: rebooting NOW");
  esp_restart();

  return true; /* unreachable */
}

/* Admin API handlers for fleet OTA (registered by ml_app on the shared httpd) */
static esp_err_t handler_fleet_ota_status(httpd_req_t * req)
{
  ml_app_t * app = (ml_app_t *)req->user_ctx;
  cJSON * json = cJSON_CreateObject();
  if (!json) return ESP_FAIL;

  const char * state_names[] = {"idle", "checking", "update_found", "downloading", "rebooting"};
  int st = app->ota_poll_state;
  if (st >= 0 && st <= 4)
    cJSON_AddStringToObject(json, "state", state_names[st]);
  else
    cJSON_AddStringToObject(json, "state", "error");
  cJSON_AddStringToObject(json, "message", app->ota_poll_msg);
  cJSON_AddBoolToObject(json, "auto_update", app->auto_update);
  cJSON_AddNumberToObject(json, "dl_percent", app->ota_dl_percent);
  cJSON_AddStringToObject(json, "backend_url", CONFIG_ML_OTA_BACKEND_URL);
  cJSON_AddNumberToObject(json, "check_interval_s", app->check_interval_s);
  cJSON_AddBoolToObject(json, "verbose", app->verbose_log);

  char * str = cJSON_PrintUnformatted(json);
  cJSON_Delete(json);
  if (!str) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, str);
  free(str);
  return ESP_OK;
}

static esp_err_t handler_fleet_ota_check(httpd_req_t * req)
{
  ml_app_t * app = (ml_app_t *)req->user_ctx;
  app->ota_check_now = true;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, "{\"ok\":true,\"message\":\"Check triggered\"}");
  return ESP_OK;
}

static esp_err_t handler_fleet_ota_toggle(httpd_req_t * req)
{
  ml_app_t * app = (ml_app_t *)req->user_ctx;
  bool new_val = !app->auto_update;

  /* Update backend (source of truth) */
  const char * backend = CONFIG_ML_OTA_BACKEND_URL;
  const char * api_key = CONFIG_ML_OTA_API_KEY;
  if (backend[0] && api_key[0]) {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char body[128];
    snprintf(
      body,
      sizeof(body),
      "{\"device_id\":\"%02x%02x%02x%02x%02x%02x\",\"auto_update\":%s}",
      mac[0],
      mac[1],
      mac[2],
      mac[3],
      mac[4],
      mac[5],
      new_val ? "true" : "false");

    char url[200];
    snprintf(url, sizeof(url), "%s/api/v1/auto_update", backend);
    char auth_hdr[100];
    snprintf(auth_hdr, sizeof(auth_hdr), "Bearer %s", api_key);

    esp_http_client_config_t cfg = {
      .url = url,
      .method = HTTP_METHOD_POST,
      .timeout_ms = 10000,
      .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Authorization", auth_hdr);
    esp_http_client_set_post_field(client, body, strlen(body));
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    if (err == ESP_OK && status == 200) {
      ESP_LOGI(TAG, "Fleet OTA: auto_update=%s synced to backend", new_val ? "ON" : "OFF");
    } else {
      ESP_LOGW(
        TAG, "Fleet OTA: failed to sync auto_update to backend (err=%s status=%d)", esp_err_to_name(err), status);
    }
  }

  app->auto_update = new_val;

  char resp[64];
  snprintf(resp, sizeof(resp), "{\"ok\":true,\"auto_update\":%s}", new_val ? "true" : "false");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, resp);
  return ESP_OK;
}

  #define NVS_NS_MLAPP "ml_app"
  #define NVS_KEY_INTERVAL "ota_intv"

static void save_interval_to_nvs(int interval_s)
{
  nvs_handle_t nvs;
  if (nvs_open(NVS_NS_MLAPP, NVS_READWRITE, &nvs) == ESP_OK) {
    nvs_set_i32(nvs, NVS_KEY_INTERVAL, interval_s);
    nvs_commit(nvs);
    nvs_close(nvs);
  }
}

static int load_interval_from_nvs(int default_val)
{
  nvs_handle_t nvs;
  int32_t val = default_val;
  if (nvs_open(NVS_NS_MLAPP, NVS_READONLY, &nvs) == ESP_OK) {
    nvs_get_i32(nvs, NVS_KEY_INTERVAL, &val);
    nvs_close(nvs);
  }
  if (val < 60 || val > 86400) val = default_val;
  return (int)val;
}

static esp_err_t handler_fleet_ota_interval(httpd_req_t * req)
{
  ml_app_t * app = (ml_app_t *)req->user_ctx;

  char buf[64];
  int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (len > 0) {
    buf[len] = '\0';
    cJSON * json = cJSON_Parse(buf);
    if (json) {
      cJSON * val = cJSON_GetObjectItem(json, "interval_s");
      if (val && cJSON_IsNumber(val)) {
        int s = (int)val->valuedouble;
        if (s >= 60 && s <= 86400) {
          app->check_interval_s = s;
          save_interval_to_nvs(s);
          ESP_LOGI(TAG, "Fleet OTA check interval set to %ds (saved to NVS)", s);
        }
      }
      cJSON_Delete(json);
    }
  }

  char resp[64];
  snprintf(resp, sizeof(resp), "{\"ok\":true,\"check_interval_s\":%d}", app->check_interval_s);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, resp);
  return ESP_OK;
}

/* wireguard-lwip bare printf control */
extern volatile int wg_verbose_logging;

static void apply_log_level(ml_app_t * app)
{
  bool verbose = app->verbose_log;
  wg_verbose_logging = verbose ? 1 : 0;
  esp_log_level_t base = verbose ? ESP_LOG_INFO : ESP_LOG_WARN;

  /* Always show ml_app messages (OTA status, critical events) */
  esp_log_level_set("ml_app", ESP_LOG_INFO);
  esp_log_level_set("ip_blink", ESP_LOG_INFO);

  /* Base level for everything else */
  esp_log_level_set("ml_config", base);
  esp_log_level_set("wifi", base);
  esp_log_level_set("wifi_init", base);
  esp_log_level_set("esp_netif_handlers", base);
  esp_log_level_set("main", base);
  esp_log_level_set("racer", base);

  /* MicroLink protocol tags — respect debug checkboxes if set */
  /* Debug flags: bit0=DISCO, bit1=WG, bit2=DERP, bit3=COORD */
  uint8_t dbg = 0;
  #ifdef CONFIG_ML_ENABLE_CONFIG_HTTPD
  if (app->ml && app->ml->config_httpd) {
    dbg = ml_config_get_debug_flags(app->ml->config_httpd);
  }
  #endif
  esp_log_level_set("ml_wg_mgr", (dbg & 2) ? ESP_LOG_DEBUG : base);
  esp_log_level_set("ml_net_io", (dbg & 1) ? ESP_LOG_DEBUG : base);
  esp_log_level_set("ml_derp", (dbg & 4) ? ESP_LOG_DEBUG : base);
  esp_log_level_set("ml_coord", (dbg & 8) ? ESP_LOG_DEBUG : base);
  esp_log_level_set("ml_stun", (dbg & 1) ? ESP_LOG_DEBUG : base);
  esp_log_level_set("ml_noise", base);
  esp_log_level_set("ml_h2", (dbg & 8) ? ESP_LOG_DEBUG : base);
  esp_log_level_set("ml_peer_nvs", base);

  ESP_LOGI(TAG, "Logging: %s (debug_flags=0x%02x)", verbose ? "VERBOSE" : "QUIET", dbg);
}

static esp_err_t handler_verbose_toggle(httpd_req_t * req)
{
  ml_app_t * app = (ml_app_t *)req->user_ctx;
  app->verbose_log = !app->verbose_log;
  apply_log_level(app);

  char resp[64];
  snprintf(resp, sizeof(resp), "{\"ok\":true,\"verbose\":%s}", app->verbose_log ? "true" : "false");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, resp);
  return ESP_OK;
}

static esp_err_t handler_verbose_status(httpd_req_t * req)
{
  ml_app_t * app = (ml_app_t *)req->user_ctx;
  /* Re-apply log levels (picks up any debug flag changes from settings save) */
  apply_log_level(app);
  char resp[32];
  snprintf(resp, sizeof(resp), "{\"verbose\":%s}", app->verbose_log ? "true" : "false");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, resp);
  return ESP_OK;
}

static void fleet_ota_task(void * arg)
{
  ml_app_t * app = (ml_app_t *)arg;

  ESP_LOGI(
    TAG, "Fleet OTA task started (interval: %ds, backend: %s)", app->check_interval_s, CONFIG_ML_OTA_BACKEND_URL);
  ota_set_state(app, OTA_STATE_IDLE, "Waiting for Tailscale...");

  /* Wait for Tailscale connection before first check */
  while (!app->tailscale_connected) {
    vTaskDelay(pdMS_TO_TICKS(5000));
  }

  /* Initial delay after connect to let things settle */
  ota_set_state(app, OTA_STATE_IDLE, "Connected, waiting for first check...");
  vTaskDelay(pdMS_TO_TICKS(10000));

  while (true) {
    /* Wait for interval, break on manual trigger or interval change */
    int interval_s = app->check_interval_s;
    for (int waited = 0; waited < interval_s; waited++) {
      if (app->ota_check_now) break;
      if (app->check_interval_s != interval_s) break; /* interval changed */
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    app->ota_check_now = false;

    ota_set_state(app, OTA_STATE_CHECKING, "Checking backend...");
    ESP_LOGI(TAG, "OTA: checking in with backend...");

    char fw_url[256] = "";
    char fw_sha256[65] = "";
    int fw_size = 0;

    /* Always check in (syncs auto_update flag from backend).
         * Only download if auto_update is enabled. */
    bool reached = false;
    bool update_available = fleet_ota_checkin(app, fw_url, sizeof(fw_url), fw_sha256, &fw_size, &reached);

    if (!reached) {
      /* Distinguish an unreachable fleet from "no update" — the old code
             * showed "Up to date" for both, hiding a silently-unreachable
             * backend. */
      ota_set_state(app, OTA_STATE_ERROR, "Check-in failed (fleet unreachable)");
      ESP_LOGW(TAG, "OTA: check-in did not reach the fleet backend");
      continue;
    }

    if (!app->auto_update) {
      ota_set_state(app, OTA_STATE_IDLE, "Auto-update disabled");
      ESP_LOGI(TAG, "OTA: auto-update disabled, skipping download");
      continue;
    }

    if (update_available) {
      ota_set_state(app, OTA_STATE_FOUND, "Update found, downloading...");
      ESP_LOGI(TAG, "OTA: downloading and applying update...");

      bool ok = false;
      for (int attempt = 1; attempt <= 3; attempt++) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Downloading... (attempt %d/3)", attempt);
        ota_set_state(app, OTA_STATE_DOWNLOAD, msg);
        app->ota_dl_percent = 0;

        if (fleet_ota_download_and_apply(app, fw_url, fw_sha256, fw_size)) {
          ok = true;
          break; /* reboots inside, never reaches here */
        }

        ESP_LOGW(TAG, "OTA: attempt %d/3 failed", attempt);
        if (attempt < 3) {
          ota_set_state(app, OTA_STATE_ERROR, "Download failed, retrying in 5s...");
          vTaskDelay(pdMS_TO_TICKS(5000));
        }
      }
      if (!ok) {
        ota_set_state(app, OTA_STATE_ERROR, "Update failed after 3 attempts");
        ESP_LOGE(TAG, "OTA: update failed after 3 attempts, will retry next cycle");
      }
      /* If download_and_apply succeeds, it reboots — we never reach here */
    } else {
      ota_set_state(app, OTA_STATE_IDLE, "Up to date");
      ESP_LOGI(TAG, "OTA: no update available");
    }
  }
}

#endif /* CONFIG_ML_OTA_BACKEND_URL && CONFIG_ML_OTA_API_KEY */

/* ============================================================================
 * Public API
 * ========================================================================== */

ml_app_t * ml_app_start(const ml_app_config_t * cfg)
{
  ml_app_t * app = calloc(1, sizeof(ml_app_t));
  if (!app) {
    ESP_LOGE(TAG, "Failed to allocate app context");
    return NULL;
  }
  s_app = app;

  /* --- NVS init --- */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "MicroLink App Framework starting...");
  ESP_LOGI(
    TAG,
    "Free heap: %lu bytes (PSRAM: %lu bytes)",
    (unsigned long)esp_get_free_heap_size(),
    (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

  /* --- Load WiFi credentials (NVS -> Kconfig fallback) --- */
  strncpy(app->wifi_ssid, CONFIG_ML_WIFI_SSID, sizeof(app->wifi_ssid) - 1);
  strncpy(app->wifi_password, CONFIG_ML_WIFI_PASSWORD, sizeof(app->wifi_password) - 1);

  memset(&app->wifi_list, 0, sizeof(app->wifi_list));
  app->wifi_list.active_idx = 0xFF;

  if (ml_config_get_wifi_list(&app->wifi_list) && app->wifi_list.count > 1) {
    app->wifi_list_count = app->wifi_list.count;
    app->current_wifi_idx = 0;
    strncpy(app->wifi_ssid, app->wifi_list.entries[0].ssid, sizeof(app->wifi_ssid) - 1);
    strncpy(app->wifi_password, app->wifi_list.entries[0].pass, sizeof(app->wifi_password) - 1);
    ESP_LOGI(TAG, "WiFi multi-SSID: %d networks (first: %s)", app->wifi_list_count, app->wifi_ssid);
  } else if (ml_config_get_nvs_wifi(
               app->wifi_ssid, sizeof(app->wifi_ssid), app->wifi_password, sizeof(app->wifi_password)))
  {
    ESP_LOGI(TAG, "WiFi from NVS: %s", app->wifi_ssid);
  } else {
    ESP_LOGI(TAG, "WiFi from Kconfig: %s", app->wifi_ssid);
  }

  /* Seed the NVS multi-SSID list from Kconfig defaults if it's empty AND
     * we actually have a secondary SSID configured. This means a chip with
     * fresh NVS (post-erase-flash) gets BOTH networks cycled through
     * automatically — no user setup needed before AP fallback kicks in.
     * Idempotent: only runs when the NVS wifi_list has 1 entry or fewer.
     * Once the user provisions via admin UI, that list wins.
     *
     * Heap-alloc the seed struct (~1.5 KB) — putting it on the stack
     * overflows app_main's task stack (default 3.5 KB).
     */
  if (app->wifi_list_count <= 1 && strlen(CONFIG_ML_WIFI_SSID) > 0 && strlen(CONFIG_ML_WIFI_SSID_2) > 0) {
    ml_config_wifi_list_t * seed = calloc(1, sizeof(*seed));
    if (seed) {
      seed->count = 2;
      seed->active_idx = 0xFF;
      strncpy(seed->entries[0].ssid, CONFIG_ML_WIFI_SSID, sizeof(seed->entries[0].ssid) - 1);
      strncpy(seed->entries[0].pass, CONFIG_ML_WIFI_PASSWORD, sizeof(seed->entries[0].pass) - 1);
      strncpy(seed->entries[1].ssid, CONFIG_ML_WIFI_SSID_2, sizeof(seed->entries[1].ssid) - 1);
      strncpy(seed->entries[1].pass, CONFIG_ML_WIFI_PASSWORD_2, sizeof(seed->entries[1].pass) - 1);

      nvs_handle_t h;
      if (nvs_open("ml_config", NVS_READWRITE, &h) == ESP_OK) {
        size_t save_len = 2 + seed->count * sizeof(ml_config_wifi_entry_t);
        if (nvs_set_blob(h, "wifi_list", seed, save_len) == ESP_OK) {
          nvs_commit(h);
          ESP_LOGI(
            TAG,
            "WiFi: seeded NVS list with 2 networks from Kconfig (%s + %s)",
            seed->entries[0].ssid,
            seed->entries[1].ssid);
          app->wifi_list = *seed;
          app->wifi_list_count = 2;
          app->current_wifi_idx = 0;
          strncpy(app->wifi_ssid, seed->entries[0].ssid, sizeof(app->wifi_ssid) - 1);
          strncpy(app->wifi_password, seed->entries[0].pass, sizeof(app->wifi_password) - 1);
        }
        nvs_close(h);
      }
      free(seed);
    }
  }

  /* --- Optional: try alternative network first (e.g. USB-CDC-NCM tether)
     * --- If the caller registered a try_alt_network() hook and it returns
     * ESP_OK within its timeout, skip WiFi entirely and run on that netif. */
  bool used_alt_network = false;
  if (cfg->try_alt_network) {
    uint32_t to_ms = cfg->alt_network_timeout_ms ? cfg->alt_network_timeout_ms : 5000;
    ESP_LOGI(TAG, "Trying alternative network (timeout %lu ms)...", (unsigned long)to_ms);
    esp_err_t alt = cfg->try_alt_network(to_ms);
    if (alt == ESP_OK) {
      ESP_LOGI(TAG, "Alternative network up — skipping WiFi");
      used_alt_network = true;
    } else {
      ESP_LOGI(TAG, "Alternative network not available (%s) — falling back to WiFi", esp_err_to_name(alt));
    }
  }

  /* --- WiFi init + connect (blocks until connected). Skipped if alt
     * network already brought up an internet-capable default route. */
  if (!used_alt_network) {
    wifi_init(app);
    xEventGroupWaitBits(app->wifi_events, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
  }

  /* --- Create HTTP server (accessible immediately on local WiFi) --- */
  httpd_config_t http_cfg = HTTPD_DEFAULT_CONFIG();
  http_cfg.stack_size = 10240;
  http_cfg.max_uri_handlers = 16 + (cfg->max_user_uri_handlers ? cfg->max_user_uri_handlers : 8);
  http_cfg.uri_match_fn = httpd_uri_match_wildcard;
  http_cfg.recv_wait_timeout = 30;
  http_cfg.task_priority = cfg->http_priority ? cfg->http_priority : 10;
  if (cfg->http_port) http_cfg.server_port = cfg->http_port;
  if (cfg->http_max_sockets) http_cfg.max_open_sockets = cfg->http_max_sockets;
  /* Purge the least-recently-used connection instead of refusing new ones
     * when the socket cap is hit — browsers hold idle keep-alives that would
     * otherwise starve fresh requests. */
  http_cfg.lru_purge_enable = true;

  ret = httpd_start(&app->httpd, &http_cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
    free(app);
    return NULL;
  }
  ESP_LOGI(TAG, "HTTP server started on port %d", http_cfg.server_port);

  /* --- MicroLink init --- */
  uint8_t max_peers = cfg->max_peers ? cfg->max_peers : CONFIG_ML_MAX_PEERS;
  microlink_config_t ml_cfg = {
    .auth_key = CONFIG_ML_TAILSCALE_AUTH_KEY,
    .device_name = (cfg->device_name && cfg->device_name[0]) ? cfg->device_name : CONFIG_ML_DEVICE_NAME,
    .enable_derp = cfg->enable_derp,
    .enable_stun = cfg->enable_stun,
    .enable_disco = cfg->enable_disco,
    .max_peers = max_peers,
    .wifi_tx_power_dbm = cfg->wifi_tx_power_dbm,
  };

  app->ml = microlink_init(&ml_cfg);
  if (!app->ml) {
    ESP_LOGE(TAG, "Failed to initialize MicroLink");
    httpd_stop(app->httpd);
    free(app);
    return NULL;
  }

  /* --- Register admin routes at /admin/ with auth --- */
#ifdef CONFIG_ML_ENABLE_CONFIG_HTTPD
  ml_config_httpd_register_routes(app->ml->config_httpd, NULL, app->httpd, "/admin");
  #ifdef CONFIG_ML_ADMIN_PASSWORD
  ml_config_httpd_set_auth(app->ml->config_httpd, admin_auth_check);
  ESP_LOGI(TAG, "Admin panel at /admin/ (password protected)");
  #else
  ESP_LOGI(TAG, "Admin panel at /admin/ (no password)");
  #endif
#endif

  /* --- Set callbacks and start MicroLink (non-blocking) --- */
  microlink_set_state_callback(app->ml, ml_state_callback, app);
  microlink_set_peer_callback(app->ml, ml_peer_callback, app);
  ESP_ERROR_CHECK(microlink_start(app->ml));

  /* --- Start fleet OTA poll task --- */
#if defined(CONFIG_ML_OTA_BACKEND_URL) && defined(CONFIG_ML_OTA_API_KEY)
  if (strlen(CONFIG_ML_OTA_BACKEND_URL) > 0 && strlen(CONFIG_ML_OTA_API_KEY) > 0) {
    app->auto_update = true;
    app->check_interval_s = load_interval_from_nvs(CONFIG_ML_OTA_CHECK_INTERVAL_S);
  #ifdef CONFIG_ML_OTA_AUTO_UPDATE
    app->auto_update = CONFIG_ML_OTA_AUTO_UPDATE;
  #endif
    xTaskCreatePinnedToCore(fleet_ota_task, "ml_ota", 8192, app, 3, NULL, 1);

    /* Register fleet OTA admin endpoints */
    const httpd_uri_t fleet_uris[] = {
      {.uri = "/admin/api/fleet-ota/status", .method = HTTP_GET, .handler = handler_fleet_ota_status, .user_ctx = app},
      {.uri = "/admin/api/fleet-ota/check", .method = HTTP_POST, .handler = handler_fleet_ota_check, .user_ctx = app},
      {.uri = "/admin/api/fleet-ota/toggle", .method = HTTP_POST, .handler = handler_fleet_ota_toggle, .user_ctx = app},
      {.uri = "/admin/api/fleet-ota/interval",
       .method = HTTP_POST,
       .handler = handler_fleet_ota_interval,
       .user_ctx = app},
    };
    for (int i = 0; i < 4; i++) {
      httpd_register_uri_handler(app->httpd, &fleet_uris[i]);
    }
  }
#endif

  /* --- Register verbose log toggle (always available) --- */
  {
    const httpd_uri_t verbose_uris[] = {
      {.uri = "/admin/api/verbose", .method = HTTP_GET, .handler = handler_verbose_status, .user_ctx = app},
      {.uri = "/admin/api/verbose/toggle", .method = HTTP_POST, .handler = handler_verbose_toggle, .user_ctx = app},
    };
    for (int i = 0; i < 2; i++) {
      httpd_register_uri_handler(app->httpd, &verbose_uris[i]);
    }
  }

  /* Default to quiet mode */
  app->verbose_log = false;
  apply_log_level(app);

  ESP_LOGI(TAG, "App framework ready — Tailscale connecting in background");
  return app;
}

esp_err_t ml_app_add_page(ml_app_t * app, const char * uri, httpd_method_t method, esp_err_t (*handler)(httpd_req_t *))
{
  if (!app || !app->httpd || !uri || !handler) return ESP_ERR_INVALID_ARG;

  httpd_uri_t uri_def = {
    .uri = uri,
    .method = method,
    .handler = handler,
    .user_ctx = NULL,
  };
  return httpd_register_uri_handler(app->httpd, &uri_def);
}

esp_err_t ml_app_on_connected(ml_app_t * app, ml_app_connected_cb_t cb, void * user_data)
{
  if (!app) return ESP_ERR_INVALID_ARG;
  app->connected_cb = cb;
  app->connected_cb_data = user_data;
  return ESP_OK;
}

httpd_handle_t ml_app_get_httpd(const ml_app_t * app)
{
  return app ? app->httpd : NULL;
}

microlink_t * ml_app_get_microlink(const ml_app_t * app)
{
  return app ? app->ml : NULL;
}
