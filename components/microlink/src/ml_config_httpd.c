/**
 * @file ml_config_httpd.c
 * @brief MicroLink HTTP Config Server
 *
 * Runtime configuration via web UI served on the Tailscale VPN IP.
 * Settings persist in NVS namespace "ml_config".
 * Peer allowlist controls DISCO probe filtering in wg_mgr.
 *
 * Gated by CONFIG_ML_ENABLE_CONFIG_HTTPD.
 */

#include "sdkconfig.h"

#ifdef CONFIG_ML_ENABLE_CONFIG_HTTPD

#include "ml_config_httpd.h"
#include "microlink.h"
#include "microlink_internal.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "soc/soc_caps.h"
#if SOC_TEMP_SENSOR_SUPPORTED
#include "driver/temperature_sensor.h"
#endif
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include <string.h>
#ifdef CONFIG_ML_ENABLE_CELLULAR
#include "ml_at_socket.h"
#endif

static const char *TAG = "ml_config";

/* Embedded HTML page */
#include "ml_config_html.h"

/* NVS namespace and keys */
#define NVS_NAMESPACE    "ml_config"
#define NVS_KEY_SETTINGS "settings"
#define NVS_KEY_PEERS    "peers"
#define NVS_KEY_WIFI     "wifi_list"

/* ============================================================================
 * Context Structure
 * ========================================================================== */

struct ml_config_ctx {
    /* NVS handle */
    nvs_handle_t nvs;

    /* Settings (loaded at init, updated via HTTP) */
    ml_config_settings_t settings;

    /* Peer allowlist (read by wg_mgr, written by httpd) */
    SemaphoreHandle_t peer_mutex;
    ml_config_peer_list_t peer_list;
    volatile bool filter_enabled;

    /* WiFi multi-SSID list */
    ml_config_wifi_list_t wifi_list;

#if SOC_TEMP_SENSOR_SUPPORTED
    /* Temperature sensor (not present on original ESP32) */
    temperature_sensor_handle_t temp_sensor;
#endif

    /* HTTP server */
    httpd_handle_t httpd;
    bool httpd_external;

    /* Auth callback (set by ml_app, NULL = no auth) */
    bool (*auth_check)(httpd_req_t *req);

    /* Backpointer */
    microlink_t *ml;
};

/* ============================================================================
 * NVS Load/Save
 * ========================================================================== */

static void config_save_peers(ml_config_ctx_t *ctx);  /* forward decl for migration */

static void config_load_settings(ml_config_ctx_t *ctx) {
    /* Read whatever size blob exists — v1 blobs are smaller than v2 struct */
    memset(&ctx->settings, 0, sizeof(ml_config_settings_t));
    size_t len = sizeof(ml_config_settings_t);
    esp_err_t err = nvs_get_blob(ctx->nvs, NVS_KEY_SETTINGS, &ctx->settings, &len);

    if (err == ESP_OK && ctx->settings.version >= 1) {
        /* v1 blob is smaller — v2 fields will be zero (defaults) */
        if (ctx->settings.version < ML_CONFIG_SETTINGS_VERSION) {
            ESP_LOGI(TAG, "Migrating settings v%d -> v%d",
                     ctx->settings.version, ML_CONFIG_SETTINGS_VERSION);
            ctx->settings.version = ML_CONFIG_SETTINGS_VERSION;
            /* Save migrated blob so next boot loads full size */
            nvs_set_blob(ctx->nvs, NVS_KEY_SETTINGS, &ctx->settings,
                         sizeof(ml_config_settings_t));
            nvs_commit(ctx->nvs);
        }
    } else {
        memset(&ctx->settings, 0, sizeof(ctx->settings));
        ctx->settings.version = ML_CONFIG_SETTINGS_VERSION;
        ESP_LOGI(TAG, "No saved settings in NVS");
    }

    /* Seed empty fields from Kconfig compile-time defaults.
     * This ensures the web UI displays sdkconfig values even before
     * the user saves anything via the web interface. */
    #define SEED_STR(field, kconfig) do { \
        if ((field)[0] == '\0' && strlen(kconfig) > 0) { \
            strncpy((field), (kconfig), sizeof(field) - 1); \
        } \
    } while (0)

    SEED_STR(ctx->settings.wifi_ssid,    CONFIG_ML_WIFI_SSID);
    SEED_STR(ctx->settings.wifi_pass,    CONFIG_ML_WIFI_PASSWORD);
    SEED_STR(ctx->settings.auth_key,     CONFIG_ML_TAILSCALE_AUTH_KEY);
    SEED_STR(ctx->settings.device_prefix, CONFIG_ML_DEVICE_NAME);
    if (ctx->settings.priority_peer_ip == 0 && strlen(CONFIG_ML_PRIORITY_PEER_IP) > 0) {
        ctx->settings.priority_peer_ip = microlink_parse_ip(CONFIG_ML_PRIORITY_PEER_IP);
    }

    #undef SEED_STR

    ESP_LOGI(TAG, "Settings v%d (wifi=%s, prefix=%s, name=%s, max_peers=%d, ctrl=%s, pip=%s)",
             ctx->settings.version,
             ctx->settings.wifi_ssid[0] ? ctx->settings.wifi_ssid : "(empty)",
             ctx->settings.device_prefix[0] ? ctx->settings.device_prefix : "(empty)",
             ctx->settings.device_name_full[0] ? ctx->settings.device_name_full : "(auto)",
             ctx->settings.max_peers,
             ctx->settings.ctrl_host[0] ? ctx->settings.ctrl_host : "(tailscale)",
             strlen(CONFIG_ML_PRIORITY_PEER_IP) > 0 ? CONFIG_ML_PRIORITY_PEER_IP : "(none)");
}

static void config_save_settings(ml_config_ctx_t *ctx) {
    esp_err_t err = nvs_set_blob(ctx->nvs, NVS_KEY_SETTINGS, &ctx->settings,
                                  sizeof(ml_config_settings_t));
    if (err == ESP_OK) {
        nvs_commit(ctx->nvs);
        ESP_LOGI(TAG, "Settings saved to NVS");
    } else {
        ESP_LOGE(TAG, "Failed to save settings: %s", esp_err_to_name(err));
    }
}

static void config_load_peers(ml_config_ctx_t *ctx) {
    /* First, query the stored blob size without reading */
    size_t stored_len = 0;
    esp_err_t err = nvs_get_blob(ctx->nvs, NVS_KEY_PEERS, NULL, &stored_len);

    if (err == ESP_OK && stored_len > 0) {
        /* Old format used uint8_t count (1 byte), new format uses uint16_t (2 bytes).
         * Detect old format by checking if stored size matches old layout:
         * old = 1 + 28*N, new = 2 + 28*N. If (stored_len - 1) % 28 == 0 and
         * (stored_len - 2) % 28 != 0, it's the old format. */
        bool is_old_format = (stored_len >= 1) &&
                             ((stored_len - 1) % sizeof(ml_config_peer_entry_t) == 0) &&
                             ((stored_len - 2) % sizeof(ml_config_peer_entry_t) != 0);

        if (is_old_format) {
            /* Read old format into temp buffer, migrate to new struct */
            uint8_t *tmp = malloc(stored_len);
            if (tmp) {
                size_t rlen = stored_len;
                err = nvs_get_blob(ctx->nvs, NVS_KEY_PEERS, tmp, &rlen);
                if (err == ESP_OK) {
                    uint8_t old_count = tmp[0];
                    uint16_t count = (old_count <= ML_CONFIG_MAX_ALLOWED_PEERS) ? old_count : 0;
                    memset(&ctx->peer_list, 0, sizeof(ctx->peer_list));
                    ctx->peer_list.count = count;
                    if (count > 0) {
                        memcpy(ctx->peer_list.entries, tmp + 1,
                               count * sizeof(ml_config_peer_entry_t));
                    }
                    ESP_LOGI(TAG, "Migrated peer allowlist from v1 (uint8 count=%d)", count);
                    /* Re-save in new format */
                    config_save_peers(ctx);
                }
                free(tmp);
            }
        } else {
            /* New format: blob may be smaller than struct (only used entries stored).
             * Zero the struct first, then read up to stored_len bytes into it. */
            memset(&ctx->peer_list, 0, sizeof(ctx->peer_list));
            size_t len = stored_len;
            if (len > sizeof(ml_config_peer_list_t)) len = sizeof(ml_config_peer_list_t);
            err = nvs_get_blob(ctx->nvs, NVS_KEY_PEERS, &ctx->peer_list, &len);
            if (err != ESP_OK || ctx->peer_list.count > ML_CONFIG_MAX_ALLOWED_PEERS) {
                memset(&ctx->peer_list, 0, sizeof(ctx->peer_list));
            }
        }

        ctx->filter_enabled = (ctx->peer_list.count > 0);
        ESP_LOGI(TAG, "Peer allowlist loaded: %d peers (filter %s)",
                 ctx->peer_list.count, ctx->filter_enabled ? "ON" : "OFF");
    } else {
        memset(&ctx->peer_list, 0, sizeof(ctx->peer_list));
        ctx->filter_enabled = false;
        ESP_LOGI(TAG, "No saved peer allowlist, filter OFF (all peers allowed)");
    }
}

static void config_save_peers(ml_config_ctx_t *ctx) {
    /* Only write count + actual entries (not the full 512-entry array) to save NVS space.
     * This reduces blob from ~14KB to just count*28+2 bytes. */
    size_t save_len = sizeof(ctx->peer_list.count) +
                      ctx->peer_list.count * sizeof(ml_config_peer_entry_t);
    esp_err_t err = nvs_set_blob(ctx->nvs, NVS_KEY_PEERS, &ctx->peer_list, save_len);
    if (err == ESP_OK) {
        nvs_commit(ctx->nvs);
        ESP_LOGI(TAG, "Peer allowlist saved (%d peers, %d bytes)", ctx->peer_list.count, (int)save_len);
    } else {
        ESP_LOGE(TAG, "Failed to save peers (%d bytes): %s", (int)save_len, esp_err_to_name(err));
    }
}

/* ============================================================================
 * WiFi List NVS Load/Save
 * ========================================================================== */

static void config_load_wifi_list(ml_config_ctx_t *ctx) {
    memset(&ctx->wifi_list, 0, sizeof(ctx->wifi_list));
    ctx->wifi_list.active_idx = 0xFF;

    size_t stored_len = 0;
    esp_err_t err = nvs_get_blob(ctx->nvs, NVS_KEY_WIFI, NULL, &stored_len);
    if (err == ESP_OK && stored_len >= 2) {
        size_t read_len = stored_len;
        if (read_len > sizeof(ml_config_wifi_list_t)) read_len = sizeof(ml_config_wifi_list_t);
        err = nvs_get_blob(ctx->nvs, NVS_KEY_WIFI, &ctx->wifi_list, &read_len);
        if (err != ESP_OK || ctx->wifi_list.count > ML_CONFIG_MAX_WIFI_ENTRIES) {
            memset(&ctx->wifi_list, 0, sizeof(ctx->wifi_list));
            ctx->wifi_list.active_idx = 0xFF;
        }
        ESP_LOGI(TAG, "WiFi list loaded: %d entries", ctx->wifi_list.count);
    } else {
        /* No wifi_list key — migrate from settings blob if it has WiFi creds */
        if (ctx->settings.wifi_ssid[0] != '\0') {
            ctx->wifi_list.count = 1;
            ctx->wifi_list.active_idx = 0;
            strncpy(ctx->wifi_list.entries[0].ssid, ctx->settings.wifi_ssid, 32);
            strncpy(ctx->wifi_list.entries[0].pass, ctx->settings.wifi_pass, 64);
            ESP_LOGI(TAG, "WiFi list: migrated from settings (%s)", ctx->settings.wifi_ssid);
        } else {
            ESP_LOGI(TAG, "WiFi list: empty (no saved networks)");
        }
    }
}

static void config_save_wifi_list(ml_config_ctx_t *ctx) {
    /* Variable-size save: header (2 bytes) + count * entry_size */
    size_t save_len = 2 + ctx->wifi_list.count * sizeof(ml_config_wifi_entry_t);
    esp_err_t err = nvs_set_blob(ctx->nvs, NVS_KEY_WIFI, &ctx->wifi_list, save_len);
    if (err == ESP_OK) {
        nvs_commit(ctx->nvs);
        ESP_LOGI(TAG, "WiFi list saved (%d entries, %d bytes)",
                 ctx->wifi_list.count, (int)save_len);
    } else {
        ESP_LOGE(TAG, "Failed to save WiFi list (%d bytes): %s",
                 (int)save_len, esp_err_to_name(err));
    }
}

/* ============================================================================
 * Peer Filter API (thread-safe, called from wg_mgr)
 * ========================================================================== */

bool ml_config_allowlist_active(const ml_config_ctx_t *ctx) {
    return ctx && ctx->filter_enabled;
}

bool ml_config_peer_is_allowed(const ml_config_ctx_t *ctx, uint32_t vpn_ip) {
    if (!ctx) return true;

    /* Fast path: filter disabled → allow all */
    if (!ctx->filter_enabled) return true;

    /* Slow path: check allowlist under mutex */
    bool allowed = false;
    ml_config_ctx_t *mctx = (ml_config_ctx_t *)ctx;  /* cast away const for mutex */
    if (xSemaphoreTake(mctx->peer_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        for (int i = 0; i < mctx->peer_list.count; i++) {
            if (mctx->peer_list.entries[i].vpn_ip == vpn_ip) {
                allowed = true;
                break;
            }
        }
        xSemaphoreGive(mctx->peer_mutex);
    } else {
        /* Mutex timeout: fail open */
        allowed = true;
    }
    return allowed;
}

/* ============================================================================
 * Settings Query API
 * ========================================================================== */

const char *ml_config_get_wifi_ssid(const ml_config_ctx_t *ctx) {
    if (!ctx || ctx->settings.wifi_ssid[0] == '\0') return NULL;
    return ctx->settings.wifi_ssid;
}

const char *ml_config_get_wifi_pass(const ml_config_ctx_t *ctx) {
    if (!ctx || ctx->settings.wifi_pass[0] == '\0') return NULL;
    return ctx->settings.wifi_pass;
}

const char *ml_config_get_auth_key(const ml_config_ctx_t *ctx) {
    if (!ctx || ctx->settings.auth_key[0] == '\0') return NULL;
    return ctx->settings.auth_key;
}

const char *ml_config_get_device_prefix(const ml_config_ctx_t *ctx) {
    if (!ctx || ctx->settings.device_prefix[0] == '\0') return NULL;
    return ctx->settings.device_prefix;
}

uint8_t ml_config_get_max_peers(const ml_config_ctx_t *ctx) {
    return ctx ? ctx->settings.max_peers : 0;
}

uint16_t ml_config_get_disco_heartbeat_ms(const ml_config_ctx_t *ctx) {
    return ctx ? ctx->settings.disco_heartbeat_ms : 0;
}

uint32_t ml_config_get_priority_peer_ip(const ml_config_ctx_t *ctx) {
    return ctx ? ctx->settings.priority_peer_ip : 0;
}

const char *ml_config_get_ctrl_host(const ml_config_ctx_t *ctx) {
    if (!ctx || ctx->settings.ctrl_host[0] == '\0') return NULL;
    return ctx->settings.ctrl_host;
}

uint8_t ml_config_get_debug_flags(const ml_config_ctx_t *ctx) {
    return ctx ? ctx->settings.debug_flags : 0;
}

const char *ml_config_get_device_name_full(const ml_config_ctx_t *ctx) {
    if (!ctx || ctx->settings.device_name_full[0] == '\0') return NULL;
    return ctx->settings.device_name_full;
}

/* ============================================================================
 * Auth check macro — returns 401 if auth callback is set and fails
 * ========================================================================== */

#define CHECK_AUTH(req) do { \
    ml_config_ctx_t *_ctx = (ml_config_ctx_t *)(req)->user_ctx; \
    if (_ctx && _ctx->auth_check && !_ctx->auth_check(req)) { \
        httpd_resp_set_status(req, "401 Unauthorized"); \
        httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"MicroLink Admin\""); \
        httpd_resp_sendstr(req, "401 Unauthorized"); \
        return ESP_FAIL; \
    } \
} while (0)

/* ============================================================================
 * HTTP Handlers
 * ========================================================================== */

/* Helper: send JSON response */
static esp_err_t send_json(httpd_req_t *req, cJSON *json) {
    char *str = cJSON_PrintUnformatted(json);
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

/* Helper: read POST body into buffer */
static char *read_post_body(httpd_req_t *req) {
    int content_len = req->content_len;
    if (content_len <= 0 || content_len > 4096) {
        return NULL;
    }
    char *buf = malloc(content_len + 1);
    if (!buf) return NULL;

    int received = 0;
    while (received < content_len) {
        int ret = httpd_req_recv(req, buf + received, content_len - received);
        if (ret <= 0) {
            free(buf);
            return NULL;
        }
        received += ret;
    }
    buf[content_len] = '\0';
    return buf;
}

/* GET / — serve HTML config page */
/* True from esp_ota_begin() until the post-flash reboot (stays true on
 * success — the device restarts moments later). Product firmware can poll
 * this to render an "updating" screen: flash writes suspend the instruction
 * cache, so any animation stutters badly during an OTA anyway. */
static volatile bool s_ota_in_progress = false;

bool ml_config_ota_in_progress(void) {
    return s_ota_in_progress;
}

static esp_err_t handler_root(httpd_req_t *req) {
    CHECK_AUTH(req);
    /* Heap-watermark guard: this is the one big transfer the device serves
     * (~32 KB inline HTML). Starting it with a depleted heap risks the
     * OOM regime (lwIP send-buffer allocs stealing the last blocks from
     * mbedTLS/coord). Tell the browser to come back in a moment instead —
     * a 503 with Retry-After is a fast, cheap response. */
    if (esp_get_free_heap_size() < 16 * 1024) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_hdr(req, "Retry-After", "3");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Low memory, retry shortly");
        return ESP_OK;
    }
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, CONFIG_PAGE_HTML);
    return ESP_OK;
}

/* GET /api/status */
static esp_err_t handler_status(httpd_req_t *req) {
    CHECK_AUTH(req);
    ml_config_ctx_t *ctx = (ml_config_ctx_t *)req->user_ctx;
    microlink_t *ml = ctx->ml;

    cJSON *json = cJSON_CreateObject();
    if (!json) return ESP_FAIL;

    char ip_buf[16] = "0.0.0.0";
    if (ml) {
        uint32_t vpn_ip = microlink_get_vpn_ip(ml);
        if (vpn_ip) microlink_ip_to_str(vpn_ip, ip_buf);
        cJSON_AddStringToObject(json, "vpn_ip", ip_buf);

        const char *states[] = {"IDLE","WIFI_WAIT","CONNECTING","REGISTERING","CONNECTED","RECONNECTING","ERROR"};
        int s = microlink_get_state(ml);
        cJSON_AddStringToObject(json, "state", (s >= 0 && s <= 6) ? states[s] : "UNKNOWN");
        cJSON_AddNumberToObject(json, "peer_count", microlink_get_peer_count(ml));
        cJSON_AddStringToObject(json, "hostname", ml->config.device_name ? ml->config.device_name : "");
    }

    /* Local WiFi IP */
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
            char local_ip[16];
            snprintf(local_ip, sizeof(local_ip), IPSTR, IP2STR(&ip_info.ip));
            cJSON_AddStringToObject(json, "local_ip", local_ip);
        }
    }

    cJSON_AddNumberToObject(json, "free_heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(json, "free_psram",
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    cJSON_AddBoolToObject(json, "filter_enabled", ctx->filter_enabled);

#ifdef CONFIG_ML_ENABLE_CELLULAR
    cJSON_AddStringToObject(json, "connection",
        ml_at_socket_is_ready() ? "cellular" : "wifi");
#else
    cJSON_AddStringToObject(json, "connection", "wifi");
#endif

    return send_json(req, json);
}

/* GET /api/settings */
static esp_err_t handler_get_settings(httpd_req_t *req) {
    CHECK_AUTH(req);
    ml_config_ctx_t *ctx = (ml_config_ctx_t *)req->user_ctx;

    cJSON *json = cJSON_CreateObject();
    if (!json) return ESP_FAIL;

    cJSON_AddStringToObject(json, "wifi_ssid", ctx->settings.wifi_ssid);
    /* Don't expose password in plain text — show masked */
    cJSON_AddStringToObject(json, "wifi_pass",
        ctx->settings.wifi_pass[0] ? "********" : "");
    cJSON_AddStringToObject(json, "auth_key",
        ctx->settings.auth_key[0] ? "********" : "");
    cJSON_AddStringToObject(json, "device_prefix", ctx->settings.device_prefix);
    cJSON_AddStringToObject(json, "cellular_apn", ctx->settings.cellular_apn);
    cJSON_AddStringToObject(json, "cellular_sim_pin",
        ctx->settings.cellular_sim_pin[0] ? "****" : "");

    /* v2 fields */
    cJSON_AddNumberToObject(json, "max_peers", ctx->settings.max_peers);
    cJSON_AddNumberToObject(json, "disco_heartbeat_ms", ctx->settings.disco_heartbeat_ms);
    char pip_str[16] = "";
    if (ctx->settings.priority_peer_ip) {
        microlink_ip_to_str(ctx->settings.priority_peer_ip, pip_str);
    }
    cJSON_AddStringToObject(json, "priority_peer_ip", pip_str);
    cJSON_AddStringToObject(json, "ctrl_host", ctx->settings.ctrl_host);
    cJSON_AddNumberToObject(json, "debug_flags", ctx->settings.debug_flags);
    cJSON_AddStringToObject(json, "device_name_full", ctx->settings.device_name_full);

    return send_json(req, json);
}

/* POST /api/settings */
static esp_err_t handler_post_settings(httpd_req_t *req) {
    CHECK_AUTH(req);
    ml_config_ctx_t *ctx = (ml_config_ctx_t *)req->user_ctx;

    char *body = read_post_body(req);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid body");
        return ESP_FAIL;
    }

    cJSON *json = cJSON_Parse(body);
    free(body);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    /* Helper macro: clear field, copy new value, ensure null termination */
    #define COPY_STR_FIELD(field, val) do { \
        memset((field), 0, sizeof(field)); \
        strncpy((field), (val), sizeof(field) - 1); \
    } while(0)

    /* Update only provided fields (don't clear fields not in the request) */
    cJSON *item;
    if ((item = cJSON_GetObjectItem(json, "wifi_ssid")) && cJSON_IsString(item)) {
        COPY_STR_FIELD(ctx->settings.wifi_ssid, item->valuestring);
    }
    /* For secret fields: skip masked placeholders AND empty strings to prevent
     * accidental credential wipe (browsers may send empty password inputs) */
    if ((item = cJSON_GetObjectItem(json, "wifi_pass")) && cJSON_IsString(item)) {
        const char *v = item->valuestring;
        if (v[0] != '\0' && strcmp(v, "********") != 0) {
            COPY_STR_FIELD(ctx->settings.wifi_pass, v);
        }
    }
    if ((item = cJSON_GetObjectItem(json, "auth_key")) && cJSON_IsString(item)) {
        const char *v = item->valuestring;
        if (v[0] != '\0' && strcmp(v, "********") != 0) {
            COPY_STR_FIELD(ctx->settings.auth_key, v);
        }
    }
    if ((item = cJSON_GetObjectItem(json, "device_prefix")) && cJSON_IsString(item)) {
        COPY_STR_FIELD(ctx->settings.device_prefix, item->valuestring);
    }
    if ((item = cJSON_GetObjectItem(json, "cellular_apn")) && cJSON_IsString(item)) {
        COPY_STR_FIELD(ctx->settings.cellular_apn, item->valuestring);
    }
    if ((item = cJSON_GetObjectItem(json, "cellular_sim_pin")) && cJSON_IsString(item)) {
        const char *v = item->valuestring;
        if (v[0] != '\0' && strcmp(v, "****") != 0) {
            COPY_STR_FIELD(ctx->settings.cellular_sim_pin, v);
        }
    }
    /* v2 fields */
    if ((item = cJSON_GetObjectItem(json, "max_peers")) && cJSON_IsNumber(item)) {
        int v = (int)item->valuedouble;
        ctx->settings.max_peers = (v >= 0 && v <= 128) ? (uint8_t)v : 0;
    }
    if ((item = cJSON_GetObjectItem(json, "disco_heartbeat_ms")) && cJSON_IsNumber(item)) {
        int v = (int)item->valuedouble;
        ctx->settings.disco_heartbeat_ms = (v >= 0 && v <= 60000) ? (uint16_t)v : 0;
    }
    if ((item = cJSON_GetObjectItem(json, "priority_peer_ip")) && cJSON_IsString(item)) {
        ctx->settings.priority_peer_ip = microlink_parse_ip(item->valuestring);
    }
    if ((item = cJSON_GetObjectItem(json, "ctrl_host")) && cJSON_IsString(item)) {
        COPY_STR_FIELD(ctx->settings.ctrl_host, item->valuestring);
    }
    if ((item = cJSON_GetObjectItem(json, "debug_flags")) && cJSON_IsNumber(item)) {
        ctx->settings.debug_flags = (uint8_t)item->valuedouble;
    }
    if ((item = cJSON_GetObjectItem(json, "device_name_full")) && cJSON_IsString(item)) {
        COPY_STR_FIELD(ctx->settings.device_name_full, item->valuestring);
    }
    #undef COPY_STR_FIELD
    cJSON_Delete(json);

    config_save_settings(ctx);

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "ok", true);
    cJSON_AddBoolToObject(resp, "restart_required", true);
    return send_json(req, resp);
}

/* GET /api/peers — all known peers with status */
static esp_err_t handler_get_peers(httpd_req_t *req) {
    CHECK_AUTH(req);
    ml_config_ctx_t *ctx = (ml_config_ctx_t *)req->user_ctx;
    microlink_t *ml = ctx->ml;

    cJSON *json = cJSON_CreateObject();
    if (!json) return ESP_FAIL;
    cJSON *arr = cJSON_AddArrayToObject(json, "peers");

    if (ml && arr) {
        int count = microlink_get_peer_count(ml);
        for (int i = 0; i < count; i++) {
            microlink_peer_info_t info;
            if (microlink_get_peer_info(ml, i, &info) != ESP_OK) continue;
            if (info.vpn_ip == 0) continue;

            cJSON *peer = cJSON_CreateObject();
            if (!peer) continue;
            char ip_str[16];
            microlink_ip_to_str(info.vpn_ip, ip_str);
            cJSON_AddStringToObject(peer, "ip", ip_str);
            cJSON_AddStringToObject(peer, "hostname", info.hostname);
            cJSON_AddBoolToObject(peer, "direct", info.direct_path);
            cJSON_AddBoolToObject(peer, "allowed",
                ml_config_peer_is_allowed(ctx, info.vpn_ip));
            cJSON_AddItemToArray(arr, peer);
        }
    }

    return send_json(req, json);
}

/* GET /api/peers/allowed — current allowlist */
static esp_err_t handler_get_allowed(httpd_req_t *req) {
    CHECK_AUTH(req);
    ml_config_ctx_t *ctx = (ml_config_ctx_t *)req->user_ctx;

    cJSON *json = cJSON_CreateObject();
    if (!json) return ESP_FAIL;
    cJSON_AddBoolToObject(json, "filter_enabled", ctx->filter_enabled);
    cJSON *arr = cJSON_AddArrayToObject(json, "peers");

    if (arr && xSemaphoreTake(ctx->peer_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < ctx->peer_list.count; i++) {
            cJSON *entry = cJSON_CreateObject();
            if (!entry) continue;
            char ip_str[16];
            microlink_ip_to_str(ctx->peer_list.entries[i].vpn_ip, ip_str);
            cJSON_AddStringToObject(entry, "ip", ip_str);
            cJSON_AddStringToObject(entry, "label", ctx->peer_list.entries[i].label);
            cJSON_AddItemToArray(arr, entry);
        }
        xSemaphoreGive(ctx->peer_mutex);
    }

    return send_json(req, json);
}

/* POST /api/peers/allowed — replace allowlist */
static esp_err_t handler_post_allowed(httpd_req_t *req) {
    CHECK_AUTH(req);
    ml_config_ctx_t *ctx = (ml_config_ctx_t *)req->user_ctx;

    char *body = read_post_body(req);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid body");
        return ESP_FAIL;
    }

    cJSON *json = cJSON_Parse(body);
    free(body);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *peers = cJSON_GetObjectItem(json, "peers");
    if (!peers || !cJSON_IsArray(peers)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing peers array");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(ctx->peer_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        ctx->peer_list.count = 0;
        int arr_size = cJSON_GetArraySize(peers);
        for (int i = 0; i < arr_size && i < ML_CONFIG_MAX_ALLOWED_PEERS; i++) {
            cJSON *entry = cJSON_GetArrayItem(peers, i);
            cJSON *ip = cJSON_GetObjectItem(entry, "ip");
            cJSON *label = cJSON_GetObjectItem(entry, "label");
            if (!ip || !cJSON_IsString(ip)) continue;

            uint32_t vpn_ip = microlink_parse_ip(ip->valuestring);
            if (vpn_ip == 0) continue;

            ml_config_peer_entry_t *e = &ctx->peer_list.entries[ctx->peer_list.count];
            e->vpn_ip = vpn_ip;
            memset(e->label, 0, sizeof(e->label));
            if (label && cJSON_IsString(label)) {
                strncpy(e->label, label->valuestring, sizeof(e->label) - 1);
            }
            ctx->peer_list.count++;
        }
        ctx->filter_enabled = (ctx->peer_list.count > 0);
        xSemaphoreGive(ctx->peer_mutex);
    }
    cJSON_Delete(json);

    config_save_peers(ctx);

    ESP_LOGI(TAG, "Allowlist updated: %d peers, filter %s",
             ctx->peer_list.count, ctx->filter_enabled ? "ON" : "OFF");

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "ok", true);
    cJSON_AddNumberToObject(resp, "count", ctx->peer_list.count);
    cJSON_AddBoolToObject(resp, "filter_enabled", ctx->filter_enabled);
    return send_json(req, resp);
}

/* DELETE /api/peers/allowed — clear allowlist */
static esp_err_t handler_delete_allowed(httpd_req_t *req) {
    CHECK_AUTH(req);
    ml_config_ctx_t *ctx = (ml_config_ctx_t *)req->user_ctx;

    if (xSemaphoreTake(ctx->peer_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        ctx->peer_list.count = 0;
        ctx->filter_enabled = false;
        xSemaphoreGive(ctx->peer_mutex);
    }
    config_save_peers(ctx);

    ESP_LOGI(TAG, "Allowlist cleared, filter OFF (all peers allowed)");

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "ok", true);
    cJSON_AddBoolToObject(resp, "filter_enabled", false);
    return send_json(req, resp);
}

/* GET /api/monitor — temperature, RSSI, uptime, task stack watermarks */
static esp_err_t handler_monitor(httpd_req_t *req) {
    CHECK_AUTH(req);
    ml_config_ctx_t *ctx = (ml_config_ctx_t *)req->user_ctx;
    microlink_t *ml = ctx->ml;

    cJSON *json = cJSON_CreateObject();
    if (!json) return ESP_FAIL;

    /* Temperature */
#if SOC_TEMP_SENSOR_SUPPORTED
    float temp_c = 0;
    if (ctx->temp_sensor &&
        temperature_sensor_get_celsius(ctx->temp_sensor, &temp_c) == ESP_OK) {
        cJSON_AddNumberToObject(json, "temp_c", (int)(temp_c * 10) / 10.0);
    } else {
        cJSON_AddNullToObject(json, "temp_c");
    }
#else
    cJSON_AddNullToObject(json, "temp_c");
#endif

    /* WiFi RSSI + channel. esp_wifi_sta_get_ap_info() NULL-derefs inside
     * current_task_is_wifi_task when WiFi was never esp_wifi_init'd (a wired-only
     * deployment may skip esp_wifi_init to save RAM). esp_wifi_get_mode() is the
     * safe init probe — it returns ESP_ERR_WIFI_NOT_INIT without touching the
     * WiFi control block — so gate the WiFi queries on it. (Without this, polling
     * GET /api/monitor crash-loops a wired device.) */
    wifi_mode_t _wmode;
    bool wifi_inited = (esp_wifi_get_mode(&_wmode) == ESP_OK);
    int rssi = 0;
    if (wifi_inited && esp_wifi_sta_get_rssi(&rssi) == ESP_OK) {
        cJSON_AddNumberToObject(json, "rssi", rssi);
    } else {
        cJSON_AddNullToObject(json, "rssi");
    }

    /* WiFi channel info */
    wifi_ap_record_t ap_info;
    if (wifi_inited && esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        cJSON_AddNumberToObject(json, "wifi_channel", ap_info.primary);
        cJSON_AddStringToObject(json, "wifi_ssid", (char *)ap_info.ssid);
    }

    /* Uptime */
    uint64_t uptime_ms = esp_timer_get_time() / 1000;
    cJSON_AddNumberToObject(json, "uptime_s", (double)(uptime_ms / 1000));

    /* Heap */
    cJSON_AddNumberToObject(json, "free_heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(json, "min_free_heap", esp_get_minimum_free_heap_size());
    cJSON_AddNumberToObject(json, "free_psram",
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    /* Task stack watermarks. xTaskGetHandle returns NULL if the task name
     * doesn't exist (no panic). Includes IDF tasks we can't reach via
     * ml->* handles — TCPIP, WiFi, event loop, timer service. */
    if (ml) {
        cJSON *tasks = cJSON_AddObjectToObject(json, "tasks");
        struct { const char *name; TaskHandle_t handle; } tlist[] = {
            {"net_io",  ml->net_io_task},
            {"derp_tx", ml->derp_tx_task},
            {"coord",   ml->coord_task},
            {"wg_mgr",  ml->wg_mgr_task},
            {"tcpip",   xTaskGetHandle("tiT")},
            {"wifi",    xTaskGetHandle("wifi")},
            {"sys_evt", xTaskGetHandle("sys_evt")},
            {"Tmr_Svc", xTaskGetHandle("Tmr Svc")},
            {"esp_timer", xTaskGetHandle("esp_timer")},
            {"app_hb",  xTaskGetHandle("app_hb")},
            {"pstop_hb", xTaskGetHandle("pstop_hb")},
        };
        for (int i = 0; i < (int)(sizeof(tlist)/sizeof(tlist[0])); i++) {
            if (tlist[i].handle) {
                cJSON *t = cJSON_AddObjectToObject(tasks, tlist[i].name);
                eTaskState st = eTaskGetState(tlist[i].handle);
                const char *sn[] = {"Running","Ready","Blocked","Suspended","Deleted","Invalid"};
                cJSON_AddStringToObject(t, "state", (st <= eInvalid) ? sn[st] : "Unknown");
                cJSON_AddNumberToObject(t, "stack_free",
                    (double)uxTaskGetStackHighWaterMark(tlist[i].handle));
            }
        }

        /* DERP status */
        cJSON_AddBoolToObject(json, "derp_connected", ml->derp.connected);
        cJSON_AddNumberToObject(json, "derp_home_region", ml->derp_home_region);
    }

    /* Connection type */
#ifdef CONFIG_ML_ENABLE_CELLULAR
    if (ml_at_socket_is_ready()) {
        cJSON_AddStringToObject(json, "connection", "cellular");
    } else {
        cJSON_AddStringToObject(json, "connection", "wifi");
    }
#else
    cJSON_AddStringToObject(json, "connection", "wifi");
#endif

    return send_json(req, json);
}

/* POST /api/ota — firmware upload */
static esp_err_t handler_ota(httpd_req_t *req) {
    CHECK_AUTH(req);
    if (req->content_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No firmware data");
        return ESP_FAIL;
    }

    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition found");
        return ESP_FAIL;
    }

    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "========== OTA FIRMWARE UPDATE ==========");
    ESP_LOGI(TAG, "OTA: current partition: '%s' at 0x%lx",
             running ? running->label : "?", running ? (unsigned long)running->address : 0);
    ESP_LOGI(TAG, "OTA: target partition:  '%s' at 0x%lx (size: %lu KB)",
             update->label, (unsigned long)update->address, (unsigned long)update->size / 1024);
    ESP_LOGI(TAG, "OTA: incoming firmware: %d bytes (%d KB)",
             req->content_len, req->content_len / 1024);

    ESP_LOGI(TAG, "OTA: erasing target partition and beginning write...");
    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: begin FAILED: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "OTA: partition ready, receiving data...");
    s_ota_in_progress = true;

    char *buf = malloc(4096);
    if (!buf) {
        esp_ota_abort(ota_handle);
        s_ota_in_progress = false;
        ESP_LOGE(TAG, "OTA: out of memory for receive buffer");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    int total_written = 0;
    int last_pct = -1;

    while (remaining > 0) {
        int recv_len = httpd_req_recv(req, buf, (remaining < 4096) ? remaining : 4096);
        if (recv_len <= 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "OTA: recv timeout at %d%%, retrying...",
                         total_written * 100 / req->content_len);
                continue;
            }
            ESP_LOGE(TAG, "OTA: connection lost at %d / %d bytes (%d%%)",
                     total_written, req->content_len,
                     total_written * 100 / req->content_len);
            free(buf);
            esp_ota_abort(ota_handle);
            s_ota_in_progress = false;
        s_ota_in_progress = false;
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
            return ESP_FAIL;
        }

        err = esp_ota_write(ota_handle, buf, recv_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OTA: flash write FAILED at %d bytes: %s",
                     total_written, esp_err_to_name(err));
            free(buf);
            esp_ota_abort(ota_handle);
            s_ota_in_progress = false;
        s_ota_in_progress = false;
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash write failed");
            return ESP_FAIL;
        }

        remaining -= recv_len;
        total_written += recv_len;

        int pct = total_written * 100 / req->content_len;
        if (pct / 10 != last_pct / 10) {
            last_pct = pct;
            ESP_LOGI(TAG, "OTA: %3d%%  (%d / %d bytes, %d KB remaining)",
                     pct, total_written, req->content_len, remaining / 1024);
        }
    }

    free(buf);
    ESP_LOGI(TAG, "OTA: receive complete — %d bytes, validating image...", total_written);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: image validation FAILED: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "OTA: firmware may be corrupted or built for a different chip");
        s_ota_in_progress = false;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Image validation failed (corrupt or wrong chip?)");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "OTA: image validated OK");

    err = esp_ota_set_boot_partition(update);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: set boot partition FAILED: %s", esp_err_to_name(err));
        s_ota_in_progress = false;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "========== OTA UPDATE SUCCESSFUL ==========");
    ESP_LOGI(TAG, "OTA: %d bytes written to '%s'", total_written, update->label);
    ESP_LOGI(TAG, "OTA: next boot will load from '%s'", update->label);
    ESP_LOGW(TAG, "OTA: rebooting in 3 seconds...");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"message\":\"Firmware updated. Rebooting...\"}");

    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGW(TAG, "OTA: rebooting in 2 seconds...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGW(TAG, "OTA: rebooting in 1 second...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGW(TAG, "OTA: rebooting NOW");
    esp_restart();
    return ESP_OK;  /* unreachable */
}

/* GET /api/ota/status — current OTA partition info */
static esp_err_t handler_ota_status(httpd_req_t *req) {
    CHECK_AUTH(req);
    cJSON *json = cJSON_CreateObject();
    if (!json) return ESP_FAIL;

    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();
    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);

    if (running) {
        cJSON_AddStringToObject(json, "running_partition", running->label);

        /* Get actual firmware binary size from image header */
        esp_image_header_t hdr;
        if (esp_partition_read(running, 0, &hdr, sizeof(hdr)) == ESP_OK) {
            /* Walk segments to compute total image size */
            uint32_t offset = sizeof(esp_image_header_t);
            for (int seg = 0; seg < hdr.segment_count && seg < 16; seg++) {
                esp_image_segment_header_t seg_hdr;
                if (esp_partition_read(running, offset, &seg_hdr, sizeof(seg_hdr)) != ESP_OK) break;
                offset += sizeof(seg_hdr) + seg_hdr.data_len;
            }
            /* Add checksum byte + SHA-256 hash (32 bytes) if enabled */
            offset += 1 + 32;
            cJSON_AddNumberToObject(json, "current_firmware_size", offset);
        }
    }
    if (boot) {
        cJSON_AddStringToObject(json, "boot_partition", boot->label);
    }
    if (update) {
        cJSON_AddStringToObject(json, "update_partition", update->label);
        cJSON_AddNumberToObject(json, "max_firmware_size", update->size);
    }

    /* App version info */
    const esp_app_desc_t *app_desc = esp_app_get_description();
    if (app_desc) {
        cJSON_AddStringToObject(json, "app_version", app_desc->version);
        cJSON_AddStringToObject(json, "compile_date", app_desc->date);
        cJSON_AddStringToObject(json, "compile_time", app_desc->time);
        cJSON_AddStringToObject(json, "idf_version", app_desc->idf_ver);
    }

    /* Rollback state */
    esp_ota_img_states_t ota_state;
    if (running && esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        const char *state_names[] = {
            "NEW", "PENDING_VERIFY", "VALID", "INVALID", "ABORTED", "UNDEFINED"
        };
        int si = (ota_state <= ESP_OTA_IMG_UNDEFINED) ? (int)ota_state : 5;
        cJSON_AddStringToObject(json, "ota_state", state_names[si]);
    }

    /* Rollback detection: if there's a last-invalid partition, a rollback occurred */
    const esp_partition_t *invalid = esp_ota_get_last_invalid_partition();
    if (invalid) {
        cJSON_AddBoolToObject(json, "rollback_occurred", true);
        cJSON_AddStringToObject(json, "rolled_back_from", invalid->label);
        esp_ota_img_states_t inv_state;
        if (esp_ota_get_state_partition(invalid, &inv_state) == ESP_OK) {
            const char *reasons[] = {
                "NEW", "PENDING_VERIFY", "VALID", "INVALID", "ABORTED", "UNDEFINED"
            };
            int ri = (inv_state <= ESP_OTA_IMG_UNDEFINED) ? (int)inv_state : 5;
            cJSON_AddStringToObject(json, "rollback_reason", reasons[ri]);
        }
    } else {
        cJSON_AddBoolToObject(json, "rollback_occurred", false);
    }

    return send_json(req, json);
}

/* POST /api/restart */
static esp_err_t handler_restart(httpd_req_t *req) {
    CHECK_AUTH(req);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"restarting\":true}");

    /* Delay to let response complete, then restart */
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;  /* unreachable */
}

/* ============================================================================
 * WiFi List HTTP Handlers
 * ========================================================================== */

/* GET /api/wifi — return WiFi list with masked passwords */
static esp_err_t handler_get_wifi(httpd_req_t *req) {
    CHECK_AUTH(req);
    ml_config_ctx_t *ctx = (ml_config_ctx_t *)req->user_ctx;

    cJSON *json = cJSON_CreateObject();
    if (!json) return ESP_FAIL;
    cJSON *arr = cJSON_AddArrayToObject(json, "networks");

    for (int i = 0; i < ctx->wifi_list.count && i < ML_CONFIG_MAX_WIFI_ENTRIES; i++) {
        cJSON *entry = cJSON_CreateObject();
        if (!entry) continue;
        cJSON_AddStringToObject(entry, "ssid", ctx->wifi_list.entries[i].ssid);
        cJSON_AddStringToObject(entry, "pass",
            ctx->wifi_list.entries[i].pass[0] ? "********" : "");
        cJSON_AddItemToArray(arr, entry);
    }
    cJSON_AddNumberToObject(json, "active_idx",
        ctx->wifi_list.active_idx == 0xFF ? -1 : ctx->wifi_list.active_idx);

    return send_json(req, json);
}

/* POST /api/wifi — replace entire WiFi list */
static esp_err_t handler_post_wifi(httpd_req_t *req) {
    CHECK_AUTH(req);
    ml_config_ctx_t *ctx = (ml_config_ctx_t *)req->user_ctx;

    char *body = read_post_body(req);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid body");
        return ESP_FAIL;
    }

    cJSON *json = cJSON_Parse(body);
    free(body);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *networks = cJSON_GetObjectItem(json, "networks");
    if (!networks || !cJSON_IsArray(networks)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing networks array");
        return ESP_FAIL;
    }

    /* Save old list so we can resolve masked passwords */
    ml_config_wifi_list_t old_list;
    memcpy(&old_list, &ctx->wifi_list, sizeof(old_list));

    /* Build new list */
    memset(&ctx->wifi_list, 0, sizeof(ctx->wifi_list));
    ctx->wifi_list.active_idx = 0xFF;

    int arr_size = cJSON_GetArraySize(networks);
    for (int i = 0; i < arr_size && i < ML_CONFIG_MAX_WIFI_ENTRIES; i++) {
        cJSON *entry = cJSON_GetArrayItem(networks, i);
        cJSON *ssid = cJSON_GetObjectItem(entry, "ssid");
        cJSON *pass = cJSON_GetObjectItem(entry, "pass");
        if (!ssid || !cJSON_IsString(ssid) || ssid->valuestring[0] == '\0') continue;

        ml_config_wifi_entry_t *e = &ctx->wifi_list.entries[ctx->wifi_list.count];
        memset(e, 0, sizeof(*e));
        strncpy(e->ssid, ssid->valuestring, sizeof(e->ssid) - 1);

        if (pass && cJSON_IsString(pass)) {
            const char *pv = pass->valuestring;
            if (strcmp(pv, "********") == 0) {
                /* Masked: find matching SSID in old list and copy password */
                for (int j = 0; j < old_list.count; j++) {
                    if (strcmp(old_list.entries[j].ssid, e->ssid) == 0) {
                        strncpy(e->pass, old_list.entries[j].pass, sizeof(e->pass) - 1);
                        break;
                    }
                }
            } else if (pv[0] != '\0') {
                strncpy(e->pass, pv, sizeof(e->pass) - 1);
            }
        }
        ctx->wifi_list.count++;
    }
    cJSON_Delete(json);

    /* Update settings blob wifi_ssid/wifi_pass with entry[0] for backwards compat */
    if (ctx->wifi_list.count > 0) {
        memset(ctx->settings.wifi_ssid, 0, sizeof(ctx->settings.wifi_ssid));
        memset(ctx->settings.wifi_pass, 0, sizeof(ctx->settings.wifi_pass));
        strncpy(ctx->settings.wifi_ssid, ctx->wifi_list.entries[0].ssid,
                sizeof(ctx->settings.wifi_ssid) - 1);
        strncpy(ctx->settings.wifi_pass, ctx->wifi_list.entries[0].pass,
                sizeof(ctx->settings.wifi_pass) - 1);
        config_save_settings(ctx);
    }

    config_save_wifi_list(ctx);

    ESP_LOGI(TAG, "WiFi list updated: %d networks", ctx->wifi_list.count);

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "ok", true);
    cJSON_AddNumberToObject(resp, "count", ctx->wifi_list.count);
    cJSON_AddBoolToObject(resp, "restart_required", true);
    return send_json(req, resp);
}

/* ============================================================================
 * WiFi List Public API
 * ========================================================================== */

bool ml_config_get_wifi_list(ml_config_wifi_list_t *list) {
    if (!list) return false;

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) return false;

    memset(list, 0, sizeof(*list));
    list->active_idx = 0xFF;

    /* Try wifi_list key first */
    size_t stored_len = 0;
    err = nvs_get_blob(nvs, NVS_KEY_WIFI, NULL, &stored_len);
    if (err == ESP_OK && stored_len >= 2) {
        size_t read_len = stored_len;
        if (read_len > sizeof(ml_config_wifi_list_t)) read_len = sizeof(ml_config_wifi_list_t);
        err = nvs_get_blob(nvs, NVS_KEY_WIFI, list, &read_len);
        if (err == ESP_OK && list->count > 0 && list->count <= ML_CONFIG_MAX_WIFI_ENTRIES) {
            nvs_close(nvs);
            return true;
        }
        /* Invalid data, fall through */
        memset(list, 0, sizeof(*list));
        list->active_idx = 0xFF;
    }

    /* Fall back to settings blob WiFi (heap-alloc to avoid stack overflow) */
    ml_config_settings_t *settings = heap_caps_malloc(sizeof(*settings), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!settings) { nvs_close(nvs); return false; }
    memset(settings, 0, sizeof(*settings));
    size_t slen = sizeof(*settings);
    err = nvs_get_blob(nvs, NVS_KEY_SETTINGS, settings, &slen);
    nvs_close(nvs);

    if (err == ESP_OK && settings->wifi_ssid[0] != '\0') {
        list->count = 1;
        list->active_idx = 0;
        strncpy(list->entries[0].ssid, settings->wifi_ssid, 32);
        strncpy(list->entries[0].pass, settings->wifi_pass, 64);
        free(settings);
        return true;
    }

    free(settings);
    return false;
}

/* ============================================================================
 * Early Boot WiFi Override (reads NVS without full config init)
 * ========================================================================== */

bool ml_config_get_nvs_wifi(char *ssid, size_t ssid_len,
                             char *pass, size_t pass_len) {
    /* Heap-allocate to avoid stack overflow in app_main (1570+400 bytes) */
    ml_config_wifi_list_t *wlist = heap_caps_malloc(sizeof(*wlist), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (wlist && ml_config_get_wifi_list(wlist) && wlist->count > 0) {
        if (ssid && ssid_len > 0) {
            strncpy(ssid, wlist->entries[0].ssid, ssid_len - 1);
            ssid[ssid_len - 1] = '\0';
        }
        if (pass && pass_len > 0) {
            strncpy(pass, wlist->entries[0].pass, pass_len - 1);
            pass[pass_len - 1] = '\0';
        }
        ESP_LOGI(TAG, "WiFi from wifi_list[0]: %s", wlist->entries[0].ssid);
        free(wlist);
        return true;
    }
    free(wlist);  /* NULL-safe */

    /* Fall back to settings blob */
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) return false;

    ml_config_settings_t *settings = heap_caps_malloc(sizeof(*settings), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!settings) { nvs_close(nvs); return false; }
    memset(settings, 0, sizeof(*settings));
    size_t len = sizeof(*settings);
    err = nvs_get_blob(nvs, NVS_KEY_SETTINGS, settings, &len);
    nvs_close(nvs);

    if (err != ESP_OK || len < 1 || settings->version < 1 || settings->wifi_ssid[0] == '\0') {
        free(settings);
        return false;
    }

    if (ssid && ssid_len > 0) {
        strncpy(ssid, settings->wifi_ssid, ssid_len - 1);
        ssid[ssid_len - 1] = '\0';
    }
    if (pass && pass_len > 0) {
        strncpy(pass, settings->wifi_pass, pass_len - 1);
        pass[pass_len - 1] = '\0';
    }

    ESP_LOGI(TAG, "WiFi from settings: %s", settings->wifi_ssid);
    free(settings);
    return true;
}

/* ============================================================================
 * External HTTPD support (used by ml_app framework)
 * ========================================================================== */

esp_err_t ml_config_httpd_register_routes(ml_config_ctx_t *ctx, microlink_t *ml,
                                           httpd_handle_t httpd, const char *prefix) {
    if (!ctx || !httpd) return ESP_ERR_INVALID_ARG;
    ctx->ml = ml;
    ctx->httpd = httpd;
    ctx->httpd_external = true;

    const char *pfx = prefix ? prefix : "";

    const struct {
        const char *uri;
        httpd_method_t method;
        esp_err_t (*handler)(httpd_req_t *);
    } routes[] = {
        { "/",                HTTP_GET,    handler_root },
        { "/api/status",      HTTP_GET,    handler_status },
        { "/api/settings",    HTTP_GET,    handler_get_settings },
        { "/api/settings",    HTTP_POST,   handler_post_settings },
        { "/api/monitor",     HTTP_GET,    handler_monitor },
        { "/api/peers",       HTTP_GET,    handler_get_peers },
        { "/api/peers/allowed", HTTP_GET,  handler_get_allowed },
        { "/api/peers/allowed", HTTP_POST, handler_post_allowed },
        { "/api/peers/allowed", HTTP_DELETE, handler_delete_allowed },
        { "/api/restart",     HTTP_POST,   handler_restart },
        { "/api/wifi",        HTTP_GET,    handler_get_wifi },
        { "/api/wifi",        HTTP_POST,   handler_post_wifi },
        { "/api/ota",         HTTP_POST,   handler_ota },
        { "/api/ota/status",  HTTP_GET,    handler_ota_status },
    };

    char uri_buf[64];
    for (int i = 0; i < (int)(sizeof(routes) / sizeof(routes[0])); i++) {
        snprintf(uri_buf, sizeof(uri_buf), "%s%s", pfx, routes[i].uri);
        httpd_uri_t uri_def = {
            .uri = uri_buf,
            .method = routes[i].method,
            .handler = routes[i].handler,
            .user_ctx = ctx,
        };
        httpd_register_uri_handler(httpd, &uri_def);
    }

    ESP_LOGI(TAG, "Admin routes registered with prefix '%s' (%d handlers)",
             pfx, (int)(sizeof(routes) / sizeof(routes[0])));
    return ESP_OK;
}

void ml_config_httpd_set_ml(ml_config_ctx_t *ctx, microlink_t *ml) {
    if (ctx) ctx->ml = ml;
}

bool ml_config_httpd_is_started(const ml_config_ctx_t *ctx) {
    return ctx && ctx->httpd != NULL;
}

void ml_config_httpd_set_auth(ml_config_ctx_t *ctx, bool (*auth_check)(httpd_req_t *)) {
    if (ctx) ctx->auth_check = auth_check;
}

/* ============================================================================
 * Lifecycle
 * ========================================================================== */

ml_config_ctx_t *ml_config_httpd_init(void) {
    ml_config_ctx_t *ctx = calloc(1, sizeof(ml_config_ctx_t));
    if (!ctx) return NULL;

    ctx->peer_mutex = xSemaphoreCreateMutex();
    if (!ctx->peer_mutex) {
        free(ctx);
        return NULL;
    }

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &ctx->nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s",
                 NVS_NAMESPACE, esp_err_to_name(err));
        vSemaphoreDelete(ctx->peer_mutex);
        free(ctx);
        return NULL;
    }

    config_load_settings(ctx);
    config_load_peers(ctx);
    config_load_wifi_list(ctx);

#if SOC_TEMP_SENSOR_SUPPORTED
    /* Initialize temperature sensor */
    temperature_sensor_config_t tsens_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    if (temperature_sensor_install(&tsens_cfg, &ctx->temp_sensor) == ESP_OK) {
        temperature_sensor_enable(ctx->temp_sensor);
        ESP_LOGI(TAG, "Temperature sensor initialized");
    } else {
        ctx->temp_sensor = NULL;
        ESP_LOGW(TAG, "Temperature sensor init failed (non-critical)");
    }
#endif

    ESP_LOGI(TAG, "Config module initialized");
    return ctx;
}

esp_err_t ml_config_httpd_start(ml_config_ctx_t *ctx, microlink_t *ml) {
    if (!ctx) return ESP_ERR_INVALID_ARG;
    ctx->ml = ml;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 10240;
    config.max_uri_handlers = 16;
    config.recv_wait_timeout = 30;
    config.uri_match_fn = httpd_uri_match_wildcard;

    esp_err_t err = httpd_start(&ctx->httpd, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(err));
        return err;
    }

    /* Register URI handlers */
    const httpd_uri_t uris[] = {
        { .uri = "/",                .method = HTTP_GET,    .handler = handler_root,          .user_ctx = ctx },
        { .uri = "/api/status",      .method = HTTP_GET,    .handler = handler_status,        .user_ctx = ctx },
        { .uri = "/api/settings",    .method = HTTP_GET,    .handler = handler_get_settings,  .user_ctx = ctx },
        { .uri = "/api/settings",    .method = HTTP_POST,   .handler = handler_post_settings, .user_ctx = ctx },
        { .uri = "/api/monitor",     .method = HTTP_GET,    .handler = handler_monitor,       .user_ctx = ctx },
        { .uri = "/api/peers",       .method = HTTP_GET,    .handler = handler_get_peers,     .user_ctx = ctx },
        { .uri = "/api/peers/allowed", .method = HTTP_GET,  .handler = handler_get_allowed,   .user_ctx = ctx },
        { .uri = "/api/peers/allowed", .method = HTTP_POST, .handler = handler_post_allowed,  .user_ctx = ctx },
        { .uri = "/api/peers/allowed", .method = HTTP_DELETE, .handler = handler_delete_allowed, .user_ctx = ctx },
        { .uri = "/api/restart",     .method = HTTP_POST,   .handler = handler_restart,       .user_ctx = ctx },
        { .uri = "/api/wifi",        .method = HTTP_GET,    .handler = handler_get_wifi,      .user_ctx = ctx },
        { .uri = "/api/wifi",        .method = HTTP_POST,   .handler = handler_post_wifi,     .user_ctx = ctx },
        { .uri = "/api/ota",         .method = HTTP_POST,   .handler = handler_ota,           .user_ctx = ctx },
        { .uri = "/api/ota/status",  .method = HTTP_GET,    .handler = handler_ota_status,    .user_ctx = ctx },
    };

    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(ctx->httpd, &uris[i]);
    }

    ESP_LOGI(TAG, "HTTP config server started on port %d", config.server_port);
    return ESP_OK;
}

void ml_config_httpd_stop(ml_config_ctx_t *ctx) {
    if (!ctx) return;
    if (ctx->httpd) {
        if (!ctx->httpd_external) {
            httpd_stop(ctx->httpd);
        }
        ctx->httpd = NULL;
        ESP_LOGI(TAG, "HTTP config server stopped");
    }
}

void ml_config_httpd_deinit(ml_config_ctx_t *ctx) {
    if (!ctx) return;
    ml_config_httpd_stop(ctx);
#if SOC_TEMP_SENSOR_SUPPORTED
    if (ctx->temp_sensor) {
        temperature_sensor_disable(ctx->temp_sensor);
        temperature_sensor_uninstall(ctx->temp_sensor);
    }
#endif
    if (ctx->nvs) nvs_close(ctx->nvs);
    if (ctx->peer_mutex) vSemaphoreDelete(ctx->peer_mutex);
    free(ctx);
}

#endif /* CONFIG_ML_ENABLE_CONFIG_HTTPD */
