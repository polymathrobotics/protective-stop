// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ml_secrets.c
 * @brief Loader for the HMAC-encrypted "secrets" NVS partition.
 */

#include "ml_secrets.h"

#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "mbedtls/platform_util.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "nvs_sec_provider.h"
#include "sdkconfig.h"

static const char * TAG = "ml_secrets";

/* Indexed by ml_secret_t. */
static const char * const NVS_KEYS[ML_SECRET_COUNT] = {
  [ML_SECRET_TAILSCALE_AUTH_KEY] = "ts_auth_key",
  [ML_SECRET_ADMIN_PASSWORD] = "admin_pw",
  [ML_SECRET_WIFI_SSID] = "wifi_ssid",
  [ML_SECRET_WIFI_PASSWORD] = "wifi_pass",
  [ML_SECRET_WIFI_SSID_2] = "wifi_ssid_2",
  [ML_SECRET_WIFI_PASSWORD_2] = "wifi_pass_2",
  [ML_SECRET_FLEET_SERVER_IP] = "fleet_srv_ip",
  [ML_SECRET_OTA_BACKEND_URL] = "ota_url",
  [ML_SECRET_OTA_API_KEY] = "ota_api_key",
};

/* PSRAM when available; NULL until loaded, and on allocation failure. */
static char (*s_values)[ML_SECRETS_MAX_LEN];
static esp_err_t s_status = ESP_ERR_INVALID_STATE;
static pthread_once_t s_once = PTHREAD_ONCE_INIT;

static esp_err_t read_values(nvs_handle_t h)
{
  s_values = heap_caps_calloc(ML_SECRET_COUNT, ML_SECRETS_MAX_LEN, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (NULL == s_values) {
    s_values = calloc(ML_SECRET_COUNT, ML_SECRETS_MAX_LEN);
  }
  if (NULL == s_values) {
    return ESP_ERR_NO_MEM;
  }
  int loaded = 0;
  for (int i = 0; i < ML_SECRET_COUNT; i++) {
    size_t len = ML_SECRETS_MAX_LEN;
    esp_err_t err = nvs_get_str(h, NVS_KEYS[i], s_values[i], &len);
    if (ESP_OK == err) {
      loaded++;
    } else {
      s_values[i][0] = '\0';
      if (ESP_ERR_NVS_NOT_FOUND != err) {
        ESP_LOGE(TAG, "%s: %s", NVS_KEYS[i], esp_err_to_name(err));
      }
    }
  }
  ESP_LOGI(TAG, "Loaded %d of %d secrets", loaded, ML_SECRET_COUNT);
  return ESP_OK;
}

static esp_err_t load(void)
{
  const nvs_sec_config_hmac_t hmac_cfg = {.hmac_key_id = (hmac_key_id_t)CONFIG_ML_SECRETS_HMAC_KEY_ID};
  nvs_sec_scheme_t * scheme = NULL;
  esp_err_t err = nvs_sec_provider_register_hmac(&hmac_cfg, &scheme);
  if (ESP_OK != err) {
    return err;
  }

  nvs_sec_cfg_t keys = {0};
  err = nvs_flash_read_security_cfg_v2(scheme, &keys);
  if (ESP_OK == err) {
    err = nvs_flash_secure_init_partition(ML_SECRETS_PARTITION, &keys);
  }
  mbedtls_platform_zeroize(&keys, sizeof(keys));
  (void)nvs_sec_provider_deregister(scheme);
  if (ESP_OK != err) {
    return err;
  }

  nvs_handle_t h;
  err = nvs_open_from_partition(ML_SECRETS_PARTITION, ML_SECRETS_NAMESPACE, NVS_READONLY, &h);
  if (ESP_OK == err) {
    err = read_values(h);
    nvs_close(h);
  }
  (void)nvs_flash_deinit_partition(ML_SECRETS_PARTITION);
  return err;
}

static void load_once(void)
{
  s_status = load();
  if (ESP_ERR_NVS_SEC_HMAC_KEY_NOT_FOUND == s_status) {
    ESP_LOGW(TAG, "Not provisioned: no HMAC key in eFuse key block %d", CONFIG_ML_SECRETS_HMAC_KEY_ID);
  } else if (ESP_OK != s_status) {
    ESP_LOGE(TAG, "Secrets unavailable: %s", esp_err_to_name(s_status));
  }
}

esp_err_t ml_secrets_init(void)
{
  (void)pthread_once(&s_once, load_once);
  return s_status;
}

const char * ml_secrets_get(ml_secret_t id)
{
  (void)ml_secrets_init();
  if ((ESP_OK != s_status) || (NULL == s_values) || ((unsigned)id >= (unsigned)ML_SECRET_COUNT)) {
    return "";
  }
  return s_values[id];
}
