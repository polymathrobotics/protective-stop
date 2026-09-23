// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ml_secrets.h
 * @brief Read-only access to the per-device provisioned secrets.
 *
 * Secrets live in the "secrets" NVS partition, XTS-AES encrypted with keys the
 * HMAC peripheral derives from the eFuse key block CONFIG_ML_SECRETS_HMAC_KEY_ID.
 * tools/provision_secrets.py builds that partition at flash time.
 * The firmware only reads it: it never writes the partition or burns a key.
 *
 * The NVS key names below are a contract with tools/provision_secrets.py;
 * tools/test/test_provision_secrets.py checks the two agree.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

/// Label of the encrypted partition in partitions.csv.
#define ML_SECRETS_PARTITION "secrets"
/// NVS namespace inside that partition.
#define ML_SECRETS_NAMESPACE "ml_secrets"
/// Capacity of every secret, including the terminating NUL.
#define ML_SECRETS_MAX_LEN 129

  /// One provisioned value.
  typedef enum
  {
    ML_SECRET_TAILSCALE_AUTH_KEY,  ///< NVS key "ts_auth_key"
    ML_SECRET_ADMIN_PASSWORD,  ///< NVS key "admin_pw"
    ML_SECRET_WIFI_SSID,  ///< NVS key "wifi_ssid"
    ML_SECRET_WIFI_PASSWORD,  ///< NVS key "wifi_pass"
    ML_SECRET_WIFI_SSID_2,  ///< NVS key "wifi_ssid_2"
    ML_SECRET_WIFI_PASSWORD_2,  ///< NVS key "wifi_pass_2"
    ML_SECRET_FLEET_SERVER_IP,  ///< NVS key "fleet_srv_ip"
    ML_SECRET_OTA_BACKEND_URL,  ///< NVS key "ota_url"
    ML_SECRET_OTA_API_KEY,  ///< NVS key "ota_api_key"
    ML_SECRET_COUNT,
  } ml_secret_t;

  /**
 * @brief Load the secrets partition. Idempotent and thread-safe.
 *
 * Called implicitly by the first ml_secrets_get(); call it earlier to log the
 * provisioning state at a chosen point in boot.
 *
 * @return ESP_OK when the partition was decrypted and read.
 *         ESP_ERR_NVS_SEC_HMAC_KEY_NOT_FOUND when no HMAC key is burned
 *         (unprovisioned unit); ESP_ERR_NOT_FOUND when the partition table has
 *         no "secrets" partition; any other NVS error from the read.
 *         On every error each secret reads as "".
 */
  esp_err_t ml_secrets_init(void);

  /**
 * @brief A provisioned secret.
 *
 * @param id which secret; out-of-range reads as "".
 * @return NUL-terminated value, "" when not provisioned. Valid for the life of
 *         the program; never NULL.
 */
  const char * ml_secrets_get(ml_secret_t id);

#ifdef __cplusplus
}
#endif
