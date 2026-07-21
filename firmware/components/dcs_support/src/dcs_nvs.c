// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dcs_nvs.c
 * @brief All NVS read/write for the dcs_app namespace.
 *
 * Keys (namespace = "dcs_app"):
 *   usb_en    u8   USB-NCM tether enabled (default 1)
 *   ts_boot   u8   Tailscale active at boot (default 0)
 *   boot_cnt  u16  CRASH-class boots since last age-out
 *   ps_ip     u32  pstop peer IPv4 in host byte order
 *   ps_port   u16  pstop peer UDP port
 */

#include <string.h>

#include "dcs_internal.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"

static const char * TAG = "dcs_nvs";

bool dcs_nvs_read_usb_enabled(void)
{
  nvs_handle_t h;
  if (nvs_open(DCS_NVS_NS, NVS_READONLY, &h) != ESP_OK) return true;
  uint8_t v = 1;
  (void)nvs_get_u8(h, DCS_NVS_KEY_USB_EN, &v); /* absent -> default */
  nvs_close(h);
  return v != 0u;
}

esp_err_t dcs_nvs_write_usb_enabled(bool enable)
{
  nvs_handle_t h;
  esp_err_t r = nvs_open(DCS_NVS_NS, NVS_READWRITE, &h);
  if (r != ESP_OK) return r;
  r = nvs_set_u8(h, DCS_NVS_KEY_USB_EN, enable ? 1 : 0);
  if (r == ESP_OK) {
    r = nvs_commit(h);
  }
  nvs_close(h);
  return r;
}

bool dcs_nvs_read_ts_boot_en(void)
{
  nvs_handle_t h;
  if (nvs_open(DCS_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
  uint8_t v = 0;
  (void)nvs_get_u8(h, DCS_NVS_KEY_TS_BOOT_EN, &v); /* absent -> default */
  nvs_close(h);
  return v != 0u;
}

esp_err_t dcs_nvs_write_ts_boot_en(bool enable)
{
  nvs_handle_t h;
  esp_err_t r = nvs_open(DCS_NVS_NS, NVS_READWRITE, &h);
  if (r != ESP_OK) return r;
  r = nvs_set_u8(h, DCS_NVS_KEY_TS_BOOT_EN, enable ? 1 : 0);
  if (r == ESP_OK) {
    r = nvs_commit(h);
  }
  nvs_close(h);
  return r;
}

uint16_t dcs_nvs_read_boot_count(void)
{
  nvs_handle_t h;
  if (nvs_open(DCS_NVS_NS, NVS_READONLY, &h) != ESP_OK) return 0;
  uint16_t v = 0;
  (void)nvs_get_u16(h, DCS_NVS_KEY_BOOT_COUNT, &v); /* absent -> default */
  nvs_close(h);
  return v;
}

esp_err_t dcs_nvs_write_boot_count(uint16_t v)
{
  nvs_handle_t h;
  esp_err_t r = nvs_open(DCS_NVS_NS, NVS_READWRITE, &h);
  if (r != ESP_OK) return r;
  r = nvs_set_u16(h, DCS_NVS_KEY_BOOT_COUNT, v);
  if (r == ESP_OK) {
    r = nvs_commit(h);
  }
  nvs_close(h);
  return r;
}

uint32_t dcs_nvs_read_pstop_peer_ip(void)
{
  nvs_handle_t h;
  if (nvs_open(DCS_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
    return DCS_PSTOP_PEER_DEFAULT_IP;
  }
  uint32_t v = DCS_PSTOP_PEER_DEFAULT_IP;
  (void)nvs_get_u32(h, DCS_NVS_KEY_PSTOP_IP, &v); /* absent -> default */
  nvs_close(h);
  return v;
}

uint16_t dcs_nvs_read_pstop_peer_port(void)
{
  nvs_handle_t h;
  if (nvs_open(DCS_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
    return DCS_PSTOP_PEER_DEFAULT_PORT;
  }
  uint16_t v = DCS_PSTOP_PEER_DEFAULT_PORT;
  (void)nvs_get_u16(h, DCS_NVS_KEY_PSTOP_PORT, &v); /* absent -> default */
  nvs_close(h);
  return v;
}

esp_err_t dcs_nvs_write_pstop_peer(uint32_t ip, uint16_t port)
{
  nvs_handle_t h;
  esp_err_t r = nvs_open(DCS_NVS_NS, NVS_READWRITE, &h);
  if (r != ESP_OK) return r;
  r = nvs_set_u32(h, DCS_NVS_KEY_PSTOP_IP, ip);
  if (r == ESP_OK) {
    r = nvs_set_u16(h, DCS_NVS_KEY_PSTOP_PORT, port);
  }
  if (r == ESP_OK) {
    r = nvs_commit(h);
  }
  nvs_close(h);
  return r;
}

void dcs_nvs_push_reset_reason(uint8_t reason)
{
  nvs_handle_t h;
  if (nvs_open(DCS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
  uint8_t hist[DCS_RST_HIST_LEN] = {0};
  size_t len = sizeof(hist);
  (void)nvs_get_blob(h, DCS_NVS_KEY_RST_HIST, hist, &len); /* absent -> stays zeroed */
  (void)memmove(hist, hist + 1, DCS_RST_HIST_LEN - 1); /* drop oldest */
  hist[DCS_RST_HIST_LEN - 1] = reason; /* newest at the end */
  if (nvs_set_blob(h, DCS_NVS_KEY_RST_HIST, hist, sizeof(hist)) == ESP_OK) {
    if (nvs_commit(h) != ESP_OK) {
      ESP_LOGW(TAG, "reset-history commit failed");
    }
  }
  nvs_close(h);
}

uint8_t dcs_nvs_read_pstop_unit_num(void)
{
  nvs_handle_t h;
  if (nvs_open(DCS_NVS_NS, NVS_READONLY, &h) != ESP_OK) return 0;
  uint8_t v = 0;
  (void)nvs_get_u8(h, DCS_NVS_KEY_PSTOP_NUM, &v); /* absent -> default */
  nvs_close(h);
  return v;
}

esp_err_t dcs_nvs_write_pstop_unit_num(uint8_t n)
{
  nvs_handle_t h;
  esp_err_t r = nvs_open(DCS_NVS_NS, NVS_READWRITE, &h);
  if (r != ESP_OK) return r;
  r = nvs_set_u8(h, DCS_NVS_KEY_PSTOP_NUM, n);
  if (r == ESP_OK) {
    r = nvs_commit(h);
  }
  nvs_close(h);
  return r;
}

int dcs_nvs_read_reset_history(uint8_t * out, int max)
{
  if ((out == NULL) || (max <= 0)) return 0;
  nvs_handle_t h;
  if (nvs_open(DCS_NVS_NS, NVS_READONLY, &h) != ESP_OK) return 0;
  uint8_t hist[DCS_RST_HIST_LEN] = {0};
  size_t len = sizeof(hist);
  esp_err_t r = nvs_get_blob(h, DCS_NVS_KEY_RST_HIST, hist, &len);
  nvs_close(h);
  if (r != ESP_OK) return 0;
  int n = (max < DCS_RST_HIST_LEN) ? max : DCS_RST_HIST_LEN;
  (void)memcpy(out, hist, (size_t)n);
  return n;
}
