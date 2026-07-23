// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dcs_nvs.c
 * @brief All NVS read/write for the dcs_app namespace.
 *
 * Keys (namespace = "dcs_app"):
 *   usb_en    u8   USB-NCM tether enabled (default 1)
 *   ts_boot   u8   Tailscale active at boot (default 1: fresh units auto-join)
 *   boot_cnt  u16  CRASH-class boots since last age-out
 *   ps_ip     u32  pstop peer IPv4 in host byte order
 *   ps_port   u16  pstop peer UDP port
 *   ring_off  u8   LED-ring rotation: physical pixel index of LED 1 (default 0)
 *   ps_peers  blob multi-machine peer table: version byte + per-slot records
 *                  (absent -> migrate legacy ps_ip/ps_port into slot 0)
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
  uint8_t v = 1; /* absent -> default ON: fresh fleet units auto-join Tailscale */
  (void)nvs_get_u8(h, DCS_NVS_KEY_TS_BOOT_EN, &v); /* explicit 0 still honored */
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

uint8_t dcs_nvs_read_ring_offset(void)
{
  nvs_handle_t h;
  if (nvs_open(DCS_NVS_NS, NVS_READONLY, &h) != ESP_OK) return 0;
  uint8_t v = 0;
  (void)nvs_get_u8(h, DCS_NVS_KEY_RING_OFF, &v); /* absent -> default */
  nvs_close(h);
  return (uint8_t)(v & 0x0Fu); /* corrupt value degrades to a valid rotation */
}

esp_err_t dcs_nvs_write_ring_offset(uint8_t off)
{
  nvs_handle_t h;
  esp_err_t r = nvs_open(DCS_NVS_NS, NVS_READWRITE, &h);
  if (r != ESP_OK) return r;
  r = nvs_set_u8(h, DCS_NVS_KEY_RING_OFF, (uint8_t)(off & 0x0Fu));
  if (r == ESP_OK) {
    r = nvs_commit(h);
  }
  nvs_close(h);
  return r;
}

/* ps_peers blob layout (byte-serialized, no struct padding on the wire):
 *   [0]            format version (1)
 *   per slot, DCS_PSTOP_MAX_MACHINES records of 11 bytes:
 *   [0]            used (0/1)
 *   [1..4]         ip, big-endian, host-order value
 *   [5..6]         port, big-endian
 *   [7..10]        machine_id, big-endian
 */
#define PS_PEERS_VER 1u
#define PS_PEERS_REC_LEN 11
#define PS_PEERS_BLOB_LEN (1 + (DCS_PSTOP_MAX_MACHINES * PS_PEERS_REC_LEN))

static void ps_peers_put_u32(uint8_t * p, uint32_t v)
{
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);
  p[3] = (uint8_t)v;
}

static uint32_t ps_peers_get_u32(const uint8_t * p)
{
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

void dcs_nvs_read_pstop_peers(dcs_pstop_peer_rec_t out[DCS_PSTOP_MAX_MACHINES])
{
  (void)memset(out, 0, DCS_PSTOP_MAX_MACHINES * sizeof(out[0]));

  uint8_t blob[PS_PEERS_BLOB_LEN] = {0};
  size_t len = sizeof(blob);
  nvs_handle_t h;
  esp_err_t r = ESP_FAIL;
  if (nvs_open(DCS_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
    r = nvs_get_blob(h, DCS_NVS_KEY_PSTOP_PEERS, blob, &len);
    nvs_close(h);
  }

  if ((r == ESP_OK) && (len == sizeof(blob)) && (blob[0] == PS_PEERS_VER)) {
    for (int i = 0; i < DCS_PSTOP_MAX_MACHINES; i++) {
      const uint8_t * rec = &blob[1 + (i * PS_PEERS_REC_LEN)];
      out[i].configured = (rec[0] != 0u);
      out[i].ip = ps_peers_get_u32(&rec[1]);
      out[i].port = (uint16_t)(((uint16_t)rec[5] << 8) | (uint16_t)rec[6]);
      out[i].machine_id = ps_peers_get_u32(&rec[7]);
      if ((out[i].ip == 0u) || (out[i].port == 0u)) {
        out[i].configured = false; /* corrupt/cleared record degrades to empty */
      }
    }
    return;
  }

  /* Blob absent (first boot on this firmware) or unreadable: migrate the
   * legacy single peer into slot 0 so existing installs keep working. */
  out[0].configured = true;
  out[0].ip = dcs_nvs_read_pstop_peer_ip();
  out[0].port = dcs_nvs_read_pstop_peer_port();
  out[0].machine_id = DCS_PSTOP_DEFAULT_MACHINE_ID;
}

esp_err_t dcs_nvs_write_pstop_peers(const dcs_pstop_peer_rec_t recs[DCS_PSTOP_MAX_MACHINES])
{
  uint8_t blob[PS_PEERS_BLOB_LEN] = {0};
  blob[0] = PS_PEERS_VER;
  for (int i = 0; i < DCS_PSTOP_MAX_MACHINES; i++) {
    uint8_t * rec = &blob[1 + (i * PS_PEERS_REC_LEN)];
    rec[0] = recs[i].configured ? 1u : 0u;
    ps_peers_put_u32(&rec[1], recs[i].ip);
    rec[5] = (uint8_t)(recs[i].port >> 8);
    rec[6] = (uint8_t)recs[i].port;
    ps_peers_put_u32(&rec[7], recs[i].machine_id);
  }

  nvs_handle_t h;
  esp_err_t r = nvs_open(DCS_NVS_NS, NVS_READWRITE, &h);
  if (r != ESP_OK) return r;
  r = nvs_set_blob(h, DCS_NVS_KEY_PSTOP_PEERS, blob, sizeof(blob));
  if ((r == ESP_OK) && recs[0].configured) {
    /* Mirror slot 0 to the legacy keys so a firmware ROLLBACK (old image
     * reads only ps_ip/ps_port) still heartbeats its primary machine. */
    r = nvs_set_u32(h, DCS_NVS_KEY_PSTOP_IP, recs[0].ip);
    if (r == ESP_OK) {
      r = nvs_set_u16(h, DCS_NVS_KEY_PSTOP_PORT, recs[0].port);
    }
  }
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
