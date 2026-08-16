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
 *   operators blob operator allowlist: count byte + u32 ids
 *   wifi_txp  u8   WiFi max TX power, quarter-dBm (8..84); 0/absent = config default
 *   led_bri   u8   master LED brightness, 0..100%; absent/corrupt = default 50
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
  uint8_t v = 1; /* absent -> default ON: fresh units auto-join Tailscale */
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

void dcs_nvs_set_ctrl_reset_cause(uint8_t cause)
{
  nvs_handle_t h;
  if (nvs_open(DCS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
  if (nvs_set_u8(h, "ctrl_rst", cause) == ESP_OK) {
    (void)nvs_commit(h);
  }
  nvs_close(h);
}

uint8_t dcs_nvs_take_ctrl_reset_cause(void)
{
  nvs_handle_t h;
  if (nvs_open(DCS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return 0;
  uint8_t v = 0;
  if (nvs_get_u8(h, "ctrl_rst", &v) == ESP_OK) {
    (void)nvs_erase_key(h, "ctrl_rst"); /* one-shot: stale crumbs must not
                                         * mislabel a later POWERON/panic */
    (void)nvs_commit(h);
  }
  nvs_close(h);
  return v;
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

uint8_t dcs_nvs_read_wifi_tx_power(void)
{
  nvs_handle_t h;
  if (nvs_open(DCS_NVS_NS, NVS_READONLY, &h) != ESP_OK) return 0;
  uint8_t v = 0; /* absent -> 0 = "use config default" */
  (void)nvs_get_u8(h, DCS_NVS_KEY_WIFI_TXP, &v);
  nvs_close(h);
  /* Reject out-of-range persisted values (corruption / older schema) rather
   * than pushing an invalid level into esp_wifi_set_max_tx_power. */
  if ((v < 8u) || (v > 84u)) return 0;
  return v;
}

esp_err_t dcs_nvs_write_wifi_tx_power(uint8_t quarter_dbm)
{
  if ((quarter_dbm < 8u) || (quarter_dbm > 84u)) return ESP_ERR_INVALID_ARG;
  nvs_handle_t h;
  esp_err_t r = nvs_open(DCS_NVS_NS, NVS_READWRITE, &h);
  if (r != ESP_OK) return r;
  r = nvs_set_u8(h, DCS_NVS_KEY_WIFI_TXP, quarter_dbm);
  if (r == ESP_OK) {
    r = nvs_commit(h);
  }
  nvs_close(h);
  return r;
}

uint8_t dcs_nvs_read_led_brightness(void)
{
  nvs_handle_t h;
  if (nvs_open(DCS_NVS_NS, NVS_READONLY, &h) != ESP_OK) return DCS_LED_BRIGHTNESS_DEFAULT;
  uint8_t v = DCS_LED_BRIGHTNESS_DEFAULT;
  (void)nvs_get_u8(h, DCS_NVS_KEY_LED_BRIGHT, &v); /* absent -> default */
  nvs_close(h);
  if (v > 100u) return DCS_LED_BRIGHTNESS_DEFAULT; /* corrupt/older schema degrades to default */
  /* Floor self-heals a persisted sub-floor value (e.g. 0 from a pre-floor
   * build) so the ring can never boot dark. */
  return dcs_led_brightness_floor(v);
}

esp_err_t dcs_nvs_write_led_brightness(uint8_t pct)
{
  if (pct > 100u) return ESP_ERR_INVALID_ARG;
  nvs_handle_t h;
  esp_err_t r = nvs_open(DCS_NVS_NS, NVS_READWRITE, &h);
  if (r != ESP_OK) return r;
  r = nvs_set_u8(h, DCS_NVS_KEY_LED_BRIGHT, pct);
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

  /* Blob absent (first boot on this firmware) or unreadable: migrate a
   * legacy single peer into slot 0 so existing installs keep working — but
   * ONLY if one was actually persisted. A blank NVS (fresh production flash,
   * fleet-only remote with no machine yet) has no ps_ip key; the legacy
   * readers would substitute a hardcoded DEFAULT ip/port, fabricating a
   * configured-but-unreachable machine slot that lights the safety ring
   * yellow (UNREACHABLE) instead of white (IDLE). So probe for the real key:
   * migrate only when it exists, otherwise leave the table zeroed = no
   * machine configured. */
  nvs_handle_t lh;
  bool have_legacy = false;
  uint32_t legacy_ip = 0;
  if (nvs_open(DCS_NVS_NS, NVS_READONLY, &lh) == ESP_OK) {
    /* Value check, not key-exists: the writer mirrors a slot-0 CLEAR as
     * ip=0, and migrating that would fabricate a configured 0.0.0.0 peer. */
    have_legacy = (nvs_get_u32(lh, DCS_NVS_KEY_PSTOP_IP, &legacy_ip) == ESP_OK) && legacy_ip != 0;
    nvs_close(lh);
  }
  if (have_legacy) {
    out[0].configured = true;
    out[0].ip = legacy_ip;
    out[0].port = dcs_nvs_read_pstop_peer_port();
    out[0].machine_id = DCS_PSTOP_DEFAULT_MACHINE_ID;
  }
  /* else: no legacy peer -> whole table stays zeroed (no machine configured). */
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
  if (r == ESP_OK) {
    /* Mirror slot 0 to the legacy keys UNCONDITIONALLY so a firmware ROLLBACK
     * (old image reads only ps_ip/ps_port) tracks slot 0 exactly — including a
     * CLEAR (ip=0/port=0 when unconfigured). Gating on configured left stale
     * legacy keys behind /api/pstop_peer?clear=1, silently resurrecting the
     * cleared peer after a rollback. */
    r = nvs_set_u32(h, DCS_NVS_KEY_PSTOP_IP, recs[0].configured ? recs[0].ip : 0u);
    if (r == ESP_OK) {
      r = nvs_set_u16(h, DCS_NVS_KEY_PSTOP_PORT, recs[0].configured ? recs[0].port : 0u);
    }
  }
  if (r == ESP_OK) {
    r = nvs_commit(h);
  }
  nvs_close(h);
  return r;
}

/* operators blob layout (byte-serialized):
 *   [0]            count (0..DCS_MAX_OPERATORS)
 *   per id, big-endian u32
 * Absent/corrupt -> empty list (0 operators = every remote stop-only = safe). */
#define OPERATORS_BLOB_LEN (1 + (DCS_MAX_OPERATORS * 4))

int dcs_nvs_read_operators(uint32_t out[DCS_MAX_OPERATORS])
{
  (void)memset(out, 0, DCS_MAX_OPERATORS * sizeof(out[0]));

  uint8_t blob[OPERATORS_BLOB_LEN] = {0};
  size_t len = sizeof(blob);
  nvs_handle_t h;
  esp_err_t r = ESP_FAIL;
  if (nvs_open(DCS_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
    r = nvs_get_blob(h, DCS_NVS_KEY_OPERATORS, blob, &len);
    nvs_close(h);
  }
  if ((r != ESP_OK) || (len < 1u)) {
    return 0; /* blank NVS: empty allowlist */
  }
  int count = blob[0];
  if (count > DCS_MAX_OPERATORS) {
    count = DCS_MAX_OPERATORS; /* corrupt length degrades safely (never over-reads) */
  }
  int n = 0;
  for (int i = 0; i < count; i++) {
    size_t off = (size_t)1 + ((size_t)i * 4u);
    if ((off + 4u) > len) {
      break; /* truncated blob: stop at what we have */
    }
    uint32_t id = ps_peers_get_u32(&blob[off]);
    if (id != 0u) {
      out[n++] = id; /* skip cleared/zero slots */
    }
  }
  return n;
}

esp_err_t dcs_nvs_write_operators(const uint32_t ids[DCS_MAX_OPERATORS], int count)
{
  if ((count < 0) || (count > DCS_MAX_OPERATORS)) {
    return ESP_ERR_INVALID_ARG;
  }
  uint8_t blob[OPERATORS_BLOB_LEN] = {0};
  blob[0] = (uint8_t)count;
  for (int i = 0; i < count; i++) {
    ps_peers_put_u32(&blob[1 + (i * 4)], ids[i]);
  }
  nvs_handle_t h;
  esp_err_t r = nvs_open(DCS_NVS_NS, NVS_READWRITE, &h);
  if (r != ESP_OK) return r;
  r = nvs_set_blob(h, DCS_NVS_KEY_OPERATORS, blob, sizeof(blob));
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
