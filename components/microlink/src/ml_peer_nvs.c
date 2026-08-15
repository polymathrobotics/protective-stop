// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ml_peer_nvs.c
 * @brief Peer NVS Persistence - Cache peers across reboots
 *
 * Stores peer data (public key, disco key, VPN IP, endpoints) in NVS flash.
 * On boot, wg_mgr loads cached peers immediately so DISCO probing can start
 * before the control plane registration completes.
 *
 * Storage: single NVS blob containing a packed array of peer entries.
 * Supports up to ML_NVS_MAX_PEERS (64) with LRU eviction when full.
 * The entire table is written as one blob (~5.8KB max) for efficiency.
 *
 * Reference: microlink v1 microlink_peer_registry.c
 */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "microlink_internal.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char * TAG = "ml_peer_nvs";

#define PEER_NVS_NAMESPACE "ml_peers"
#define PEER_NVS_BLOB_KEY "tbl"
#ifdef CONFIG_ML_NVS_MAX_PEERS
  #define ML_NVS_MAX_PEERS CONFIG_ML_NVS_MAX_PEERS
#else
  #define ML_NVS_MAX_PEERS 64 /* Fallback default */
#endif

/* Compact peer storage (92 bytes per entry) */
typedef struct __attribute__((packed))
{
  uint32_t vpn_ip; /* 4 bytes */
  uint8_t public_key[32]; /* 32 bytes */
  uint8_t disco_key[32]; /* 32 bytes */
  uint16_t derp_region; /* 2 bytes */

  struct
  {
    uint32_t ip; /* 4 bytes */
    uint16_t port; /* 2 bytes */
  } __attribute__((packed)) endpoints[2]; /* 12 bytes */

  uint8_t endpoint_count; /* 1 byte */
  char hostname_short[7]; /* 7 bytes (truncated) */
  uint16_t lru_counter; /* 2 bytes — higher = more recently used */
} peer_nvs_entry_t; /* Total: 92 bytes */

/* In-memory table (loaded from NVS blob) */
typedef struct __attribute__((packed))
{
  uint16_t count; /* Number of valid entries */
  uint16_t lru_clock; /* Monotonic LRU counter */
  peer_nvs_entry_t entries[ML_NVS_MAX_PEERS];
} peer_nvs_table_t;

static nvs_handle_t s_nvs = 0;
static bool s_initialized = false;
static peer_nvs_table_t * s_table = NULL; /* PSRAM-allocated working copy */
static bool s_dirty = false; /* working copy ahead of flash */
static uint64_t s_last_flush_ms = 0;
/* Never-LRU-evicted VPN IPs: the priority/safety peer, the fleet server and
 * (via ml_wg_mgr's save path) app-pinned operator remotes. These are exactly
 * the keys the boot-time WG preseed needs to answer inbound handshakes after
 * a reboot with the netmap still absent (cold-bond far-side gap, 2026-08-08)
 * — losing one to cache pressure would silently re-open that hole. RAM-only:
 * eviction only ever happens inside runtime saves, when the pin sources are
 * live to re-mark them. Bounded; sized to ML_EXTRA_PINS + priority + fleet. */
#define ML_NVS_MAX_PROTECTED 18
static uint32_t s_protected_ips[ML_NVS_MAX_PROTECTED];
static int s_protected_count = 0;
#define PEER_NVS_FLUSH_INTERVAL_MS 5000u

static void load_table(void)
{
  if (!s_table) {
    s_table = ml_psram_calloc(1, sizeof(peer_nvs_table_t));
    if (!s_table) return;
  }

  size_t len = sizeof(peer_nvs_table_t);
  if (nvs_get_blob(s_nvs, PEER_NVS_BLOB_KEY, s_table, &len) != ESP_OK) {
    memset(s_table, 0, sizeof(peer_nvs_table_t));
  }
}

static esp_err_t flush_table(void)
{
  if (!s_table) return ESP_ERR_INVALID_STATE;

  size_t blob_size = sizeof(uint16_t) * 2 + s_table->count * sizeof(peer_nvs_entry_t);
  esp_err_t err = nvs_set_blob(s_nvs, PEER_NVS_BLOB_KEY, s_table, blob_size);
  if (err == ESP_OK) {
    nvs_commit(s_nvs);
  }
  return err;
}

esp_err_t ml_peer_nvs_init(void)
{
  if (s_initialized) return ESP_OK;

  esp_err_t err = nvs_open(PEER_NVS_NAMESPACE, NVS_READWRITE, &s_nvs);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "NVS open failed: %d (peer cache disabled)", err);
    return err;
  }

  s_initialized = true;
  load_table();

  ESP_LOGI(TAG, "Peer NVS initialized: %d cached peers (max %d)", s_table ? s_table->count : 0, ML_NVS_MAX_PEERS);

  return ESP_OK;
}

void ml_peer_nvs_deinit(void)
{
  if (!s_initialized) return;
  if (s_dirty) {
    (void)flush_table();
    s_dirty = false;
  }
  nvs_close(s_nvs);
  s_initialized = false;
  if (s_table) {
    free(s_table);
    s_table = NULL;
  }
}

esp_err_t ml_peer_nvs_save(const ml_peer_t * peer)
{
  if (!s_initialized || !peer || !s_table) return ESP_ERR_INVALID_STATE;

  /* Advance LRU clock */
  s_table->lru_clock++;

  /* Build entry */
  peer_nvs_entry_t entry;
  memset(&entry, 0, sizeof(entry));

  entry.vpn_ip = peer->vpn_ip;
  memcpy(entry.public_key, peer->public_key, 32);
  memcpy(entry.disco_key, peer->disco_key, 32);
  entry.derp_region = peer->derp_region;
  entry.lru_counter = s_table->lru_clock;

  /* Store up to 2 endpoints */
  int stored = 0;
  for (int i = 0; i < peer->endpoint_count && stored < 2; i++) {
    if (!peer->endpoints[i].is_ipv6 && peer->endpoints[i].ip != 0) {
      entry.endpoints[stored].ip = peer->endpoints[i].ip;
      entry.endpoints[stored].port = peer->endpoints[i].port;
      stored++;
    }
  }
  entry.endpoint_count = stored;

  /* Truncated hostname for display (snprintf: always NUL-terminated) */
  strlcpy(entry.hostname_short, peer->hostname, sizeof(entry.hostname_short));

  /* Find existing entry by VPN IP (update) or public key (re-keyed) */
  int slot = -1;
  for (int i = 0; i < s_table->count; i++) {
    if (s_table->entries[i].vpn_ip == peer->vpn_ip || memcmp(s_table->entries[i].public_key, peer->public_key, 32) == 0)
    {
      slot = i;
      break;
    }
  }

  if (slot >= 0) {
    /* Update existing entry */
    s_table->entries[slot] = entry;
  } else if (s_table->count < ML_NVS_MAX_PEERS) {
    /* Append new entry */
    slot = s_table->count;
    s_table->entries[slot] = entry;
    s_table->count++;
  } else {
    /* LRU eviction: lowest lru_counter, but never a protected peer —
         * priority/fleet/operator keys are what the boot-time WG preseed
         * answers cold handshakes with, and the priority peer's cached
         * endpoint is what makes the pstop direct path re-form fast. */
    int lru_idx = -1;
    uint16_t min_lru = 0;
    for (int i = 0; i < s_table->count; i++) {
      bool prot = false;
      for (int k = 0; k < s_protected_count; k++) {
        if (s_table->entries[i].vpn_ip == s_protected_ips[k]) {
          prot = true;
          break;
        }
      }
      if (prot) {
        continue;
      }
      if (lru_idx < 0 || s_table->entries[i].lru_counter < min_lru) {
        min_lru = s_table->entries[i].lru_counter;
        lru_idx = i;
      }
    }
    if (lru_idx < 0) {
      lru_idx = 0;
    } /* all protected (degenerate) */
    ESP_LOGI(TAG, "LRU evict: %s (slot=%d lru=%d)", s_table->entries[lru_idx].hostname_short, lru_idx, min_lru);
    s_table->entries[lru_idx] = entry;
    slot = lru_idx;
  }

  /* Debounced: first sight of a 100-peer tailnet used to rewrite the
     * growing blob once PER PEER (~465 KB cumulative flash writes into a
     * 24 KB partition), stalling the flash cache for both cores while the
     * pstop heartbeats waited. The working copy is authoritative; flash
     * catches up via ml_peer_nvs_flush_if_due() at most every 5 s. */
  s_dirty = true;
  ESP_LOGD(TAG, "Saved peer %s (slot=%d/%d, flush deferred)", entry.hostname_short, slot, s_table->count);
  return ESP_OK;
}

void ml_peer_nvs_set_protected(uint32_t vpn_ip)
{
  if (vpn_ip == 0) {
    return;
  }
  for (int i = 0; i < s_protected_count; i++) {
    if (s_protected_ips[i] == vpn_ip) {
      return; /* already protected */
    }
  }
  if (s_protected_count >= ML_NVS_MAX_PROTECTED) {
    /* Bounded set full — drop, don't wrap: wrongly UNprotecting an earlier
         * safety peer would be worse than best-efforting this one. */
    ESP_LOGW(TAG, "protected-peer set full (%d), not protecting new IP", ML_NVS_MAX_PROTECTED);
    return;
  }
  s_protected_ips[s_protected_count++] = vpn_ip;
}

/* Flush timing diag (§7 R6): the flash commit suspends flash-resident
 * execution on BOTH cores — these gauges make that cost measurable and the
 * fix falsifiable (flush_count flat in steady state = §7a working). */
static uint32_t s_diag_flush_last_ms;
static uint32_t s_diag_flush_max_ms;
static uint32_t s_diag_flush_count;

void ml_peer_nvs_get_flush_diag(uint32_t out[3])
{
  out[0] = s_diag_flush_last_ms;
  out[1] = s_diag_flush_max_ms;
  out[2] = s_diag_flush_count;
}

esp_err_t ml_peer_nvs_flush_if_due(uint64_t now_ms, bool ingest_busy)
{
  if (!s_initialized || !s_dirty) {
    return ESP_OK;
  }
  if ((now_ms - s_last_flush_ms) < PEER_NVS_FLUSH_INTERVAL_MS) {
    return ESP_OK;
  }
  /* §7b quiet-gate: an ingest burst is the worst moment to suspend the
   * cores for a flash commit — defer while updates are pending, CAPPED so
   * a sustained storm cannot starve durability (the boot preseed for the
   * cold-bond far-side gap depends on this cache). */
  if (ingest_busy && (now_ms - s_last_flush_ms) < ML_PEER_NVS_FLUSH_MAX_DEFER_MS) {
    return ESP_OK;
  }
  int64_t t0 = esp_timer_get_time();
  esp_err_t r = flush_table();
  uint32_t dur = (uint32_t)((esp_timer_get_time() - t0) / 1000);
  s_diag_flush_last_ms = dur;
  if (dur > s_diag_flush_max_ms) s_diag_flush_max_ms = dur;
  s_diag_flush_count++;
  s_last_flush_ms = now_ms;
  if (r == ESP_OK) {
    s_dirty = false; /* keep the dirt on a failed flush */
  }
  return r;
}

int ml_peer_nvs_load_all(ml_peer_t * peers, int max_peers)
{
  if (!s_initialized || !peers || !s_table) return 0;
  if (s_table->count == 0) return 0;

  int loaded = 0;
  for (int i = 0; i < s_table->count && loaded < max_peers; i++) {
    peer_nvs_entry_t * entry = &s_table->entries[i];
    if (entry->vpn_ip == 0) continue;

    ml_peer_t * p = &peers[loaded];
    memset(p, 0, sizeof(ml_peer_t));

    p->vpn_ip = entry->vpn_ip;
    memcpy(p->public_key, entry->public_key, 32);
    memcpy(p->disco_key, entry->disco_key, 32);
    p->derp_region = entry->derp_region;
    p->active = true;
    p->wg_peer_index = -1;

    /* Restore hostname (truncated, ensure null-terminated) */
    memcpy(p->hostname, entry->hostname_short, sizeof(entry->hostname_short));
    p->hostname[sizeof(entry->hostname_short)] = '\0';

    /* Restore endpoints */
    p->endpoint_count = entry->endpoint_count;
    for (int j = 0; j < entry->endpoint_count && j < 2; j++) {
      p->endpoints[j].ip = entry->endpoints[j].ip;
      p->endpoints[j].port = entry->endpoints[j].port;
      p->endpoints[j].is_ipv6 = false;
    }

    loaded++;

    char ip_str[16];
    microlink_ip_to_str(p->vpn_ip, ip_str);
    ESP_LOGI(TAG, "Loaded cached peer: %s (%s)", p->hostname, ip_str);
  }

  ESP_LOGI(TAG, "Loaded %d cached peers from NVS", loaded);
  return loaded;
}

esp_err_t ml_peer_nvs_clear(void)
{
  if (!s_initialized) return ESP_ERR_INVALID_STATE;

  esp_err_t err = nvs_erase_all(s_nvs);
  if (err == ESP_OK) {
    nvs_commit(s_nvs);
    if (s_table) memset(s_table, 0, sizeof(peer_nvs_table_t));
    ESP_LOGI(TAG, "Peer NVS cleared");
  }
  return err;
}
