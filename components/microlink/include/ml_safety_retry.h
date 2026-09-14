// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Fixed-lifetime health registry. IP zero means unregistered; DERP reads only
 * atomic membership, not the WG owner's liveness flag. */
typedef struct
{
  _Atomic uint32_t ip;
  volatile bool healthy;
} ml_health_ent_t;

static inline bool ml_health_peer_registered(const ml_health_ent_t * peers, size_t count, uint32_t vpn_ip)
{
  if (vpn_ip == 0) return false;
  for (size_t i = 0; i < count; i++) {
    if (atomic_load_explicit(&peers[i].ip, memory_order_relaxed) == vpn_ip) return true;
  }
  return false;
}

static inline bool ml_safety_peer_matches(
  uint32_t priority_peer_ip, const ml_health_ent_t * peers, size_t count, uint32_t vpn_ip)
{
  return vpn_ip != 0 && (vpn_ip == priority_peer_ip || ml_health_peer_registered(peers, count, vpn_ip));
}

/* Ordinary reachability pins and current health do not select this policy. */
static inline bool ml_safety_peers_present(
  uint32_t priority_peer_ip, const ml_health_ent_t * health_peers, size_t count)
{
  if (priority_peer_ip != 0) return true;
  for (size_t i = 0; i < count; i++) {
    if (atomic_load_explicit(&health_peers[i].ip, memory_order_relaxed) != 0) return true;
  }
  return false;
}

/* Retry intervals, not safety timeouts or connect-completion bounds. */
static inline uint32_t ml_derp_retry_gap_ms(bool has_safety_peer, uint32_t retry_burst)
{
  if (!has_safety_peer) return 60000u;
  return retry_burst < 2 ? 5000u : retry_burst < 3 ? 10000u : 60000u;
}

static inline bool ml_derp_retry_due(
  uint64_t now_ms, uint64_t last_retry_ms, bool has_safety_peer, uint32_t retry_burst)
{
  return now_ms - last_retry_ms > ml_derp_retry_gap_ms(has_safety_peer, retry_burst);
}

/* Preserve the existing attempt-based fallback and locked-home guards. */
static inline bool ml_derp_home_fallback_allowed(
  bool enough_attempts, uint16_t region_override, uint16_t priority_peer_region, bool fallback_available)
{
  return enough_attempts && region_override == 0 && priority_peer_region == 0 && fallback_available;
}

static inline int ml_derp_rescue_aux_slot(
  bool enough_attempts, bool fallback_available, bool any_connected, int pending_slot, int free_slot)
{
  return enough_attempts && fallback_available && !any_connected && pending_slot < 0 ? free_slot : -1;
}
