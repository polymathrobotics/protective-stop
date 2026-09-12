// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Application health registration, shared by the WG manager's health readers
 * and the DERP task's retry eligibility check. IP zero means unregistered.
 * Membership is atomic because registration/removal and readers run on
 * different tasks. The retry predicate reads membership only, not liveness. */
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

/* Single per-peer membership definition used by both the WG manager and the
 * any-safety-peer query. Reachability pins and the healthy flag are excluded. */
static inline bool ml_safety_peer_matches(
  uint32_t priority_peer_ip, const ml_health_ent_t * peers, size_t count, uint32_t vpn_ip)
{
  return vpn_ip != 0 && (vpn_ip == priority_peer_ip || ml_health_peer_registered(peers, count, vpn_ip));
}

/* A safety target need not be healthy, direct, or admitted to the WG peer
 * table yet. Management/fleet pins alone are not health registrations.
 * The caller supplies a fixed-lifetime array; concurrent membership changes
 * may be observed on this pass or the next, without a cached-count lifetime. */
static inline bool ml_safety_peers_present(
  uint32_t priority_peer_ip, const ml_health_ent_t * health_peers, size_t count)
{
  if (ml_safety_peer_matches(priority_peer_ip, health_peers, count, priority_peer_ip)) return true;
  for (size_t i = 0; i < count; i++) {
    uint32_t ip = atomic_load_explicit(&health_peers[i].ip, memory_order_relaxed);
    if (ml_safety_peer_matches(priority_peer_ip, health_peers, count, ip)) return true;
  }
  return false;
}

/* Existing periodic home-retry ladder. This is a retry interval, not a safety
 * timeout or a bound on connect completion. Immediate reconnect kicks and
 * the owner's burst/reset/due-time bookkeeping remain outside this helper. */
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

/* Existing attempt-based fallback guard. Faster retries can reach the attempt
 * threshold sooner on an unlocked, ownerless health-only device. A LOCK or
 * re-home owner still forbids moving home, even with a proven alternative. */
static inline bool ml_derp_home_fallback_allowed(
  bool enough_attempts, uint16_t region_override, uint16_t priority_peer_region, bool fallback_available)
{
  return enough_attempts && region_override == 0 && priority_peer_region == 0 && fallback_available;
}

/* The existing fallback picker supplies the candidate and free non-home slot.
 * Keep at most one rescue connect pending, and only when the pool is dark. */
static inline int ml_derp_rescue_aux_slot(
  bool enough_attempts, bool fallback_available, bool any_connected, int pending_slot, int free_slot)
{
  return enough_attempts && fallback_available && !any_connected && pending_slot < 0 ? free_slot : -1;
}
