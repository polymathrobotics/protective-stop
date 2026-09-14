// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Per-peer schedule for the relay-stuck coord re-fetch (ML_CMD_FORCE_RECONNECT).
 * Pure; host tests in host/test_relay_refetch.c.
 *
 * Each issued re-fetch doubles that peer's interval up to a cap; the peer's own
 * direct-regain edge resets it. A fleet-wide floor (last re-fetch for ANY peer)
 * still applies because one re-fetch refreshes every peer's endpoints. Call
 * ml_relay_refetch_sent() only after the command was actually enqueued. */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint64_t next_ms; /* 0 = never fetched: due once the floor allows */
  uint64_t interval_ms; /* gap that will separate the next two re-fetches */
} ml_relay_refetch_t;

static inline void ml_relay_refetch_reset(ml_relay_refetch_t * s, uint64_t min_ms)
{
  s->next_ms = 0;
  s->interval_ms = min_ms;
}

static inline bool ml_relay_refetch_due(
  const ml_relay_refetch_t * s, uint64_t now_ms, uint64_t last_any_ms, uint64_t min_ms)
{
  if (s->next_ms != 0 && now_ms < s->next_ms) return false;
  if (last_any_ms != 0 && now_ms - last_any_ms < min_ms) return false;
  return true;
}

/* Record an issued re-fetch: escalate, then arm next_ms with the escalated
 * interval. Returns that interval (ms); next_ms - now_ms == return value. */
static inline uint64_t ml_relay_refetch_sent(ml_relay_refetch_t * s, uint64_t now_ms, uint64_t min_ms, uint64_t max_ms)
{
  uint64_t next = (s->interval_ms ? s->interval_ms : min_ms) * 2u;
  if (next > max_ms) next = max_ms;
  s->interval_ms = next;
  s->next_ms = now_ms + next;
  return next;
}
