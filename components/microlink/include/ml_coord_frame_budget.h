// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Wall-clock budget for finishing ONE partial coordination (Noise) frame —
 * pure, host-testable (host/test_coord_frame_budget.c). Used by
 * ml_coord.c coord_recv().
 *
 * Issue #127: the old bound was a retry COUNT (300 x 10 ms, "~3 s"), but each
 * retry first blocked for the socket's SO_RCVTIMEO — 2 s in long-poll, 60 s
 * while fetching the MapResponse — so a stalled-but-open peer held the coord
 * task for 300 x 2 s = 10 min, starving the ML_CTRL_WATCHDOG_MS (120 s) check
 * that only runs between reads.
 *
 * Contract:
 *  - The budget arms on the FIRST byte of a frame. Progress never extends it:
 *    a trickling peer is a stall too.
 *  - Once armed, one blocking wait may last at most the REMAINING budget —
 *    never the socket's own timeout — so a stall costs at most the budget.
 *  - Not armed (no byte consumed yet) = an idle stream, not a partial frame:
 *    the caller keeps using the socket timeout and may simply retry later.
 *  - 10 s = the connect-phase SO_RCVTIMEO, >= 4 whole long-poll socket
 *    timeouts (a lost segment + TCP retransmit backoff fits; control frames
 *    are <= 4 KB) and 1/12 of the watchdog.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ML_COORD_PARTIAL_FRAME_DEADLINE_MS 10000u

typedef struct
{
  bool armed;
  int64_t deadline_us;
} ml_coord_frame_budget_t;

static inline void ml_coord_frame_budget_reset(ml_coord_frame_budget_t * b)
{
  b->armed = false;
  b->deadline_us = 0;
}

/* Call after every read that consumed n > 0 bytes. */
static inline void ml_coord_frame_budget_on_bytes(ml_coord_frame_budget_t * b, int64_t now_us)
{
  if (!b->armed) {
    b->armed = true;
    b->deadline_us = now_us + (int64_t)ML_COORD_PARTIAL_FRAME_DEADLINE_MS * 1000LL;
  }
}

static inline bool ml_coord_frame_budget_armed(const ml_coord_frame_budget_t * b)
{
  return b->armed;
}

/* True once an armed budget has run out: the frame must be abandoned. */
static inline bool ml_coord_frame_budget_expired(const ml_coord_frame_budget_t * b, int64_t now_us)
{
  return b->armed && now_us >= b->deadline_us;
}

/* Longest one blocking wait for more bytes may last. 0 when not armed or expired. */
static inline int64_t ml_coord_frame_budget_wait_us(const ml_coord_frame_budget_t * b, int64_t now_us)
{
  if (!b->armed) return 0;
  int64_t rem = b->deadline_us - now_us;
  return rem > 0 ? rem : 0;
}
