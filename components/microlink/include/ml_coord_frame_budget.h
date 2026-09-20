// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Wall-clock budget for finishing ONE partial coordination (Noise) frame —
 * pure, host-testable (host/test_coord_frame_budget.c). Used by
 * ml_coord.c coord_recv_ex().
 *
 * Issue #127: the old bound was a retry COUNT (300 x 10 ms, "~3 s"), but each
 * retry first blocked for the socket's SO_RCVTIMEO — 2 s in long-poll, 60 s
 * while fetching the MapResponse — so a stalled-but-open peer held the coord
 * task for 300 x 2 s = 10 min, starving the ML_CTRL_WATCHDOG_MS (120 s) check
 * that only runs between reads.
 *
 * Two limits, both counted from the FIRST byte of the frame:
 *  - NO-PROGRESS window (10 s): every byte received restarts it. A stalled
 *    peer (TCP black hole, dead relay) is abandoned 10 s after its last byte —
 *    the #127 case — while a slow-but-live link keeps its frame as long as
 *    bytes keep arriving. Bytes arriving is exactly the liveness the control
 *    watchdog wants to see, so extending on progress does not hide a dead
 *    control plane from it.
 *  - HARD CAP scaled by frame size: max(10 s, frame_len / 1 KiB/s). Control
 *    frames (<= 4 KB) get the 10 s floor; a full 64 KB MapResponse frame gets
 *    64 s, i.e. a link must sustain ~8 kbit/s to keep it. Below that the
 *    5 Hz safety heartbeat is marginal anyway, and the cap bounds how long a
 *    trickling peer can hold the coord task (previously unbounded).
 *  - Not armed (no byte consumed yet) = an idle stream, not a partial frame:
 *    the caller keeps using the socket timeout and may simply retry later.
 *  - Once armed, one blocking wait may last at most the time to the nearer of
 *    the two deadlines — never the socket's own timeout.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ML_COORD_FRAME_NO_PROGRESS_MS 10000u /* abandon after this long without a byte */
#define ML_COORD_FRAME_MIN_BUDGET_MS 10000u /* hard-cap floor (control frames <= 4 KB) */
#define ML_COORD_FRAME_MIN_RATE_BPS 1024u /* bytes/s a live link must sustain for the hard cap */

typedef struct
{
  bool armed;
  int64_t total_us; /* hard cap length for this frame (from frame_len) */
  int64_t hard_deadline_us; /* first byte + total_us */
  int64_t progress_deadline_us; /* last byte + NO_PROGRESS */
} ml_coord_frame_budget_t;

/* Hard cap for a frame of frame_len bytes, in ms. */
static inline uint32_t ml_coord_frame_budget_total_ms(size_t frame_len)
{
  uint64_t by_size = ((uint64_t)frame_len * 1000u) / ML_COORD_FRAME_MIN_RATE_BPS;
  return (by_size > ML_COORD_FRAME_MIN_BUDGET_MS) ? (uint32_t)by_size : ML_COORD_FRAME_MIN_BUDGET_MS;
}

/* frame_len = total bytes this read must complete (the caller's len). */
static inline void ml_coord_frame_budget_init(ml_coord_frame_budget_t * b, size_t frame_len)
{
  b->armed = false;
  b->total_us = (int64_t)ml_coord_frame_budget_total_ms(frame_len) * 1000LL;
  b->hard_deadline_us = 0;
  b->progress_deadline_us = 0;
}

/* Call after every read that consumed n > 0 bytes: arms on the first byte,
 * restarts the no-progress window on every byte, never moves the hard cap. */
static inline void ml_coord_frame_budget_on_bytes(ml_coord_frame_budget_t * b, int64_t now_us)
{
  if (!b->armed) {
    b->armed = true;
    b->hard_deadline_us = now_us + b->total_us;
  }
  b->progress_deadline_us = now_us + (int64_t)ML_COORD_FRAME_NO_PROGRESS_MS * 1000LL;
}

static inline bool ml_coord_frame_budget_armed(const ml_coord_frame_budget_t * b)
{
  return b->armed;
}

static inline int64_t ml_coord_frame_budget_deadline_us(const ml_coord_frame_budget_t * b)
{
  return (b->progress_deadline_us < b->hard_deadline_us) ? b->progress_deadline_us : b->hard_deadline_us;
}

/* True once an armed budget has run out (either limit): the frame must be abandoned. */
static inline bool ml_coord_frame_budget_expired(const ml_coord_frame_budget_t * b, int64_t now_us)
{
  return b->armed && now_us >= ml_coord_frame_budget_deadline_us(b);
}

/* Longest one blocking wait for more bytes may last. 0 when not armed or expired. */
static inline int64_t ml_coord_frame_budget_wait_us(const ml_coord_frame_budget_t * b, int64_t now_us)
{
  if (!b->armed) return 0;
  int64_t rem = ml_coord_frame_budget_deadline_us(b) - now_us;
  return rem > 0 ? rem : 0;
}
