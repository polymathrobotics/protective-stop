// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Host test for ml_coord_frame_budget.h — the wall-clock budget that bounds a
 * partial coordination frame (ml_coord.c coord_recv_ex, issue #127).
 *
 * Pins:
 *  - the #127 regression: a STALLED peer costs 10 s, not 300 x SO_RCVTIMEO;
 *  - a slow-but-live frame is kept while bytes arrive (no-progress window
 *    restarts on every byte), so a 64 KB MapResponse at 8 kbit/s completes
 *    instead of forcing a reconnect;
 *  - the hard cap scales with frame size (64 KB -> 64 s, <= 10 KB -> 10 s) so a
 *    trickling peer cannot hold the coord task unboundedly.
 *
 * Build/run: make -C host test
 */

#include <stdio.h>

#include "ml_coord_frame_budget.h"

static int fails = 0;
#define CHECK(cond, msg)                                   \
  do {                                                     \
    if (!(cond)) {                                         \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, msg); \
      fails++;                                             \
    }                                                      \
  } while (0)

#define S(x) ((int64_t)(x) * 1000000LL) /* seconds -> us */

int main(void)
{
  ml_coord_frame_budget_t b;

  /* Hard cap sizing. */
  CHECK(ml_coord_frame_budget_total_ms(3) == 10000u, "3-byte header -> 10 s floor");
  CHECK(ml_coord_frame_budget_total_ms(4096) == 10000u, "4 KB control frame -> 10 s floor");
  CHECK(ml_coord_frame_budget_total_ms(10240) == 10000u, "10 KB -> exactly the floor");
  CHECK(ml_coord_frame_budget_total_ms(65535) == 63999u, "64 KB MapResponse frame -> ~64 s");

  /* Idle stream: not armed, never expires, no bounded wait (socket timeout rules). */
  ml_coord_frame_budget_init(&b, 4096);
  CHECK(!ml_coord_frame_budget_armed(&b), "init -> not armed");
  CHECK(!ml_coord_frame_budget_expired(&b, S(1000)), "not armed -> never expired");
  CHECK(ml_coord_frame_budget_wait_us(&b, S(1000)) == 0, "not armed -> no bounded wait");

  /* First byte at t=100 s arms both limits (small frame: both 10 s). */
  ml_coord_frame_budget_on_bytes(&b, S(100));
  CHECK(ml_coord_frame_budget_armed(&b), "first byte arms");
  CHECK(ml_coord_frame_budget_wait_us(&b, S(100)) == S(10), "full 10 s right after arming");
  CHECK(!ml_coord_frame_budget_expired(&b, S(109)), "t+9 s not expired");
  CHECK(ml_coord_frame_budget_wait_us(&b, S(103)) == S(7), "remaining shrinks with time");
  CHECK(ml_coord_frame_budget_expired(&b, S(110)), "no byte for 10 s -> expired (hard cap == window here)");
  CHECK(ml_coord_frame_budget_wait_us(&b, S(200)) == 0, "never negative");

  /* Regression (#127): a STALLED peer. Each wait is the remaining time to the
   * nearer deadline, so the total is 10 s, not 300 x SO_RCVTIMEO. */
  ml_coord_frame_budget_init(&b, 4096);
  int64_t now = S(500);
  ml_coord_frame_budget_on_bytes(&b, now);
  int64_t total_wait = 0;
  int reads = 0;
  for (int i = 0; i < 300 && !ml_coord_frame_budget_expired(&b, now); i++) {
    int64_t w = ml_coord_frame_budget_wait_us(&b, now);
    const int64_t so_rcvtimeo = S(2); /* what the old count-based loop blocked for */
    if (w > so_rcvtimeo) w = so_rcvtimeo; /* select() returns when data arrives or w elapses */
    total_wait += w;
    now += w;
    reads++;
  }
  CHECK(ml_coord_frame_budget_expired(&b, now), "stalled frame is abandoned");
  CHECK(total_wait == S(10), "total stall cost = 10 s");
  CHECK(reads == 5, "5 bounded waits of <= 2 s, not 300");

  /* Slow but LIVE: 64 KB MapResponse frame at ~8 kbit/s (1 KiB every second).
   * The old fixed 10 s budget would have reconnected at byte ~10 K; now every
   * byte restarts the no-progress window and the hard cap (64 s) is not hit. */
  ml_coord_frame_budget_init(&b, 65535);
  now = S(1000);
  size_t got = 0;
  bool aborted = false;
  while (got < 65535) {
    ml_coord_frame_budget_on_bytes(&b, now); /* a chunk just landed */
    got += 1024;
    if (got >= 65535) break; /* frame complete — coord_recv_ex returns before any further wait */
    now += S(1); /* next chunk arrives one second later */
    if (ml_coord_frame_budget_expired(&b, now)) {
      aborted = true;
      break;
    }
  }
  CHECK(!aborted, "64 KB at 1 KiB/s completes (progress restarts the window)");
  CHECK(now - S(1000) == S(63), "last chunk landed at t+63 s, inside the ~64 s cap");

  /* Same frame, 2x slower (512 B/s): the hard cap ends it at 64 s with half the frame. */
  ml_coord_frame_budget_init(&b, 65535);
  now = S(2000);
  got = 0;
  aborted = false;
  while (got < 65535) {
    ml_coord_frame_budget_on_bytes(&b, now);
    got += 512;
    if (got >= 65535) break;
    now += S(1);
    if (ml_coord_frame_budget_expired(&b, now)) {
      aborted = true;
      break;
    }
  }
  CHECK(aborted, "512 B/s trickle is cut by the hard cap");
  CHECK(now - S(2000) == S(64), "cut at the 64 s cap, not earlier");
  CHECK(got <= 33280, "roughly half the frame had arrived");

  /* Stall in the middle of a big frame: 10 s after the last byte, not at the cap. */
  ml_coord_frame_budget_init(&b, 65535);
  ml_coord_frame_budget_on_bytes(&b, S(3000));
  ml_coord_frame_budget_on_bytes(&b, S(3005)); /* last byte at t+5 */
  CHECK(!ml_coord_frame_budget_expired(&b, S(3014)), "9 s after the last byte: still waiting");
  CHECK(ml_coord_frame_budget_wait_us(&b, S(3014)) == S(1), "wait bounded by the no-progress window");
  CHECK(ml_coord_frame_budget_expired(&b, S(3015)), "10 s after the last byte: abandoned");

  /* Mid-frame entry (Noise payload after its consumed header): armed at once. */
  ml_coord_frame_budget_init(&b, 40000);
  ml_coord_frame_budget_on_bytes(&b, S(700));
  CHECK(ml_coord_frame_budget_wait_us(&b, S(700)) == S(10), "armed at entry; first wait is the no-progress window");

  /* init forgets everything. */
  ml_coord_frame_budget_init(&b, 1);
  CHECK(!ml_coord_frame_budget_armed(&b) && ml_coord_frame_budget_wait_us(&b, S(999)) == 0, "init clears");

  if (fails == 0) printf("test_coord_frame_budget: OK (27 checks)\n");
  return fails ? 1 : 0;
}
