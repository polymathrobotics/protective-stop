// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Host test for ml_coord_frame_budget.h — the wall-clock budget that bounds a
 * partial coordination frame (ml_coord.c coord_recv, issue #127).
 *
 * Pins the regression: the old retry COUNT let each retry block for the
 * socket's SO_RCVTIMEO, so 300 retries x 2 s held the coord task ~10 min.
 * Now a stall costs at most the 10 s budget, and progress never extends it.
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
  ml_coord_frame_budget_reset(&b);

  /* Idle stream: not armed, never expires, no bounded wait (socket timeout rules). */
  CHECK(!ml_coord_frame_budget_armed(&b), "reset -> not armed");
  CHECK(!ml_coord_frame_budget_expired(&b, S(1000)), "not armed -> never expired");
  CHECK(ml_coord_frame_budget_wait_us(&b, S(1000)) == 0, "not armed -> no bounded wait");

  /* First byte at t=100 s arms a 10 s budget. */
  ml_coord_frame_budget_on_bytes(&b, S(100));
  CHECK(ml_coord_frame_budget_armed(&b), "first byte arms");
  CHECK(ml_coord_frame_budget_wait_us(&b, S(100)) == S(10), "full budget right after arming");
  CHECK(!ml_coord_frame_budget_expired(&b, S(109)), "t+9 s not expired");
  CHECK(ml_coord_frame_budget_wait_us(&b, S(103)) == S(7), "remaining shrinks with time");

  /* Progress does NOT extend the budget (a trickling peer is a stall too). */
  ml_coord_frame_budget_on_bytes(&b, S(105));
  ml_coord_frame_budget_on_bytes(&b, S(109));
  CHECK(ml_coord_frame_budget_wait_us(&b, S(109)) == S(1), "progress at t+5/t+9 did not extend");
  CHECK(ml_coord_frame_budget_expired(&b, S(110)), "expires exactly at t+10 s");
  CHECK(ml_coord_frame_budget_wait_us(&b, S(110)) == 0, "no wait once expired");
  CHECK(ml_coord_frame_budget_wait_us(&b, S(200)) == 0, "never negative");

  /* Regression (#127): 300 stalled reads. Each wait is the REMAINING budget, so
   * the sum of all waits is the budget, not 300 x SO_RCVTIMEO. */
  ml_coord_frame_budget_reset(&b);
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
  CHECK(total_wait == S(10), "total stall cost = the 10 s budget");
  CHECK(reads == 5, "5 bounded waits of <= 2 s, not 300");

  /* Mid-frame entry (Noise payload after its consumed header): armed at once. */
  ml_coord_frame_budget_reset(&b);
  ml_coord_frame_budget_on_bytes(&b, S(700));
  CHECK(ml_coord_frame_budget_wait_us(&b, S(700)) == S(10), "armed at entry when the header is already consumed");

  /* Reset forgets everything. */
  ml_coord_frame_budget_reset(&b);
  CHECK(!ml_coord_frame_budget_armed(&b) && ml_coord_frame_budget_wait_us(&b, S(999)) == 0, "reset clears");

  if (fails == 0) printf("test_coord_frame_budget: OK (18 checks)\n");
  return fails ? 1 : 0;
}
