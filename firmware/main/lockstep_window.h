// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Lockstep publish window — pure, host-testable (firmware/test/test_lockstep_window.c).
 *
 * Each 100 ms tick the comparator notifies both encode cores at the same
 * instant and then waits for their completion semaphores, core 0 first, then
 * core 1. A tick transmits only if BOTH cores published within
 * CORE_PUBLISH_TIMEOUT (80 ms) of the notify (SR-R-07: transmit on exact
 * agreement of two fresh encodings; a late encoding is not fresh).
 *
 * Two sequential takes each armed with the full timeout do not implement that:
 * when core 0 consumed the whole budget, core 1 got a fresh 80 ms on top —
 * a tick could transmit with core 1 landing at 160 ms, and a both-late event
 * was reported as "core 0 late". The window is therefore ONE absolute deadline
 * shared by both takes: the second take gets whatever the first left.
 */

#ifndef LOCKSTEP_WINDOW_H
#define LOCKSTEP_WINDOW_H

#include <stdbool.h>
#include <stdint.h>

/* Wait for the second take given the budget and what the first take consumed
 * (both in ticks). 0 = non-blocking take (still evaluated: a give already
 * pending is consumed; one landing later is drained by the next tick). */
static inline uint32_t lockstep_second_wait_ticks(uint32_t budget_ticks, uint32_t elapsed_ticks)
{
  return (elapsed_ticks < budget_ticks) ? (budget_ticks - elapsed_ticks) : 0u;
}

/* Model of one tick's decision, for the host test and for reasoning about the
 * window: each core's publish latency in ticks since the notify
 * (LOCKSTEP_NEVER = did not publish). Mirrors the comparator: take 0 returns
 * when core 0 publishes or at the deadline; take 1 waits the remainder. */
#define LOCKSTEP_NEVER 0xFFFFFFFFu

typedef struct
{
  bool in0; /* core 0 published within the window */
  bool in1; /* core 1 published within the window */
  bool transmit; /* both in -> the comparator may transmit (given agreement) */
  uint32_t late_mask; /* bit 0 = core 0 late, bit 1 = core 1 late */
  uint32_t tick_hold; /* how long the comparator was held by the two takes */
} lockstep_tick_t;

static inline lockstep_tick_t lockstep_tick_model(uint32_t budget_ticks, uint32_t done0, uint32_t done1)
{
  lockstep_tick_t r;
  r.in0 = (done0 <= budget_ticks);
  uint32_t elapsed = r.in0 ? done0 : budget_ticks; /* take 0 returns at publish or deadline */
  uint32_t wait1 = lockstep_second_wait_ticks(budget_ticks, elapsed);
  r.in1 = (done1 <= elapsed + wait1); /* core 1 may already be done, or land inside the remainder */
  uint32_t end1 = r.in1 ? ((done1 > elapsed) ? done1 : elapsed) : (elapsed + wait1);
  r.tick_hold = end1;
  r.transmit = r.in0 && r.in1;
  r.late_mask = (r.in0 ? 0u : 1u) | (r.in1 ? 0u : 2u);
  return r;
}

#endif /* LOCKSTEP_WINDOW_H */
