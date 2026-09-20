// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Host test for lockstep_window.h — the comparator's shared publish window
 * (SR-R-07: transmit only on exact agreement of two FRESH encodings).
 *
 * Pins the behavioural change made in the lockstep-attribution work: both
 * cores must publish within ONE 80 ms window measured from the notify. Under
 * the previous two sequential takes (each with its own 80 ms), a tick could
 * transmit with core 1 landing at up to 160 ms, and a both-late tick was
 * attributed to core 0 alone.
 *
 * Build/run: make -C firmware/test run
 */

#include <stdio.h>

#include "lockstep_window.h"

static int fails = 0;
#define CHECK(cond, msg)                                   \
  do {                                                     \
    if (!(cond)) {                                         \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, msg); \
      fails++;                                             \
    }                                                      \
  } while (0)

#define BUDGET 80u /* CORE_PUBLISH_TIMEOUT in ms (1 ms ticks) */

int main(void)
{
  /* Remainder arithmetic. */
  CHECK(lockstep_second_wait_ticks(BUDGET, 0) == 80u, "core 0 instant -> core 1 gets the full window");
  CHECK(lockstep_second_wait_ticks(BUDGET, 30) == 50u, "core 0 at 30 ms -> core 1 gets 50 ms");
  CHECK(lockstep_second_wait_ticks(BUDGET, 79) == 1u, "core 0 at 79 ms -> core 1 gets 1 ms");
  CHECK(lockstep_second_wait_ticks(BUDGET, 80) == 0u, "core 0 timed out -> core 1 take is non-blocking");
  CHECK(lockstep_second_wait_ticks(BUDGET, 200) == 0u, "never negative / never a fresh budget");

  /* Normal tick: both well inside. */
  lockstep_tick_t t = lockstep_tick_model(BUDGET, 5, 7);
  CHECK(t.transmit && t.late_mask == 0u, "5/7 ms -> transmit");
  CHECK(t.tick_hold == 7u, "comparator held only until the slower core (7 ms)");

  /* Core 1 first, core 0 later but inside: still one window. */
  t = lockstep_tick_model(BUDGET, 60, 10);
  CHECK(t.transmit && t.tick_hold == 60u, "core 1 already done when take 1 runs -> transmit at 60 ms");

  /* Boundary: exactly at the window edge is in; one tick past is out. */
  CHECK(lockstep_tick_model(BUDGET, 80, 80).transmit, "80/80 ms -> in (edge inclusive)");
  CHECK(!lockstep_tick_model(BUDGET, 81, 10).transmit, "core 0 at 81 ms -> tainted");
  CHECK(!lockstep_tick_model(BUDGET, 10, 81).transmit, "core 1 at 81 ms -> tainted");

  /* THE CHANGE: core 0 on time at 79 ms, core 1 at 150 ms. The old sequential
   * takes gave core 1 a fresh 80 ms after the 79 -> in until 159 ms -> the tick
   * transmitted with a 150 ms-old encoding. Now the window is shared. */
  t = lockstep_tick_model(BUDGET, 79, 150);
  CHECK(!t.transmit, "79/150 ms -> TAINTED (previously transmitted)");
  CHECK(t.in0 && !t.in1 && t.late_mask == 2u, "attributed to core 1 only");
  CHECK(t.tick_hold == 80u, "held exactly the window, not 159 ms");

  /* Both late: both bits set, hold is exactly the window (was 160 ms). */
  t = lockstep_tick_model(BUDGET, 120, 130);
  CHECK(!t.transmit && t.late_mask == 3u, "both late -> mask 0b11 (previously 'core 0 late' only)");
  CHECK(t.tick_hold == 80u, "worst-case tick stall is the window, not 2x");

  /* A core that never publishes: tainted, hold bounded. */
  t = lockstep_tick_model(BUDGET, LOCKSTEP_NEVER, 3);
  CHECK(!t.transmit && t.late_mask == 1u && t.tick_hold == 80u, "core 0 dead -> tainted, core 0 late, 80 ms hold");
  t = lockstep_tick_model(BUDGET, 3, LOCKSTEP_NEVER);
  CHECK(!t.transmit && t.late_mask == 2u && t.tick_hold == 80u, "core 1 dead -> tainted, core 1 late, 80 ms hold");

  /* Fail-safe direction only: nothing that was tainted before can transmit now. */
  for (uint32_t d0 = 0; d0 <= 200; d0 += 5) {
    for (uint32_t d1 = 0; d1 <= 200; d1 += 5) {
      bool old_in0 = d0 <= BUDGET;
      uint32_t old_elapsed = old_in0 ? d0 : BUDGET;
      bool old_in1 = d1 <= old_elapsed + BUDGET; /* the previous fresh-budget second take */
      bool old_tx = old_in0 && old_in1;
      lockstep_tick_t n = lockstep_tick_model(BUDGET, d0, d1);
      if (n.transmit && !old_tx) {
        CHECK(0, "new window transmits where the old one did not (would be a relaxation)");
      }
      if (n.transmit != (d0 <= BUDGET && d1 <= BUDGET)) {
        CHECK(0, "transmit iff both cores published within the shared window");
      }
    }
  }

  if (fails == 0) printf("test_lockstep_window: OK (17 checks + 1681-point sweep)\n");
  return fails ? 1 : 0;
}
