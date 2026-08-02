// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the pure E-stop decision core (firmware/main/estop_verdict.c).
// Compiled native with gcc-14 --coverage -fcondition-coverage to measure
// statement + branch + MC/DC on the SIL-critical verdict logic that cannot be
// gcov-instrumented on-target (its JTAG pins are the E-stop loop pins).
//
// Requirement trace (docs/safety/SAFETY_REQUIREMENTS.md): exercises the
// fresh-both-phase verdict (SR-R-01/02), Option-B diversity equivalence
// (SR-R-03), asymmetric release debounce (SR-R-04-adjacent), and boot priming.

#include <stdio.h>
#include <string.h>

#include "estop_verdict.h"
#include "pstop/pstop_msg.h"

static int g_checks = 0;
static int g_fails = 0;
#define CHECK(cond, what)                             \
  do {                                                \
    g_checks++;                                       \
    if (!(cond)) {                                    \
      g_fails++;                                       \
      printf("  FAIL: %s\n", (what));                 \
    }                                                 \
  } while (0)

static estop_state_t fresh(void)
{
  estop_state_t s;
  memset(&s, 0, sizeof(s));
  return s;
}

int main(void)
{
  // 1. A healthy closed read (rb_hi=1, rb_lo=0) is STOP until the release
  //    debounce is satisfied (3 consecutive), then OK; any open reports STOP
  //    on the SAME tick; reclose re-runs the full debounce. Both cores.
  for (int core = 0; core < 2; core++) {
    estop_state_t s = fresh();
    CHECK(estop_decide(&s, core, 1, 0) == PSTOP_MESSAGE_STOP, "healthy tick1 STOP (debounce)");
    CHECK(estop_decide(&s, core, 1, 0) == PSTOP_MESSAGE_STOP, "healthy tick2 STOP");
    CHECK(estop_decide(&s, core, 1, 0) == PSTOP_MESSAGE_OK, "healthy tick3 -> OK");
    CHECK(estop_decide(&s, core, 1, 0) == PSTOP_MESSAGE_OK, "healthy tick4 stays OK");
    CHECK(estop_decide(&s, core, 0, 0) == PSTOP_MESSAGE_STOP, "open -> immediate STOP");
    CHECK(estop_decide(&s, core, 1, 0) == PSTOP_MESSAGE_STOP, "reclose tick1 STOP");
    CHECK(estop_decide(&s, core, 1, 0) == PSTOP_MESSAGE_STOP, "reclose tick2 STOP");
    CHECK(estop_decide(&s, core, 1, 0) == PSTOP_MESSAGE_OK, "reclose tick3 -> OK");
  }

  // 2. Every fault read is STOP forever (never manufactures OK), both cores:
  //    (1,1) stuck-high short (low phase fails), (0,0) open, (0,1) drive-high fail.
  const int fault[3][2] = {{1, 1}, {0, 0}, {0, 1}};
  for (int core = 0; core < 2; core++) {
    for (int i = 0; i < 3; i++) {
      estop_state_t s = fresh();
      uint8_t m = PSTOP_MESSAGE_OK;
      for (int t = 0; t < 6; t++) {
        m = estop_decide(&s, core, fault[i][0], fault[i][1]);
      }
      CHECK(m == PSTOP_MESSAGE_STOP, "fault combo never OK");
    }
  }

  // 3. Option-B diversity equivalence: core 0 (arithmetic image) and core 1
  //    (boolean) must return the SAME message for EVERY (rb_hi, rb_lo) at the
  //    same debounce point. This is the "no nuisance divergence" invariant.
  for (int hi = 0; hi < 2; hi++) {
    for (int lo = 0; lo < 2; lo++) {
      estop_state_t s0 = fresh(), s1 = fresh();
      uint8_t m0 = 0, m1 = 0;
      for (int t = 0; t < 4; t++) {
        m0 = estop_decide(&s0, 0, hi, lo);
        m1 = estop_decide(&s1, 1, hi, lo);
      }
      CHECK(m0 == m1, "core0 and core1 agree for every (hi,lo)");
    }
  }

  // 4. Boot priming — not primed until BOTH cores settle. Healthy path.
  {
    estop_state_t st[2] = {fresh(), fresh()};
    CHECK(!estop_channels_primed(st), "not primed at boot");
    for (int t = 0; t < LOOP_RECLOSE_DEBOUNCE_TICKS; t++) {
      estop_decide(&st[0], 0, 1, 0);
    }
    CHECK(!estop_channels_primed(st), "not primed with only core0 settled");
    for (int t = 0; t < LOOP_RECLOSE_DEBOUNCE_TICKS; t++) {
      estop_decide(&st[1], 1, 1, 0);
    }
    CHECK(estop_channels_primed(st), "primed once both cores settle (healthy)");
  }

  // 4b. Boot priming via the held-open path (button held at power-on): settles
  //     after LOOP_BOOT_OPEN_CONFIRM_TICKS consecutive open reads.
  {
    estop_state_t st[2] = {fresh(), fresh()};
    for (int t = 0; t < LOOP_BOOT_OPEN_CONFIRM_TICKS; t++) {
      estop_decide(&st[0], 0, 0, 0);
      estop_decide(&st[1], 1, 0, 0);
    }
    CHECK(estop_channels_primed(st), "primed via held-open boot path");
  }

  // 5. Streak-saturation guards (the `< 255` false branches): a long healthy
  //    run stays OK; a long open run stays STOP. No wrap flips the verdict.
  {
    estop_state_t s = fresh();
    uint8_t m = 0;
    for (int t = 0; t < 300; t++) m = estop_decide(&s, 0, 1, 0);
    CHECK(m == PSTOP_MESSAGE_OK, "300 healthy ticks stay OK (closed_streak saturates)");
    s = fresh();
    for (int t = 0; t < 300; t++) m = estop_decide(&s, 1, 0, 0);
    CHECK(m == PSTOP_MESSAGE_STOP, "300 open ticks stay STOP (open_streak saturates)");
  }

  // 6. MC/DC on estop_channels_primed's 6-term AND: from all-true, flipping ANY
  //    single term must make it false (each condition independently decides).
  {
    estop_state_t b[2];
#define ALLTRUE()                                                       \
    do {                                                                \
      for (int c = 0; c < 2; c++) {                                     \
        b[c] = fresh();                                                 \
        b[c].primed_high = b[c].primed_low = b[c].settled = true;       \
      }                                                                 \
    } while (0)
    ALLTRUE();
    CHECK(estop_channels_primed(b), "all-true -> primed");
    ALLTRUE(); b[0].primed_high = false; CHECK(!estop_channels_primed(b), "flip c0.primed_high");
    ALLTRUE(); b[0].primed_low = false;  CHECK(!estop_channels_primed(b), "flip c0.primed_low");
    ALLTRUE(); b[1].primed_high = false; CHECK(!estop_channels_primed(b), "flip c1.primed_high");
    ALLTRUE(); b[1].primed_low = false;  CHECK(!estop_channels_primed(b), "flip c1.primed_low");
    ALLTRUE(); b[0].settled = false;     CHECK(!estop_channels_primed(b), "flip c0.settled");
    ALLTRUE(); b[1].settled = false;     CHECK(!estop_channels_primed(b), "flip c1.settled");
#undef ALLTRUE
  }

  printf("estop_verdict host tests: %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
