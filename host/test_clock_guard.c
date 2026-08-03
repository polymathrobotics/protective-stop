// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Host unit tests for the machine-side clock-freeze guard (clock_guard.c),
 * the SR-H-04b / FMEA DU-2 mitigation. Fully host-runnable (no hardware): a
 * mock/stuck/backward clock is injected as reading triples and the detector
 * must trip -> STOP (the runner forces machine_stop_robot on any latched
 * fault). Style follows firmware/test/test_estop_verdict.c.
 *
 * Requirement trace (docs/safety/SAFETY_REQUIREMENTS.md SR-H-04):
 *   - a frozen CLOCK_MONOTONIC while independent references advance -> FROZEN
 *   - a frozen CLOCK_MONOTONIC while ALL clocks are wedged (call-count proxy) -> FROZEN
 *   - a backward CLOCK_MONOTONIC jump -> BACKWARD
 *   - a healthy advancing clock never false-trips (including a REALTIME step-back)
 *   - the fault verdict is sticky once latched
 */

#include <stdio.h>

#include "clock_guard.h"

static int g_checks = 0;
static int g_fails = 0;
#define CHECK(cond, what)             \
  do {                                \
    g_checks++;                       \
    if (!(cond)) {                    \
      g_fails++;                      \
      printf("  FAIL: %s\n", (what)); \
    }                                 \
  } while (0)

int main(void)
{
  /* 1. Healthy operation: every clock advances ~10 ms/poll for a long run.
   *    The guard must never trip. Also exercises a REALTIME step-BACKWARD
   *    (NTP correction) mid-run, which must not be read as time passing. */
  {
    clock_guard_t g;
    clock_guard_init(&g, 500u, 2000u);
    uint64_t mono = 1000u, boot = 5000u, real = 1700000000000ULL;
    for (int i = 0; i < 5000; i++) {
      mono += 10u;
      boot += 10u;
      real += 10u;
      if (i == 2500) real -= 250u; /* NTP steps wall-clock back 250 ms */
      CHECK(clock_guard_update(&g, mono, boot, real) == CLOCK_GUARD_OK, "healthy run never trips");
    }
    CHECK(clock_guard_fault(&g) == CLOCK_GUARD_OK, "healthy end state OK");
  }

  /* 2. Frozen monotonic clock while an INDEPENDENT reference advances.
   *    This is the classic DU-2: get_time_cb stuck at a constant. BOOTTIME
   *    keeps moving, so the window (500 ms) trips FROZEN — well before the
   *    call-count backstop would. */
  {
    clock_guard_t g;
    clock_guard_init(&g, 500u, 100000u); /* huge call cap -> window must be the trigger */
    uint64_t mono = 42000u, boot = 9000u, real = 1700000000000ULL;
    /* prime + a few healthy ticks */
    for (int i = 0; i < 5; i++) {
      mono += 10u;
      boot += 10u;
      real += 10u;
      CHECK(clock_guard_update(&g, mono, boot, real) == CLOCK_GUARD_OK, "pre-freeze healthy");
    }
    /* mono now frozen; references keep advancing 10 ms per poll */
    clock_guard_fault_t f = CLOCK_GUARD_OK;
    int tripped_at = -1;
    for (int i = 0; i < 200; i++) {
      boot += 10u;
      real += 10u;
      f = clock_guard_update(&g, mono, boot, real); /* mono NOT advanced */
      if (f != CLOCK_GUARD_OK && tripped_at < 0) tripped_at = i;
    }
    CHECK(f == CLOCK_GUARD_FAULT_FROZEN, "frozen mono + advancing ref -> FROZEN");
    /* 500 ms window / 10 ms per poll ~= 50 polls; allow slack. */
    CHECK(tripped_at >= 0 && tripped_at <= 60, "freeze detected within ~window");
  }

  /* 3. Frozen monotonic clock AND both references wedged too (silicon
   *    clocksource dead) — only the spinning loop makes progress. The
   *    call-count proxy is the sole remaining evidence and must trip FROZEN. */
  {
    clock_guard_t g;
    clock_guard_init(&g, 500u, 300u); /* window can't fire (refs frozen); count backstop must */
    uint64_t mono = 7000u, boot = 7000u, real = 1700000000000ULL;
    CHECK(clock_guard_update(&g, mono, boot, real) == CLOCK_GUARD_OK, "prime all-wedged");
    clock_guard_fault_t f = CLOCK_GUARD_OK;
    for (int i = 0; i < 400; i++) {
      /* nothing advances: mono, boot, real all constant */
      f = clock_guard_update(&g, mono, boot, real);
    }
    CHECK(f == CLOCK_GUARD_FAULT_FROZEN, "all clocks wedged -> call-count proxy trips FROZEN");
  }

  /* 4. Backward monotonic jump -> immediate BACKWARD fault. */
  {
    clock_guard_t g;
    clock_guard_init(&g, 500u, 2000u);
    uint64_t mono = 100000u, boot = 100000u, real = 1700000000000ULL;
    for (int i = 0; i < 5; i++) {
      mono += 10u;
      boot += 10u;
      real += 10u;
      CHECK(clock_guard_update(&g, mono, boot, real) == CLOCK_GUARD_OK, "pre-backward healthy");
    }
    boot += 10u;
    real += 10u;
    clock_guard_fault_t f = clock_guard_update(&g, mono - 5000u, boot, real); /* jump back 5 s */
    CHECK(f == CLOCK_GUARD_FAULT_BACKWARD, "backward mono -> BACKWARD");
  }

  /* 5. Sticky latch: once frozen, a subsequently "recovered" clock still
   *    reports the fault. A clock fault must never be un-seen. */
  {
    clock_guard_t g;
    clock_guard_init(&g, 500u, 2000u);
    uint64_t mono = 500u, boot = 500u, real = 1700000000000ULL;
    clock_guard_update(&g, mono, boot, real);
    for (int i = 0; i < 100; i++) { /* freeze mono, advance boot -> trips */
      boot += 20u;
      real += 20u;
      clock_guard_update(&g, mono, boot, real);
    }
    CHECK(clock_guard_fault(&g) == CLOCK_GUARD_FAULT_FROZEN, "latched FROZEN");
    /* clock "comes back" perfectly healthy — verdict must stay FROZEN */
    for (int i = 0; i < 100; i++) {
      mono += 10u;
      boot += 10u;
      real += 10u;
      CHECK(clock_guard_update(&g, mono, boot, real) == CLOCK_GUARD_FAULT_FROZEN, "fault stays latched");
    }
  }

  printf("clock_guard: %d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
