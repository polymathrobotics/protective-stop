// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the pure lifetime-health logic
// (firmware/components/dcs_support/src/dcs_health_logic.c): blob codec,
// press / mismatch edge detectors, wear thresholds, flush policy.

#include <stdio.h>
#include <string.h>

#include "dcs_health_logic.h"

#define OK 0x55u
#define STOP 0x92u

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
  // 1. Codec round-trip, version and length guards.
  {
    dcs_health_counters_t c = {
      .presses = 0x01020304u,
      .mismatch_events = 7u,
      .uptime_s = 0xFFFFFFFFu,
      .boots = 41u,
      .flashes = 6u,
      .otas = 5u,
      .fw_sha = {0xde, 0xad, 0xbe, 0xef, 0x00, 0x11, 0x22, 0x33},
      .button_swaps = 2u,
    };
    uint8_t blob[DCS_HEALTH_BLOB_LEN];
    dcs_health_encode(&c, blob);
    CHECK(blob[0] == DCS_HEALTH_BLOB_VER, "blob version byte");
    CHECK(blob[1] == 0x01 && blob[4] == 0x04, "u32 big-endian");
    dcs_health_counters_t d;
    memset(&d, 0xAA, sizeof(d));
    CHECK(dcs_health_decode(blob, sizeof(blob), &d), "decode ok");
    CHECK(
      d.presses == c.presses && d.mismatch_events == c.mismatch_events && d.uptime_s == c.uptime_s &&
        d.boots == c.boots && d.flashes == c.flashes && d.otas == c.otas &&
        memcmp(d.fw_sha, c.fw_sha, sizeof(c.fw_sha)) == 0 && d.button_swaps == c.button_swaps,
      "round-trip equal");
    CHECK(!dcs_health_decode(blob, sizeof(blob) - 1u, &d), "short blob rejected");
    blob[0] = 99;
    CHECK(!dcs_health_decode(blob, sizeof(blob), &d), "wrong version rejected");
    CHECK(!dcs_health_decode(NULL, sizeof(blob), &d), "NULL rejected");
  }

  // 2. Boot priming: both cores emit STOP for their first ticks regardless of
  //    the button (estop_verdict boot warm-up), publishing one after the other
  //    per tick. None of that may count. Then a real press does.
  {
    dcs_health_edge_t st = {0};
    int presses = 0, mism = 0;
    for (uint32_t t = 1; t <= 5; t++) {
      dcs_health_edge_out_t o0 = dcs_health_edge_step(&st, 0, t, STOP, STOP);
      dcs_health_edge_out_t o1 = dcs_health_edge_step(&st, 1, t, STOP, STOP);
      presses += o0.press + o1.press;
      mism += o0.mismatch + o1.mismatch;
    }
    CHECK(presses == 0 && mism == 0, "boot-priming STOP ticks: nothing counted");
    // Loops prove healthy -> OK on both.
    (void)dcs_health_edge_step(&st, 0, 6, OK, STOP);
    (void)dcs_health_edge_step(&st, 1, 6, OK, STOP);
    // Operator presses.
    dcs_health_edge_out_t a = dcs_health_edge_step(&st, 0, 7, STOP, STOP);
    dcs_health_edge_out_t b = dcs_health_edge_step(&st, 1, 7, STOP, STOP);
    CHECK(!a.press && !a.mismatch, "core0 alone (skewed tick): no verdict yet");
    CHECK(b.press && !b.mismatch, "core1 aligns: press counted");
    CHECK(!dcs_health_edge_step(&st, 0, 8, STOP, STOP).press, "held: no press");
    CHECK(!dcs_health_edge_step(&st, 1, 8, STOP, STOP).press, "held (aligned): no press");
    CHECK(!dcs_health_edge_step(&st, 0, 9, OK, STOP).press, "release core0");
    CHECK(!dcs_health_edge_step(&st, 1, 9, OK, STOP).press, "release core1: not a press");
  }

  // 3. Button latched at power-on (no priming STOP artifact modelled): the
  //    first aligned observation is the baseline; the release and a new press
  //    give exactly one count.
  {
    dcs_health_edge_t st = {0};
    int presses = 0;
    presses += dcs_health_edge_step(&st, 1, 1, STOP, STOP).press; /* core1 publishes first this time */
    presses += dcs_health_edge_step(&st, 0, 1, STOP, STOP).press;
    CHECK(presses == 0, "latched at boot: baseline only");
    for (int i = 0; i < 3; i++) {
      uint32_t t = 10u + (uint32_t)(10 * i);
      presses += dcs_health_edge_step(&st, 0, t, OK, STOP).press;
      presses += dcs_health_edge_step(&st, 1, t, OK, STOP).press;
      presses += dcs_health_edge_step(&st, 0, t + 1, STOP, STOP).press;
      presses += dcs_health_edge_step(&st, 1, t + 1, STOP, STOP).press;
      presses += dcs_health_edge_step(&st, 0, t + 2, STOP, STOP).press;
      presses += dcs_health_edge_step(&st, 1, t + 2, STOP, STOP).press;
    }
    CHECK(presses == 3, "three release+press cycles = 3");
  }

  // 4. Mismatch: one loop open on an aligned tick is a mismatch event, not a
  //    press; sustained disagreement is one event; a mismatch that resolves
  //    into both-STOP is one press.
  {
    dcs_health_edge_t st = {0};
    (void)dcs_health_edge_step(&st, 0, 0, OK, STOP);
    (void)dcs_health_edge_step(&st, 1, 0, OK, STOP);
    dcs_health_edge_out_t o;
    (void)dcs_health_edge_step(&st, 0, 1, STOP, STOP);
    o = dcs_health_edge_step(&st, 1, 1, OK, STOP);
    CHECK(o.mismatch && !o.press, "one loop open = mismatch, not press");
    (void)dcs_health_edge_step(&st, 0, 2, STOP, STOP);
    o = dcs_health_edge_step(&st, 1, 2, OK, STOP);
    CHECK(!o.mismatch, "sustained disagreement = one event");
    (void)dcs_health_edge_step(&st, 0, 3, OK, STOP);
    o = dcs_health_edge_step(&st, 1, 3, OK, STOP);
    CHECK(!o.mismatch && !o.press, "recovered");
    (void)dcs_health_edge_step(&st, 0, 4, OK, STOP);
    o = dcs_health_edge_step(&st, 1, 4, STOP, STOP);
    CHECK(o.mismatch, "other loop open: new event");
    (void)dcs_health_edge_step(&st, 0, 5, STOP, STOP);
    o = dcs_health_edge_step(&st, 1, 5, STOP, STOP);
    CHECK(o.press && !o.mismatch, "second loop follows: one press, mismatch cleared");
    o = dcs_health_edge_step(&st, 2, 6, STOP, STOP);
    CHECK(!o.press && !o.mismatch, "bad core id ignored");
  }

  // 5. Button wear thresholds (NKK FF01: 100,000 ops rated).
  {
    uint32_t pct = 123;
    CHECK(dcs_health_button_level(0, 100000, 80, &pct) == 0 && pct == 0, "new unit ok");
    CHECK(dcs_health_button_level(79999, 100000, 80, &pct) == 0 && pct == 79, "79% ok");
    CHECK(dcs_health_button_level(80000, 100000, 80, &pct) == 1 && pct == 80, "80% warn");
    CHECK(dcs_health_button_level(100000, 100000, 80, &pct) == 2 && pct == 100, "100% critical");
    CHECK(dcs_health_button_level(5000000, 100000, 80, &pct) == 2 && pct == 999, "pct clamps at 999");
    CHECK(dcs_health_button_level(1000, 0, 80, &pct) == 0 && pct == 0, "rating 0 = disabled");
    CHECK(dcs_health_button_level(90000, 100000, 0, &pct) == 0, "warn_pct 0 = no warn below 100");
    CHECK(dcs_health_button_level(0xFFFFFFFFu, 100000, 80, &pct) == 2, "u32 max no overflow");
  }

  // 6. Mismatch rate threshold: min_events floor, 1000-press basis floor.
  {
    CHECK(dcs_health_mismatch_level(49, 10, 50, 20) == 0, "below min_events");
    CHECK(dcs_health_mismatch_level(50, 10, 50, 20) == 1, "50 events on 10 presses (basis 1000): 50 > 20");
    CHECK(dcs_health_mismatch_level(50, 5000, 50, 20) == 0, "50 on 5000 presses: allowed 100");
    CHECK(dcs_health_mismatch_level(101, 5000, 50, 20) == 1, "101 on 5000: > 100");
    CHECK(dcs_health_mismatch_level(100, 5000, 50, 20) == 0, "100 on 5000: == allowed, not over");
    CHECK(dcs_health_mismatch_level(1000000, 0, 50, 0) == 0, "per_1000 0 = disabled");
  }

  // 7. Flush policy.
  {
    CHECK(!dcs_health_flush_due(false, 0, 60, 600), "clean, fresh: no");
    CHECK(!dcs_health_flush_due(true, 59, 60, 600), "dirty, 59 s: no");
    CHECK(dcs_health_flush_due(true, 60, 60, 600), "dirty, 60 s: yes");
    CHECK(!dcs_health_flush_due(false, 599, 60, 600), "clean, 599 s: no");
    CHECK(dcs_health_flush_due(false, 600, 60, 600), "clean, 600 s: yes (uptime)");
    // MIN > MAX misconfiguration: the throttle wins, MAX stretches to MIN.
    CHECK(!dcs_health_flush_due(true, 600, 3600, 600), "min 3600 > max 600: dirty at 600 s still throttled");
    CHECK(!dcs_health_flush_due(false, 3599, 3600, 600), "min > max: clean at 3599 s no");
    CHECK(dcs_health_flush_due(false, 3600, 3600, 600), "min > max: clean at 3600 s yes");
  }

  printf("dcs_health_logic: %d checks, %d failures\n", g_checks, g_fails);
  return (g_fails == 0) ? 0 : 1;
}
