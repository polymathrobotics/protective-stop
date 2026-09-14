// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Host tests for the per-peer relay-stuck coord re-fetch schedule
 * (ml_relay_refetch.h). Contract: first fetch due immediately; each issued
 * fetch escalates and arms next_ms with the escalated interval (gaps 180, 360,
 * 720, 1440, 1800, 1800 s for min 90 s / cap 1800 s); the returned value equals
 * next_ms - now; a regain resets only that peer; the fleet floor spaces two
 * peers; a refused enqueue (owner skips _sent) leaves the schedule unchanged. */

#include <stdio.h>

#include "ml_relay_refetch.h"

#define MIN_MS 90000ull
#define MAX_MS 1800000ull

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
  ml_relay_refetch_t a, b;
  ml_relay_refetch_reset(&a, MIN_MS);
  CHECK(a.next_ms == 0 && a.interval_ms == MIN_MS, "reset arms immediately at the floor");

  /* Ladder: issue as soon as due, check gap == returned == stored interval. */
  const uint64_t expect[] = {180000, 360000, 720000, 1440000, 1800000, 1800000, 1800000};
  uint64_t now = 1000000, last_any = 0;
  for (int i = 0; i < 7; i++) {
    CHECK(ml_relay_refetch_due(&a, now, last_any, MIN_MS), "due when own schedule and floor allow");
    uint64_t ret = ml_relay_refetch_sent(&a, now, MIN_MS, MAX_MS);
    CHECK(ret == expect[i], "returned interval follows the ladder with cap");
    CHECK(a.interval_ms == ret, "stored interval equals returned interval");
    CHECK(a.next_ms - now == ret, "next_ms - now equals the returned interval (log matches schedule)");
    last_any = now;
    CHECK(!ml_relay_refetch_due(&a, a.next_ms - 1, last_any, MIN_MS), "not due one ms early");
    CHECK(ml_relay_refetch_due(&a, a.next_ms, last_any, MIN_MS), "due exactly at next_ms");
    now = a.next_ms;
  }

  /* Independence: regain on A does not touch B. */
  ml_relay_refetch_reset(&a, MIN_MS);
  ml_relay_refetch_reset(&b, MIN_MS);
  now = 5000000;
  (void)ml_relay_refetch_sent(&b, now, MIN_MS, MAX_MS); /* B -> 180 s */
  last_any = now;
  now = b.next_ms;
  (void)ml_relay_refetch_sent(&b, now, MIN_MS, MAX_MS); /* B -> 360 s */
  last_any = now;
  ml_relay_refetch_reset(&a, MIN_MS); /* A regains direct */
  CHECK(b.interval_ms == 360000 && b.next_ms == now + 360000, "A regain leaves B untouched");
  CHECK(a.interval_ms == MIN_MS && a.next_ms == 0, "A back at the floor");

  /* Fleet floor: A due by its own schedule but B fetched 10 s ago. */
  CHECK(!ml_relay_refetch_due(&a, now + 10000, last_any, MIN_MS), "floor blocks a second peer inside min_ms");
  CHECK(ml_relay_refetch_due(&a, now + MIN_MS, last_any, MIN_MS), "floor releases after min_ms");
  CHECK(!ml_relay_refetch_due(&b, now + MIN_MS, last_any, MIN_MS), "B still waits for its own 360 s");

  /* Refused enqueue: owner does not call _sent -> due time and interval unchanged, still due. */
  ml_relay_refetch_t c;
  ml_relay_refetch_reset(&c, MIN_MS);
  (void)ml_relay_refetch_sent(&c, 100, MIN_MS, MAX_MS);
  uint64_t n0 = c.next_ms, i0 = c.interval_ms;
  CHECK(ml_relay_refetch_due(&c, n0, n0 - 180000, MIN_MS), "due at its own next_ms");
  CHECK(c.next_ms == n0 && c.interval_ms == i0, "refused enqueue leaves schedule unchanged");
  CHECK(ml_relay_refetch_due(&c, n0 + 1000, n0 - 180000, MIN_MS), "still due on the next tick");

  /* Zero-initialised struct behaves as reset (slot reuse safety net). */
  ml_relay_refetch_t z = {0, 0};
  CHECK(
    ml_relay_refetch_sent(&z, 7, MIN_MS, MAX_MS) == 180000 && z.next_ms == 7 + 180000, "zero interval treated as min");

  printf("test_relay_refetch: %d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
