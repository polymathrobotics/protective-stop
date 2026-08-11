// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Host unit tests for the direct-path demote-verification verdict
 * (ml_demote_verdict.h). Fully host-runnable: the verdict is a pure function
 * of the demote triggers and the direct-rx freshness evidence. Style follows
 * test_clock_guard.c.
 *
 * Contract under test (2026-08-11 DUT direct-death design):
 *   - no trigger -> NONE, whatever the evidence says
 *   - trigger + safety peer + fresh authenticated direct WG rx -> VETO
 *   - trigger + safety peer + stale rx -> GO (a truly dead path must demote)
 *   - trigger + safety peer + no direct rx ever recorded -> GO
 *   - trigger + bulk (non-safety) peer -> GO even with fresh rx (behavior
 *     for the 16-peer tailnet load profile is unchanged)
 *   - freshness boundary is inclusive (age == fresh_ms still vetoes)
 */

#include <stdio.h>

#include "ml_demote_verdict.h"

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
  const uint32_t FRESH = 1000;

  /* No trigger: always NONE, even with terrible evidence. */
  CHECK(ml_demote_verdict(false, false, true, true, 0, FRESH) == ML_DEMOTE_NONE, "no trigger, fresh rx -> NONE");
  CHECK(ml_demote_verdict(false, false, true, false, 0, FRESH) == ML_DEMOTE_NONE, "no trigger, no rx -> NONE");
  CHECK(ml_demote_verdict(false, false, false, true, 999999, FRESH) == ML_DEMOTE_NONE, "no trigger, bulk -> NONE");

  /* Safety peer, fresh direct rx: veto on either trigger (and both). */
  CHECK(ml_demote_verdict(true, false, true, true, 200, FRESH) == ML_DEMOTE_VETO, "lease + fresh rx -> VETO");
  CHECK(ml_demote_verdict(false, true, true, true, 200, FRESH) == ML_DEMOTE_VETO, "pong-dead + fresh rx -> VETO");
  CHECK(ml_demote_verdict(true, true, true, true, 0, FRESH) == ML_DEMOTE_VETO, "both triggers + rx now -> VETO");

  /* Freshness boundary: inclusive at fresh_ms, GO just past it. */
  CHECK(ml_demote_verdict(true, false, true, true, FRESH, FRESH) == ML_DEMOTE_VETO, "age == fresh -> VETO");
  CHECK(ml_demote_verdict(true, false, true, true, FRESH + 1, FRESH) == ML_DEMOTE_GO, "age just stale -> GO");

  /* A truly dead path must still demote: stale rx or none ever. */
  CHECK(ml_demote_verdict(false, true, true, true, 5000, FRESH) == ML_DEMOTE_GO, "pong-dead + stale rx -> GO");
  CHECK(ml_demote_verdict(true, false, true, false, 0, FRESH) == ML_DEMOTE_GO, "lease + no rx ever -> GO");
  CHECK(ml_demote_verdict(true, true, true, false, 0, FRESH) == ML_DEMOTE_GO, "both + no rx ever -> GO");

  /* Bulk peers: verification never applies (original lease behavior). */
  CHECK(ml_demote_verdict(true, false, false, true, 0, FRESH) == ML_DEMOTE_GO, "bulk + fresh rx -> GO");
  CHECK(ml_demote_verdict(false, true, false, true, 100, FRESH) == ML_DEMOTE_GO, "bulk pong-dead + fresh rx -> GO");

  printf("test_demote_veto: %d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
