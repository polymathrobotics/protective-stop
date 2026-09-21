// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Host test for ml_wg_regain_policy.h — the WireGuard re-establishment
 * decision when a direct pong installs an endpoint for a peer with no session
 * (ml_wg_mgr.c process_disco_pong).
 *
 * Pins two bench findings (2026-09-19):
 *  - a SAFETY peer must get connect() with retries, not the bulk one-shot
 *    (relay dead + one-shot = stuck indefinitely);
 *  - inside the 5 s pace window the answer is NONE — the one-shot would clear
 *    peer->active and cancel the retries armed 200 ms earlier.
 *
 * Build/run: make -C host test
 */

#include <stdio.h>

#include "ml_wg_regain_policy.h"

static int fails = 0;
#define CHECK(cond, msg)                                   \
  do {                                                     \
    if (!(cond)) {                                         \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, msg); \
      fails++;                                             \
    }                                                      \
  } while (0)

int main(void)
{
  const bool SAFETY = true, BULK = false;
  const bool TRIED = true, FRESH = false;

  /* Safety peer, never connected: connect with retries, regardless of the
   * one-shot flag and of uptime (a first pong 3 s after boot must not wait). */
  CHECK(ml_wg_regain_action(SAFETY, FRESH, 3000, 0) == ML_WG_REGAIN_CONNECT_RETRYING, "safety first pong -> connect");
  CHECK(
    ml_wg_regain_action(SAFETY, TRIED, 3000, 0) == ML_WG_REGAIN_CONNECT_RETRYING,
    "safety never-connected, flag set -> connect");

  /* Pace window: the next pongs (several per second) do NOTHING — never the
   * one-shot, whichever state the flag is in. */
  CHECK(ml_wg_regain_action(SAFETY, TRIED, 10200, 10000) == ML_WG_REGAIN_NONE, "safety +200 ms -> none");
  CHECK(
    ml_wg_regain_action(SAFETY, FRESH, 10200, 10000) == ML_WG_REGAIN_NONE,
    "safety +200 ms, flag clear -> none (not one-shot)");
  CHECK(ml_wg_regain_action(SAFETY, TRIED, 14999, 10000) == ML_WG_REGAIN_NONE, "safety +4999 ms -> none");

  /* Window elapsed: re-arm. */
  CHECK(
    ml_wg_regain_action(SAFETY, TRIED, 15000, 10000) == ML_WG_REGAIN_CONNECT_RETRYING, "safety +5000 ms -> connect");
  CHECK(ml_wg_regain_action(SAFETY, TRIED, 60000, 10000) == ML_WG_REGAIN_CONNECT_RETRYING, "safety +50 s -> connect");

  /* Bulk peer: exactly one initiation, never paced retries. */
  CHECK(ml_wg_regain_action(BULK, FRESH, 3000, 0) == ML_WG_REGAIN_ONE_SHOT_INIT, "bulk first pong -> one-shot");
  CHECK(ml_wg_regain_action(BULK, TRIED, 3000, 0) == ML_WG_REGAIN_NONE, "bulk already tried -> none");
  CHECK(ml_wg_regain_action(BULK, TRIED, 99000, 0) == ML_WG_REGAIN_NONE, "bulk never retries on its own");

  /* Pace constant is WireGuard's REKEY_TIMEOUT. */
  CHECK(ML_WG_SAFETY_CONNECT_PACE_MS == 5000u, "pace = REKEY_TIMEOUT (5 s)");

  if (fails == 0) printf("test_wg_regain_policy: OK (11 checks)\n");
  return fails ? 1 : 0;
}
