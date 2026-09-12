// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Host tests for the predicates actually used by DERP home retry. Health
 * entries model application registration/removal; no network or SDK needed. */
#include <stdio.h>

#include "ml_safety_retry.h"

static int g_checks;
static int g_fails;
#define CHECK(cond, what)             \
  do {                                \
    g_checks++;                       \
    if (!(cond)) {                    \
      g_fails++;                      \
      printf("  FAIL: %s\n", (what)); \
    }                                 \
  } while (0)

static void check_ladder(uint32_t priority, const ml_health_ent_t * peers, size_t count, bool expect_fast)
{
  bool safety = ml_safety_peers_present(priority, peers, count);
  CHECK(safety == expect_fast, "membership selects expected retry policy");
  CHECK(ml_derp_retry_gap_ms(safety, 0) == (expect_fast ? 5000u : 60000u), "first periodic attempt");
  CHECK(ml_derp_retry_gap_ms(safety, 1) == (expect_fast ? 5000u : 60000u), "second periodic attempt");
  CHECK(ml_derp_retry_gap_ms(safety, 2) == (expect_fast ? 10000u : 60000u), "third periodic attempt");
  CHECK(ml_derp_retry_gap_ms(safety, 3) == 60000u, "fourth attempt backs off");
  CHECK(ml_derp_retry_gap_ms(safety, 100) == 60000u, "saturated production burst stays bounded");
  CHECK(ml_derp_retry_gap_ms(safety, UINT32_MAX) == 60000u, "large burst cannot wrap to fast retries");

  /* Any-peer and per-peer predicates must agree on a quiescent registry. */
  bool any = ml_safety_peer_matches(priority, peers, count, priority);
  for (size_t i = 0; i < count; i++) {
    any |= ml_safety_peer_matches(priority, peers, count, peers[i].ip);
  }
  CHECK(safety == any, "any-peer membership agrees with per-peer membership");
  CHECK(!ml_safety_peer_matches(priority, peers, count, 0), "zero is never a safety peer");
  CHECK(!ml_safety_peer_matches(priority, peers, count, UINT32_MAX), "unregistered ordinary peer is not safety");
}

int main(void)
{
  ml_health_ent_t peers[16] = {0};
  const size_t count = sizeof(peers) / sizeof(peers[0]);
  const uint32_t machine_a = 0x646e233au;
  const uint32_t machine_b = 0x64509b6fu;

  /* No safety registrations: bulk traffic and reachability-only fleet/app
   * pins do not enter this array and cannot select the safety retry ladder. */
  check_ladder(0, peers, count, false);
  check_ladder(0, NULL, 0, false);
  check_ladder(machine_a, NULL, 0, true); /* Priority works before peer admission. */

  /* Observed DUT configuration: priority unset, registered target unhealthy.
   * A tail entry catches scans that accidentally inspect only the first slot. */
  peers[15].ip = machine_a;
  peers[15].healthy = false;
  check_ladder(0, peers, count, true);
  peers[15].healthy = true;
  check_ladder(0, peers, count, true); /* Liveness does not change eligibility. */

  peers[0].ip = machine_b;
  peers[0].healthy = false;
  check_ladder(0, peers, count, true);
  peers[15].ip = 0; /* Remove one target, keep the other. */
  check_ladder(0, peers, count, true);
  peers[0].ip = 0; /* Remove the last target; stale healthy bits are irrelevant. */
  check_ladder(0, peers, count, false);
  check_ladder(machine_a, peers, count, true); /* Priority survives health removal. */

  /* Late registration must be observed without resetting a cached count or
   * promoting a direct path. Reusing a removed slot works the same way. */
  peers[7].ip = machine_a;
  check_ladder(0, peers, count, true);
  peers[7].ip = 0;
  check_ladder(0, peers, count, false);

  /* Eligibility changes do not replenish the bounded retry burst. A late
   * first registration after three failures still uses the 60s interval. */
  CHECK(!ml_derp_retry_due(5001, 0, false, 0), "no safety peer retains 60s policy");
  peers[7].ip = machine_a;
  bool safety = ml_safety_peers_present(0, peers, count);
  CHECK(!ml_derp_retry_due(5000, 0, safety, 0), "due-time boundary remains strict");
  CHECK(ml_derp_retry_due(5001, 0, safety, 0), "late registration uses elapsed time on next evaluation");
  CHECK(!ml_derp_retry_due(5001, 0, safety, 3), "late registration after exhausted burst stays slow");
  CHECK(!ml_derp_retry_due(60000, 0, safety, 100), "large burst keeps strict 60s boundary");
  CHECK(ml_derp_retry_due(60001, 0, safety, 100), "large burst retries after 60s");
  peers[7].ip = 0;
  CHECK(
    !ml_derp_retry_due(5001, 0, ml_safety_peers_present(0, peers, count), 0),
    "last removal immediately cancels fast eligibility");

  /* Instant failed periodic attempts, starting last_retry=0. This preserves
   * the existing attempt-based fallback policy for health-only targets:
   * attempt four can move home after ~80s, but never through a LOCK/owner. */
  const uint64_t attempts[] = {5001, 10002, 20003, 80004};
  uint64_t last_retry = 0;
  peers[15].ip = machine_a;
  for (uint32_t burst = 0; burst < 4; burst++) {
    safety = ml_safety_peers_present(0, peers, count);
    CHECK(!ml_derp_retry_due(attempts[burst] - 1, last_retry, safety, burst), "periodic attempt is not early");
    CHECK(ml_derp_retry_due(attempts[burst], last_retry, safety, burst), "periodic retry ladder fires");
    bool enough = burst + 1 >= 4;
    CHECK(
      ml_derp_home_fallback_allowed(enough, 0, 0, true) == enough,
      "unlocked health-only home may move on fourth attempt");
    CHECK(!ml_derp_home_fallback_allowed(enough, 9, 0, true), "DUT region override 9 forbids moving home");
    CHECK(!ml_derp_home_fallback_allowed(enough, 0, 2, true), "re-home owner forbids fallback move");
    CHECK(!ml_derp_home_fallback_allowed(enough, 0, 0, false), "absent fallback cannot become home");
    last_retry = attempts[burst];
  }

  /* LOCK preserves home but still permits one rescue aux when the pool is
   * dark. Derive LOCK from the raw region override exactly as production. */
  const uint16_t region_override = 9;
  const uint16_t fallback_region = 2;
  uint16_t home_region = 9;
  int pending_slot = -1;
  int rescue_kicks = 0;
  for (int evaluation = 0; evaluation < 2; evaluation++) {
    if (ml_derp_home_fallback_allowed(true, region_override, 0, true)) {
      home_region = fallback_region;
    } else {
      int slot = ml_derp_rescue_aux_slot(true, true, false, pending_slot, 1);
      if (slot >= 0) {
        rescue_kicks++;
        pending_slot = slot; /* Successful kick remains pending until completion. */
      }
    }
  }
  CHECK(home_region == region_override, "locked home is preserved while rescue connects");
  CHECK(rescue_kicks == 1 && pending_slot == 1, "only one rescue aux kick while pending");
  CHECK(ml_derp_rescue_aux_slot(true, true, true, -1, 1) == -1, "live connection suppresses rescue");
  CHECK(ml_derp_rescue_aux_slot(false, true, false, -1, 1) == -1, "no rescue before fallback attempt threshold");
  CHECK(ml_derp_rescue_aux_slot(true, false, false, -1, 1) == -1, "no rescue without fallback candidate");
  CHECK(ml_derp_rescue_aux_slot(true, true, false, -1, -1) == -1, "full pool suppresses rescue");
  CHECK(ml_derp_rescue_aux_slot(true, true, false, -1, 0) == 0, "slot zero may be free aux after home moves");

  printf("test_safety_retry: %d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
