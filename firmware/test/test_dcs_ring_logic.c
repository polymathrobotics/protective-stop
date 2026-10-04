// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the pure PSTOP ring logic
// (firmware/components/dcs_support/src/dcs_ring_logic.c): segment colour,
// freshness, amber blink count, and the machine role's assigned-remote set.

#include <stdio.h>
#include <string.h>

#include "dcs_ring_logic.h"
#include "pstop/pstop_msg.h"

#define FRESH 2000u

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
  // 1. Segment state: link freshness first, then the machine's last reply.
  {
    CHECK(dcs_ring_segment(false, PSTOP_MESSAGE_OK) == DCS_RING_SEG_UNREACHABLE, "stale OK -> unreachable");
    CHECK(dcs_ring_segment(false, PSTOP_MESSAGE_STOP) == DCS_RING_SEG_UNREACHABLE, "stale STOP -> unreachable");
    CHECK(dcs_ring_segment(true, PSTOP_MESSAGE_OK) == DCS_RING_SEG_OK, "live OK -> green");
    CHECK(dcs_ring_segment(true, PSTOP_MESSAGE_STOP) == DCS_RING_SEG_STOP, "live STOP -> red");
    CHECK(dcs_ring_segment(true, PSTOP_MESSAGE_BOND) == DCS_RING_SEG_BOND, "live BOND -> blue");
    CHECK(dcs_ring_segment(true, PSTOP_MESSAGE_UNBOND) == DCS_RING_SEG_BOND, "live UNBOND -> blue");
    CHECK(dcs_ring_segment(true, PSTOP_MESSAGE_UNKNOWN) == DCS_RING_SEG_BOND, "live unknown -> blue");
  }

  // 2. Freshness window.
  {
    CHECK(!dcs_ring_fresh(0u, 5000u, FRESH), "never replied: not fresh");
    CHECK(dcs_ring_fresh(4000u, 5000u, FRESH), "1 s old: fresh");
    CHECK(dcs_ring_fresh(3001u, 5000u, FRESH), "1999 ms old: fresh");
    CHECK(!dcs_ring_fresh(3000u, 5000u, FRESH), "2000 ms old: stale");
    CHECK(!dcs_ring_fresh(6000u, 5000u, FRESH), "stamp in the future: not fresh");
  }

  // 3. Amber blink count names the deepest broken layer.
  {
    CHECK(dcs_ring_blinks(false, false) == 1, "peer silent, net fine: 1");
    CHECK(dcs_ring_blinks(false, true) == 2, "tailnet down: 2");
    CHECK(dcs_ring_blinks(true, false) == 3, "no Internet: 3");
    CHECK(dcs_ring_blinks(true, true) == 3, "no Internet wins: 3");
  }

  // 4. Machine: nothing configured and nothing served -> no segments (white).
  {
    dcs_ring_peer_t out[16];
    int n = dcs_ring_machine_peers(NULL, 0, NULL, 0, NULL, 0, NULL, 0, 10000u, FRESH, out, 16);
    CHECK(n == 0, "no remotes: white");
  }

  // 5. Machine: served remotes show the last reply sent to them.
  {
    const dcs_ring_reply_t rep[3] = {
      {.id = 0x0A, .last_msg = PSTOP_MESSAGE_OK, .last_ms = 9500u},
      {.id = 0x0B, .last_msg = PSTOP_MESSAGE_STOP, .last_ms = 5000u},
      {.id = 0u, .last_msg = PSTOP_MESSAGE_OK, .last_ms = 9900u},
    };
    dcs_ring_peer_t out[16];
    int n = dcs_ring_machine_peers(NULL, 0, NULL, 0, NULL, 0, rep, 3, 10000u, FRESH, out, 16);
    CHECK(n == 2, "two served remotes, empty entry skipped");
    CHECK(out[0].id == 0x0A && out[0].connected && out[0].last_msg == PSTOP_MESSAGE_OK, "A live OK");
    CHECK(out[1].id == 0x0B && !out[1].connected, "B went silent: stays assigned, unreachable");
    CHECK(dcs_ring_segment(out[1].connected, out[1].last_msg) == DCS_RING_SEG_UNREACHABLE, "B amber");
  }

  // 6. Machine: allowlisted / pinned remotes are assigned before they connect,
  //    listed first, each once; the denylist removes an id everywhere.
  {
    const uint32_t allow[2] = {0x01, 0x02};
    const uint32_t pin[2] = {0x02, 0x03};
    const uint32_t deny[1] = {0x03};
    const dcs_ring_reply_t rep[2] = {
      {.id = 0x02, .last_msg = PSTOP_MESSAGE_BOND, .last_ms = 9800u},
      {.id = 0x04, .last_msg = PSTOP_MESSAGE_STOP, .last_ms = 9900u},
    };
    dcs_ring_peer_t out[16];
    int n = dcs_ring_machine_peers(allow, 2, pin, 2, deny, 1, rep, 2, 10000u, FRESH, out, 16);
    CHECK(n == 3, "allow 1,2 + served 4; pinned 3 denied; 2 listed once");
    CHECK(out[0].id == 0x01 && !out[0].connected, "allowlisted, never connected: unreachable");
    CHECK(out[1].id == 0x02 && out[1].connected && out[1].last_msg == PSTOP_MESSAGE_BOND, "allowlisted, bonding");
    CHECK(out[2].id == 0x04 && out[2].connected, "served remote after configured ones");
  }

  // 7. Machine: a remote that unbonded and went quiet is dropped; while its
  //    UNBOND is fresh (e.g. mid re-bond) it still shows.
  {
    const dcs_ring_reply_t gone[1] = {{.id = 0x0C, .last_msg = PSTOP_MESSAGE_UNBOND, .last_ms = 1000u}};
    const dcs_ring_reply_t rebond[1] = {{.id = 0x0C, .last_msg = PSTOP_MESSAGE_UNBOND, .last_ms = 9900u}};
    const uint32_t allow[1] = {0x0C};
    dcs_ring_peer_t out[16];
    CHECK(
      dcs_ring_machine_peers(NULL, 0, NULL, 0, NULL, 0, gone, 1, 10000u, FRESH, out, 16) == 0,
      "unbonded + quiet: dropped");
    CHECK(
      dcs_ring_machine_peers(NULL, 0, NULL, 0, NULL, 0, rebond, 1, 10000u, FRESH, out, 16) == 1, "fresh UNBOND: shown");
    int n = dcs_ring_machine_peers(allow, 1, NULL, 0, NULL, 0, gone, 1, 10000u, FRESH, out, 16);
    CHECK(n == 1 && !out[0].connected, "allowlisted remote that unbonded: stays, unreachable");
  }

  // 8. Machine: output capped at max_out.
  {
    const uint32_t allow[5] = {1u, 2u, 3u, 4u, 5u};
    dcs_ring_peer_t out[3];
    memset(out, 0, sizeof(out));
    CHECK(dcs_ring_machine_peers(allow, 5, NULL, 0, NULL, 0, NULL, 0, 10000u, FRESH, out, 3) == 3, "capped at 3");
    CHECK(out[2].id == 3u, "cap keeps list order");
  }

  printf("dcs_ring_logic: %d checks, %d failures\n", g_checks, g_fails);
  return (g_fails == 0) ? 0 : 1;
}
