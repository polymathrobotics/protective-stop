// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* WireGuard re-establishment decision on a direct-path (re)gain — pure,
 * host-testable (host/test_wg_regain_policy.c).
 *
 * Context (ml_wg_mgr.c process_disco_pong): a direct pong just installed an
 * endpoint for a peer that has NO valid WireGuard session. Two kinds of peer
 * want two different reactions:
 *
 *  - SAFETY peer (the machine host): connect() with WireGuard's own
 *    REKEY_TIMEOUT retries. Its wireguard-go initiates only when it has data,
 *    and it has nothing to send while we are silent — a one-shot that then
 *    waits for the peer to initiate deadlocks (bench 2026-09-19: relay dead,
 *    stuck indefinitely). Paced to REKEY_TIMEOUT: pongs arrive several times a
 *    second and every connect() resets the pending handshake, so a slow path
 *    could otherwise keep answering an already-superseded initiation. Inside
 *    the pace window the answer is NONE — never the one-shot, which clears
 *    peer->active and would cancel the retries just armed.
 *
 *  - BULK tailnet peer: one initiation, active cleared. If it has us trimmed
 *    it will never answer and retries would run forever.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ML_WG_SAFETY_CONNECT_PACE_MS 5000u /* = WireGuard REKEY_TIMEOUT */

typedef enum
{
  ML_WG_REGAIN_NONE = 0, /* leave the pending handshake alone           */
  ML_WG_REGAIN_CONNECT_RETRYING, /* wireguardif_connect(): active, auto-retries  */
  ML_WG_REGAIN_ONE_SHOT_INIT, /* single initiation, active cleared afterwards */
} ml_wg_regain_action_t;

/* last_safety_connect_ms == 0 means "never connected". */
static inline ml_wg_regain_action_t ml_wg_regain_action(
  bool is_safety_peer, bool tried_initial_handshake, uint64_t now_ms, uint64_t last_safety_connect_ms)
{
  if (is_safety_peer) {
    if (last_safety_connect_ms == 0u || (now_ms - last_safety_connect_ms) >= ML_WG_SAFETY_CONNECT_PACE_MS) {
      return ML_WG_REGAIN_CONNECT_RETRYING;
    }
    return ML_WG_REGAIN_NONE;
  }
  return tried_initial_handshake ? ML_WG_REGAIN_NONE : ML_WG_REGAIN_ONE_SHOT_INIT;
}
