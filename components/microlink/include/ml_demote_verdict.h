// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Demote-verification verdict — pure, host-testable (host/test_demote_veto.c).
 *
 * Direct-path demotion (ml_wg_mgr.c disco maintenance) is decided by the DISCO
 * side-channel: a trust-lease expiry or the priority-peer pong watchdog. But
 * disco rides its own UDP flow; on a jittery uplink (USB-NCM tether) its pings
 * can go silent while the WireGuard DATA flow — the 5 Hz safety heartbeat —
 * still arrives on the direct path. Demoting then TEARS DOWN A WORKING DIRECT
 * PATH (wireguardif_connect_derp clears the endpoint) and dumps the safety
 * heartbeat onto a relay measured too jittery to hold green (2026-08-11: DUT
 * relay rtt spikes >1.2 s in BOTH regions vs the 2.0 s pstop timeout).
 *
 * Verdict: authenticated WG data received ON THE DIRECT PATH within fresh_ms
 * is stronger liveness evidence than a missing disco pong — hold the path
 * (VETO). A truly dead path stops producing direct rx, the age blows past
 * fresh_ms within ~fresh_ms, and demotion proceeds unchanged (GO). Safety
 * (priority/health-tracked) peers only: bulk tailnet peers keep the plain
 * lease behavior.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  ML_DEMOTE_NONE = 0, /* no demote trigger — leave the path alone       */
  ML_DEMOTE_VETO, /* trigger fired, but direct WG rx proves liveness */
  ML_DEMOTE_GO, /* trigger fired and nothing vouches for the path  */
} ml_demote_verdict_t;

static inline ml_demote_verdict_t ml_demote_verdict(
  bool lease_expired,
  bool pong_dead,
  bool is_safety_peer,
  bool direct_rx_age_valid, /* false = no direct data rx ever recorded */
  uint32_t direct_rx_age_ms,
  uint32_t fresh_ms)
{
  if (!lease_expired && !pong_dead) return ML_DEMOTE_NONE;
  if (is_safety_peer && direct_rx_age_valid && direct_rx_age_ms <= fresh_ms) return ML_DEMOTE_VETO;
  return ML_DEMOTE_GO;
}
