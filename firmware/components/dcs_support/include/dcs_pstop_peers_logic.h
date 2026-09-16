// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
//
// Pure codec for the multi-machine peer table (ps_peers NVS blob), including
// the v1->v2 migration that adds the per-peer role. No ESP-IDF, no FreeRTOS,
// no I/O, so firmware/test/ can unit-test it on the host. The NVS glue lives
// in src/dcs_nvs.c.

#ifndef DCS_PSTOP_PEERS_LOGIC_H
#define DCS_PSTOP_PEERS_LOGIC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pstop_aux_channel.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Peer slots. Mirrors DCS_PSTOP_MAX_MACHINES in dcs_support.h; dcs_nvs.c
 * static-asserts that the two agree. */
#define DCS_PSTOP_PEERS_MAX 4

  /* One configured machine. `role` is what this remote announces to THIS peer
   * in padding1 of every frame addressed to it; a remote is stop-only toward
   * every peer it has not been explicitly promoted for. */
  typedef struct
  {
    bool configured;
    uint32_t ip; /* host byte order */
    uint16_t port;
    uint32_t machine_id; /* pstop_msg.receiver_id for this machine */
    uint8_t role; /* pstop_aux_role_t; STOP_ONLY or OPERATOR */
  } dcs_pstop_peer_rec_t;

/* Blob layout, byte-serialized (no struct padding on the wire):
 *   [0]            format version
 *   per slot, DCS_PSTOP_PEERS_MAX records:
 *   [0]            used (0/1)
 *   [1..4]         ip, big-endian, host-order value
 *   [5..6]         port, big-endian
 *   [7..10]        machine_id, big-endian
 *   [11]           role (v2 only)
 */
#define DCS_PSTOP_PEERS_VER 2u
#define DCS_PSTOP_PEERS_REC_LEN 12u
#define DCS_PSTOP_PEERS_BLOB_LEN (1u + (DCS_PSTOP_PEERS_MAX * DCS_PSTOP_PEERS_REC_LEN))

/* v1 had no role byte. Kept so decode can migrate an existing table. */
#define DCS_PSTOP_PEERS_VER_V1 1u
#define DCS_PSTOP_PEERS_REC_LEN_V1 11u
#define DCS_PSTOP_PEERS_BLOB_LEN_V1 (1u + (DCS_PSTOP_PEERS_MAX * DCS_PSTOP_PEERS_REC_LEN_V1))

  /* Parse a stored blob. `legacy_role` is the value of the retired global role
   * key, adopted by every slot of a v1 blob so a remote already promoted before
   * the per-peer split keeps arming across the upgrade.
   *
   * Returns false on an unusable length or version, leaving out[] zeroed; the
   * caller then falls back to its own single-peer migration. A record whose ip
   * or port is zero degrades to unconfigured, and any role byte that is not
   * OPERATOR degrades to STOP_ONLY. */
  bool dcs_pstop_peers_decode(
    const uint8_t * blob, size_t len, uint8_t legacy_role, dcs_pstop_peer_rec_t out[DCS_PSTOP_PEERS_MAX]);

  /* Serialize at the current version. A role that is not OPERATOR is written as
   * STOP_ONLY, so an uninitialized record can never persist as an operator. */
  void dcs_pstop_peers_encode(
    const dcs_pstop_peer_rec_t recs[DCS_PSTOP_PEERS_MAX], uint8_t out[DCS_PSTOP_PEERS_BLOB_LEN]);

  /* Fail-safe narrowing applied by both codec halves: only an explicit
   * OPERATOR grants re-arm authority. */
  uint8_t dcs_pstop_peers_role_sanitize(uint8_t role);

#ifdef __cplusplus
}
#endif

#endif /* DCS_PSTOP_PEERS_LOGIC_H */
