// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

#include "dcs_pstop_peers_logic.h"

#include <string.h>

static void put_u32(uint8_t * p, uint32_t v)
{
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);
  p[3] = (uint8_t)v;
}

static uint32_t get_u32(const uint8_t * p)
{
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

uint8_t dcs_pstop_peers_role_sanitize(uint8_t role)
{
  return (role == (uint8_t)PSTOP_AUX_ROLE_OPERATOR) ? (uint8_t)PSTOP_AUX_ROLE_OPERATOR
                                                    : (uint8_t)PSTOP_AUX_ROLE_STOP_ONLY;
}

bool dcs_pstop_peers_decode(
  const uint8_t * blob, size_t len, uint8_t legacy_role, dcs_pstop_peer_rec_t out[DCS_PSTOP_PEERS_MAX])
{
  (void)memset(out, 0, DCS_PSTOP_PEERS_MAX * sizeof(out[0]));
  for (int i = 0; i < DCS_PSTOP_PEERS_MAX; i++) {
    out[i].role = (uint8_t)PSTOP_AUX_ROLE_STOP_ONLY;
  }

  if (blob == NULL) {
    return false;
  }

  size_t rec_len;
  if ((blob[0] == DCS_PSTOP_PEERS_VER) && (len == DCS_PSTOP_PEERS_BLOB_LEN)) {
    rec_len = DCS_PSTOP_PEERS_REC_LEN;
  } else if ((blob[0] == DCS_PSTOP_PEERS_VER_V1) && (len == DCS_PSTOP_PEERS_BLOB_LEN_V1)) {
    rec_len = DCS_PSTOP_PEERS_REC_LEN_V1;
  } else {
    return false;
  }

  for (int i = 0; i < DCS_PSTOP_PEERS_MAX; i++) {
    const uint8_t * rec = &blob[1 + ((size_t)i * rec_len)];
    out[i].configured = (rec[0] != 0u);
    out[i].ip = get_u32(&rec[1]);
    out[i].port = (uint16_t)(((uint16_t)rec[5] << 8) | (uint16_t)rec[6]);
    out[i].machine_id = get_u32(&rec[7]);
    /* v1 predates the per-peer role: adopt the retired global key so a remote
     * promoted before the split keeps arming across the upgrade. */
    out[i].role = dcs_pstop_peers_role_sanitize((rec_len == DCS_PSTOP_PEERS_REC_LEN) ? rec[11] : legacy_role);
    if ((out[i].ip == 0u) || (out[i].port == 0u)) {
      out[i].configured = false; /* corrupt/cleared record degrades to empty */
    }
  }
  return true;
}

void dcs_pstop_peers_encode(const dcs_pstop_peer_rec_t recs[DCS_PSTOP_PEERS_MAX], uint8_t out[DCS_PSTOP_PEERS_BLOB_LEN])
{
  (void)memset(out, 0, DCS_PSTOP_PEERS_BLOB_LEN);
  out[0] = DCS_PSTOP_PEERS_VER;
  for (int i = 0; i < DCS_PSTOP_PEERS_MAX; i++) {
    uint8_t * rec = &out[1 + ((size_t)i * DCS_PSTOP_PEERS_REC_LEN)];
    rec[0] = recs[i].configured ? 1u : 0u;
    put_u32(&rec[1], recs[i].ip);
    rec[5] = (uint8_t)(recs[i].port >> 8);
    rec[6] = (uint8_t)recs[i].port;
    put_u32(&rec[7], recs[i].machine_id);
    rec[11] = dcs_pstop_peers_role_sanitize(recs[i].role);
  }
}
