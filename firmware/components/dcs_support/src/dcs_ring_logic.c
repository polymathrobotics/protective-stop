// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
//
// Pure logic for the PSTOP ring. See dcs_ring_logic.h.
// Host-tested in firmware/test/test_dcs_ring_logic.c.

#include "dcs_ring_logic.h"

#include <stddef.h>

#include "pstop/pstop_msg.h" /* PSTOP_MESSAGE_OK / _STOP / _UNBOND */

dcs_ring_seg_t dcs_ring_segment(bool connected, uint8_t last_msg)
{
  if (!connected) {
    return DCS_RING_SEG_UNREACHABLE;
  }
  if (last_msg == PSTOP_MESSAGE_STOP) {
    return DCS_RING_SEG_STOP;
  }
  if (last_msg == PSTOP_MESSAGE_OK) {
    return DCS_RING_SEG_OK;
  }
  return DCS_RING_SEG_BOND;
}

bool dcs_ring_fresh(uint64_t last_ms, uint64_t now_ms, uint32_t fresh_ms)
{
  return (last_ms > 0u) && (now_ms >= last_ms) && ((now_ms - last_ms) < (uint64_t)fresh_ms);
}

int dcs_ring_blinks(bool inet_down, bool tailnet_down)
{
  if (inet_down) {
    return 3;
  }
  if (tailnet_down) {
    return 2;
  }
  return 1;
}

static bool id_in(const uint32_t * ids, int n, uint32_t id)
{
  for (int i = 0; i < n; i++) {
    if (ids[i] == id) {
      return true;
    }
  }
  return false;
}

static bool peer_listed(const dcs_ring_peer_t * out, int n, uint32_t id)
{
  for (int i = 0; i < n; i++) {
    if (out[i].id == id) {
      return true;
    }
  }
  return false;
}

static const dcs_ring_reply_t * reply_for(const dcs_ring_reply_t * replies, int n, uint32_t id)
{
  for (int i = 0; i < n; i++) {
    if (replies[i].id == id) {
      return &replies[i];
    }
  }
  return NULL;
}

/* Append one assigned remote unless it is empty, denied, a duplicate, or the
 * output is full. */
static int add_peer(
  dcs_ring_peer_t * out,
  int n,
  int max_out,
  uint32_t id,
  const uint32_t * deny,
  int ndeny,
  const dcs_ring_reply_t * replies,
  int nreplies,
  uint64_t now_ms,
  uint32_t fresh_ms)
{
  if ((n >= max_out) || (id == 0u) || id_in(deny, ndeny, id) || peer_listed(out, n, id)) {
    return n;
  }
  const dcs_ring_reply_t * r = reply_for(replies, nreplies, id);
  out[n].id = id;
  out[n].connected = (r != NULL) && dcs_ring_fresh(r->last_ms, now_ms, fresh_ms);
  out[n].last_msg = (r != NULL) ? r->last_msg : (uint8_t)PSTOP_MESSAGE_UNKNOWN;
  return n + 1;
}

int dcs_ring_machine_peers(
  const uint32_t * allow,
  int nallow,
  const uint32_t * pin,
  int npin,
  const uint32_t * deny,
  int ndeny,
  const dcs_ring_reply_t * replies,
  int nreplies,
  uint64_t now_ms,
  uint32_t fresh_ms,
  dcs_ring_peer_t * out,
  int max_out)
{
  int n = 0;
  for (int i = 0; i < nallow; i++) {
    n = add_peer(out, n, max_out, allow[i], deny, ndeny, replies, nreplies, now_ms, fresh_ms);
  }
  for (int i = 0; i < npin; i++) {
    n = add_peer(out, n, max_out, pin[i], deny, ndeny, replies, nreplies, now_ms, fresh_ms);
  }
  for (int i = 0; i < nreplies; i++) {
    const dcs_ring_reply_t * r = &replies[i];
    /* Released and gone: the remote unbonded and stopped talking. */
    if ((r->last_msg == PSTOP_MESSAGE_UNBOND) && !dcs_ring_fresh(r->last_ms, now_ms, fresh_ms)) {
      continue;
    }
    n = add_peer(out, n, max_out, r->id, deny, ndeny, replies, nreplies, now_ms, fresh_ms);
  }
  return n;
}
