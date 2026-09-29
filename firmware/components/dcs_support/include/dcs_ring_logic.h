// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
//
// Pure logic behind the 16-LED PSTOP ring: which colour a link segment shows,
// how many amber blinks name a connectivity loss, and (machine role) which
// remotes get a segment. No ESP-IDF, no FreeRTOS, no I/O, so firmware/test/
// can unit-test it on the host. Painting, timing and the RMT driver live in
// src/dcs_pstop_ring.c.
//
// Both roles show the MACHINE's state, so a remote's ring and its machine's
// ring agree: the remote paints one segment per configured machine from that
// machine's last reply; the machine paints one segment per assigned remote
// from the last reply it sent that remote.

#ifndef DCS_RING_LOGIC_H
#define DCS_RING_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* A link segment's display state. Order = severity for transition logging. */
typedef enum
{
  DCS_RING_SEG_UNREACHABLE = 0, /* no fresh reply: amber, N-blink layer code */
  DCS_RING_SEG_BOND = 1, /* last reply BOND/UNBOND: blue */
  DCS_RING_SEG_OK = 2, /* last reply OK: green (robot cleared to run) */
  DCS_RING_SEG_STOP = 3 /* last reply STOP: red */
} dcs_ring_seg_t;

/* Segment state from link freshness and the machine's last reply message
   * (PSTOP_MESSAGE_*). Anything other than OK/STOP on a live link is BOND. */
dcs_ring_seg_t dcs_ring_segment(bool connected, uint8_t last_msg);

/* True when a reply stamped last_ms (0 = never) is younger than fresh_ms at
   * time now. A stamp in the future (clock oddity) is not fresh. */
bool dcs_ring_fresh(uint64_t last_ms, uint64_t now_ms, uint32_t fresh_ms);

/* Amber blink count for an unreachable segment, naming the deepest broken
   * layer: 3 = no Internet, 2 = Tailscale down, 1 = the peer itself is silent. */
int dcs_ring_blinks(bool inet_down, bool tailnet_down);

/* Machine role: the last reply sent to one remote (id 0 = empty entry). */
typedef struct
{
  uint32_t id;
  uint8_t last_msg; /* PSTOP_MESSAGE_* */
  uint64_t last_ms; /* when it was sent */
} dcs_ring_reply_t;

/* Machine role: one ring segment. */
typedef struct
{
  uint32_t id;
  bool connected;
  uint8_t last_msg;
} dcs_ring_peer_t;

/* Machine role: the remotes that get a ring segment, in display order.
   *
   * Assigned remotes are the machine's analogue of the remote's configured
   * peers: every id on the allowlist or the pin list (persistent config),
   * then every remote the machine has served since boot (reply table),
   * each id once, minus the denylist. A remote whose last reply was UNBOND
   * and is no longer fresh has left on purpose and is dropped, unless it is
   * allowlisted or pinned. A configured remote with no reply yet is
   * unreachable. Returns the count written to out (at most max_out); 0 means
   * nothing is assigned and the ring shows white. */
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
  int max_out);

#ifdef __cplusplus
}
#endif

#endif /* DCS_RING_LOGIC_H */
