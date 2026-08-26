// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_AUX_CHANNEL_H
#define PSTOP_AUX_CHANNEL_H

/*
 * pstop auxiliary data channel — wrapper-only schema over the pstop_c v2
 * message padding (padding1/padding2, added in pstop_c #104).
 *
 * IMPORTANT: this header is NON-certified wrapper code. It NEVER modifies the
 * certified pstop_c library — it only reads/writes the two spare uint32 padding
 * fields that the library already serializes (covered by the frame CRC-16 and,
 * on the remote, by the dual-core lockstep byte-compare). All four wrappers
 * (firmware remote, machn machine, host machine, ROS 2 machine) include this
 * one header so the wire layout can never drift between them.
 *
 * Channel layout (each direction owns one 32-bit field; version byte first so
 * fields can grow without a size change):
 *
 *   padding1  = UPLINK   (remote -> machine)   carries the remote's role today
 *     bits  0.. 7  uplink schema version   (PSTOP_AUX_UP_VERSION)
 *     bits  8..15  role                     (pstop_aux_role_t)
 *     bits 16..31  reserved                 (zero)
 *
 *   padding2  = DOWNLINK (machine -> remote)   RESERVED for future machine
 *                                              feedback (e.g. relay/arm state).
 *     bits  0.. 7  downlink schema version  (0 = no feedback present today)
 *     bits  8..31  reserved                 (zero)
 *
 * Direction ownership is by sender: a remote-originated frame fills padding1
 * (role) and leaves padding2 = 0; a machine reply fills padding2 (feedback,
 * none yet) and leaves padding1 = 0. Because the wrapper operates on the
 * decoded pstop_msg_t (native-endian uint32s), this schema is endianness
 * agnostic — pstop_c handles the little-endian wire serialization.
 */

#include <stdbool.h>
#include <stdint.h>

#include "pstop/pstop_msg.h"

/* ------------------------------------------------------------------ uplink */

/* Uplink schema version. Bump only for an incompatible uplink layout change. */
#define PSTOP_AUX_UP_VERSION 0x01u

/*
 * Remote role, wire-stable values. A remote announces its own role; the
 * machine ANDs an OPERATOR claim with its existing allowlist (see design doc).
 *
 *   UNSPECIFIED : remote made no role claim (unprovisioned / version mismatch).
 *                 The machine treats this as non-operator (fail-safe).
 *   STOP_ONLY   : remote may only STOP, never re-arm. Monotonic toward safety.
 *   OPERATOR    : remote claims re-arm privilege (subject to machine policy).
 */
typedef enum
{
  PSTOP_AUX_ROLE_UNSPECIFIED = 0,
  PSTOP_AUX_ROLE_STOP_ONLY = 1,
  PSTOP_AUX_ROLE_OPERATOR = 2
} pstop_aux_role_t;

/* Byte offsets within the uplink uint32 (documentation / test helpers). */
#define PSTOP_AUX_UP_VERSION_SHIFT 0u
#define PSTOP_AUX_UP_ROLE_SHIFT 8u
#define PSTOP_AUX_BYTE_MASK 0xFFu

/*
 * Encode this remote's role into an outbound message's uplink field.
 * Owns padding1 entirely (overwrites any prior value). Leaves padding2 for the
 * downlink schema — callers that also want to zero downlink should use
 * pstop_aux_clear() first.
 */
static inline void pstop_aux_encode_role(pstop_msg_t * msg, pstop_aux_role_t role)
{
  uint32_t v = (uint32_t)PSTOP_AUX_UP_VERSION;
  v |= ((uint32_t)((uint8_t)role & PSTOP_AUX_BYTE_MASK)) << PSTOP_AUX_UP_ROLE_SHIFT;
  msg->padding1 = v;
}

/*
 * Decode a received message's uplink role, fail-safe.
 *   - unrecognized uplink version              -> UNSPECIFIED (defers to policy)
 *   - recognized version, unknown role byte    -> STOP_ONLY   (fail-safe)
 * A returned UNSPECIFIED is never treated as OPERATOR by the AND-rule, so both
 * outcomes deny arming unless an explicit, well-formed OPERATOR claim is made.
 */
static inline pstop_aux_role_t pstop_aux_decode_role(const pstop_msg_t * msg)
{
  if (((msg->padding1 >> PSTOP_AUX_UP_VERSION_SHIFT) & PSTOP_AUX_BYTE_MASK) != PSTOP_AUX_UP_VERSION) {
    return PSTOP_AUX_ROLE_UNSPECIFIED;
  }

  uint8_t role = (uint8_t)((msg->padding1 >> PSTOP_AUX_UP_ROLE_SHIFT) & PSTOP_AUX_BYTE_MASK);

  switch (role) {
    case PSTOP_AUX_ROLE_UNSPECIFIED:
      return PSTOP_AUX_ROLE_UNSPECIFIED;
    case PSTOP_AUX_ROLE_OPERATOR:
      return PSTOP_AUX_ROLE_OPERATOR;
    case PSTOP_AUX_ROLE_STOP_ONLY:
    default:
      /* Known-version but unknown role byte fails safe to stop-only. */
      return PSTOP_AUX_ROLE_STOP_ONLY;
  }
}

/* True only for a well-formed OPERATOR claim (the arming half of the AND-rule). */
static inline bool pstop_aux_role_is_operator(pstop_aux_role_t role)
{
  return role == PSTOP_AUX_ROLE_OPERATOR;
}

/* Human-readable role, for logs / state.json / diagnostics. */
static inline const char * pstop_aux_role_str(pstop_aux_role_t role)
{
  switch (role) {
    case PSTOP_AUX_ROLE_OPERATOR:
      return "operator";
    case PSTOP_AUX_ROLE_STOP_ONLY:
      return "stop_only";
    case PSTOP_AUX_ROLE_UNSPECIFIED:
    default:
      return "unspecified";
  }
}

/* ---------------------------------------------------------------- downlink */

/*
 * Downlink schema version. 0 = no machine->remote feedback present (today).
 * When machine feedback lands, bump to 0x01 and define the field layout here;
 * remotes already read the field via pstop_aux_read_feedback_raw() below, so no
 * wire or rollout change is needed then.
 */
#define PSTOP_AUX_DOWN_VERSION_NONE 0x00u

/*
 * Machine side: clear the downlink field on a reply (no feedback yet). Kept as
 * a named call so the future feedback encoder has an obvious home and every
 * machine wrapper is explicit about what it emits.
 */
static inline void pstop_aux_encode_feedback_none(pstop_msg_t * msg)
{
  msg->padding2 = 0u;
}

/*
 * Remote side: read the raw downlink field from a machine reply. Returns 0
 * while no feedback schema is active. Present today so the parse hook exists
 * and is exercised; interpret per PSTOP_AUX_DOWN_VERSION_* once defined.
 */
static inline uint32_t pstop_aux_read_feedback_raw(const pstop_msg_t * msg)
{
  return msg->padding2;
}

/* Zero both aux fields (uplink + downlink). */
static inline void pstop_aux_clear(pstop_msg_t * msg)
{
  msg->padding1 = 0u;
  msg->padding2 = 0u;
}

#endif /* PSTOP_AUX_CHANNEL_H */
