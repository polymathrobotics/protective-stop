// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_AUX_CHANNEL_H
#define PSTOP_AUX_CHANNEL_H

/* Wrapper-only role schema over pstop_c v2 padding1. pstop_c remains
 * unmodified; every wrapper includes this header so the wire values stay in
 * sync. padding2 remains unused and zero-initialized by pstop_message_init(). */

#include <stdbool.h>
#include <stdint.h>

#include "pstop/pstop_msg.h"

/* padding1: bits 0..7 schema version, bits 8..15 role, bits 16..31 zero. */
#define PSTOP_AUX_UP_VERSION 0x01u
#define PSTOP_AUX_UP_ROLE_SHIFT 8u
#define PSTOP_AUX_BYTE_MASK 0xFFu

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

static inline void pstop_aux_encode_role(pstop_msg_t * msg, pstop_aux_role_t role)
{
  uint32_t v = (uint32_t)PSTOP_AUX_UP_VERSION;
  v |= ((uint32_t)((uint8_t)role & PSTOP_AUX_BYTE_MASK)) << PSTOP_AUX_UP_ROLE_SHIFT;
  msg->padding1 = v;
}

static inline pstop_aux_role_t pstop_aux_decode_role(const pstop_msg_t * msg)
{
  if (msg->version != PSTOP_VERSION) {
    return PSTOP_AUX_ROLE_UNSPECIFIED;
  }
  if ((msg->padding1 & PSTOP_AUX_BYTE_MASK) != PSTOP_AUX_UP_VERSION) {
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
      return PSTOP_AUX_ROLE_STOP_ONLY;
  }
}

static inline bool pstop_aux_role_is_operator(pstop_aux_role_t role)
{
  return role == PSTOP_AUX_ROLE_OPERATOR;
}

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

#endif /* PSTOP_AUX_CHANNEL_H */
