// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_AUX_CHANNEL_H
#define PSTOP_AUX_CHANNEL_H

/* Wrapper-only role schema over pstop_c v2 padding1. pstop_c remains
 * unmodified; every wrapper includes this header so the wire values stay in
 * sync. padding2 remains unused and zero-initialized by pstop_message_init(). */

#include <stdbool.h>
#include <stdint.h>

#include "pstop/machine.h"
#include "pstop/pstop_msg.h"
#include "pstop/pstop_remote_data.h"

/* padding1: bits 0..7 schema version, bits 8..15 role, bits 16..31 zero. */
#define PSTOP_AUX_UP_VERSION 0x01u
#define PSTOP_AUX_UP_ROLE_SHIFT 8u
#define PSTOP_AUX_BYTE_MASK 0xFFu

/*
 * Remote role, wire-stable values. The REMOTE alone declares its role and the
 * machine honours it on every frame (see pstop_aux_apply_role_pre/post below);
 * the machine keeps no operator list. Whether a remote may BOND at all is a
 * separate, optional machine-side admission decision (allow/denylist).
 *   UNSPECIFIED : remote made no role claim (unprovisioned / version mismatch).
 *                 The machine treats this as non-operator (fail-safe).
 *   STOP_ONLY   : remote may only STOP, never re-arm. Monotonic toward safety.
 *   OPERATOR    : remote may re-arm (STOP -> OK gesture).
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

/*
 * Live role enforcement — shared by every machine wrapper (machn, ROS 2, host)
 * so the policy is identical everywhere. pstop_c is untouched; these use only
 * its public structs. Call _pre right before machine_process_message() and
 * _post right after, for every frame from an already-bonded remote (a NEW
 * remote's is_stop_only is seeded by remote_details_cb at BOND).
 *
 * Why both halves are needed: pstop_c consults is_stop_only only when a STOP
 * ACQUIRES ownership of the arming cycle (remote_stop_id == 0). Once a remote
 * owns the cycle (it armed the machine), its later STOP re-opens the cycle
 * unconditionally and its OK re-arms. And a STOP from a non-owner stop-only
 * remote while armed leaves restart_state at OK, so its release would re-arm
 * too. The two hooks close both holes:
 *   _pre  : a remote announcing stop-only can never OWN the cycle — release
 *           ownership; a half-open cycle it owned is voided (NEED_STOP).
 *   _post : an accepted STOP from a stop-only remote never opens an arming
 *           cycle — force NEED_STOP so its OK cannot complete the gesture.
 * Net effect: a remote that demotes itself while the machine is armed keeps
 * it running; the next STOP from it stops the machine and it cannot re-arm
 * until some remote announcing OPERATOR performs STOP -> OK.
 */
static inline void pstop_aux_apply_role_pre(pstop_machine_t * machine, const pstop_msg_t * req)
{
  pstop_remote_data_t * c = pstop_remote_get(&machine->remotes, &req->id);
  if (c == NULL) {
    return;
  }
  c->is_stop_only = !pstop_aux_role_is_operator(pstop_aux_decode_role(req));
  if (c->is_stop_only && (machine->robot_state.remote_stop_id == c->local_remote_id)) {
    machine->robot_state.remote_stop_id = 0U;
    if (machine->robot_state.restart_state == ROBOT_RESTART_STATE_STOP_RECEIVED) {
      machine->robot_state.restart_state = ROBOT_RESTART_STATE_NEED_STOP;
    }
  }
}

static inline void pstop_aux_apply_role_post(pstop_machine_t * machine, const pstop_msg_t * req, pstop_error_t err)
{
  if ((err != PSTOP_OK) || (req->message != PSTOP_MESSAGE_STOP)) {
    return;
  }
  const pstop_remote_data_t * c = pstop_remote_get(&machine->remotes, &req->id);
  if ((c != NULL) && c->is_stop_only) {
    machine->robot_state.restart_state = ROBOT_RESTART_STATE_NEED_STOP;
  }
}

#endif /* PSTOP_AUX_CHANNEL_H */
