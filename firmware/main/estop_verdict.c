// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

#include "estop_verdict.h"

#include "pstop/pstop_msg.h"  // PSTOP_MESSAGE_OK / PSTOP_MESSAGE_STOP

uint8_t estop_decide(estop_state_t *st, int core_id, int rb_hi, int rb_lo)
{
  // The OK codeword is selected by the ARITHMETIC IMAGE of both fresh reads —
  // index 0 (OK) iff rb_hi==1 AND rb_lo==0, computed with no interpretable
  // boolean (`(rb_hi^1) | rb_lo` is 0 only when both physically matched what we
  // drove THIS tick). So OK cannot be produced by a stale/latched flag or by a
  // fault in the health/debounce logic below — every check there is a STOP-ONLY
  // override (may raise msg to STOP, never lower it to OK). Encoding-agnostic:
  // the physical image is only a table INDEX, so this survives an upstream move
  // of OK/STOP to far-Hamming values. (OK==0 today is fail-danger polarity
  // inherited from pstop_c; this live-sample derivation is the compensating
  // measure — see docs/safety/FMEA.md.)
  //
  // DIVERSITY (Option B): the two cores form the verdict by INDEPENDENT
  // expressions of the same physical reads. Core 0 selects the codeword by the
  // arithmetic image (table-index); core 1 by a boolean of the same reads. They
  // are logically identical when correct (so no lockstep-divergence in normal
  // operation), but a SYSTEMATIC bug in either expression makes only that core
  // wrong -> the comparator's memcmp diverges -> nothing is sent -> the machine
  // stops on heartbeat liveness. This turns the lockstep compare from mere
  // redundancy (identical code, identical error) into real diversity against a
  // common-mode interpretation fault.
  static const uint8_t k_estop_msg[2] = {PSTOP_MESSAGE_OK, PSTOP_MESSAGE_STOP};
  uint8_t msg;
  if (core_id == 0) {
    msg = k_estop_msg[(unsigned)((rb_hi ^ 1) | rb_lo) & 1u];
  } else {
    msg = ((rb_hi == 1) && (rb_lo == 0)) ? PSTOP_MESSAGE_OK : PSTOP_MESSAGE_STOP;
  }

  st->high_ok = (rb_hi == 1);
  st->low_ok = (rb_lo == 0);
  st->primed_high = true;
  st->primed_low = true;

  const bool raw_closed = st->high_ok && st->low_ok;

  // Asymmetric release debounce: open reports IMMEDIATELY (streak reset),
  // closed only after LOOP_RECLOSE_DEBOUNCE_TICKS consecutive healthy ticks.
  // Both cores run this identically on their own channel, so the lockstep
  // encodings stay in agreement through the debounce window.
  if (raw_closed) {
    if (st->closed_streak < (uint8_t)255U) {
      st->closed_streak++;
    }
    st->open_streak = 0U;
    if (st->closed_streak >= LOOP_RECLOSE_DEBOUNCE_TICKS) {
      st->settled = true;
    }
  } else {
    st->closed_streak = 0U;
    if (st->open_streak < (uint8_t)255U) {
      st->open_streak++;
    }
    if (st->open_streak >= LOOP_BOOT_OPEN_CONFIRM_TICKS) {
      st->settled = true;  // held open: STOP flows
    }
  }

  // STOP-ONLY overrides: the two-phase integrity + release debounce may only
  // RAISE msg to STOP — they can never lower it to OK. `msg` is already STOP
  // whenever this tick's live sample didn't match the driven level, so a fault
  // in this health/debounce logic cannot manufacture an OK.
  if (!(raw_closed && (st->closed_streak >= LOOP_RECLOSE_DEBOUNCE_TICKS))) {
    msg = PSTOP_MESSAGE_STOP;
  }
  return msg;
}

bool estop_channels_primed(const estop_state_t st[2])
{
  return st[0].primed_high && st[0].primed_low && st[1].primed_high &&
         st[1].primed_low && st[0].settled && st[1].settled;
}
