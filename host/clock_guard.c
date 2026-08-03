/* SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. */
/* SPDX-License-Identifier: Apache-2.0 */

/* Pure detection logic for the machine-side clock-freeze guard. See
 * clock_guard.h for the full rationale (SR-H-04b / FMEA DU-2). No syscalls
 * here on purpose: the caller samples the three clocks; this file only
 * reasons about them, so the whole detector is deterministically testable. */

#include "clock_guard.h"

#define CLOCK_GUARD_DEFAULT_WINDOW_MS 500u
#define CLOCK_GUARD_DEFAULT_MAX_CALLS 2000u

void clock_guard_init(clock_guard_t *g, uint64_t freeze_window_ms, uint32_t freeze_max_calls)
{
  g->freeze_window_ms = (freeze_window_ms != 0u) ? freeze_window_ms : CLOCK_GUARD_DEFAULT_WINDOW_MS;
  g->freeze_max_calls = (freeze_max_calls != 0u) ? freeze_max_calls : CLOCK_GUARD_DEFAULT_MAX_CALLS;
  g->initialized = 0;
  g->base_mono_ms = 0u;
  g->base_ref_ms = 0u;
  g->base_real_ms = 0u;
  g->stall_calls = 0u;
  g->fault = CLOCK_GUARD_OK;
}

clock_guard_fault_t clock_guard_fault(const clock_guard_t *g)
{
  return g->fault;
}

clock_guard_fault_t clock_guard_update(clock_guard_t *g, uint64_t mono_ms, uint64_t ref_ms, uint64_t real_ms)
{
  /* Sticky latch: once a clock fault is seen it is never cleared. A frozen or
   * backward machine clock is a hardware/kernel fault the app cannot recover
   * from in place; the runner holds STOP and (in a full system) would restart. */
  if (g->fault != CLOCK_GUARD_OK) {
    return g->fault;
  }

  if (!g->initialized) {
    g->base_mono_ms = mono_ms;
    g->base_ref_ms = ref_ms;
    g->base_real_ms = real_ms;
    g->stall_calls = 0u;
    g->initialized = 1;
    return CLOCK_GUARD_OK;
  }

  /* (1) Backward: the primary read is below the highest we have ever seen.
   * CLOCK_MONOTONIC must never regress; if it does, every heartbeat-timeout
   * arithmetic in check_heartbeats is compromised. Fail safe immediately. */
  if (mono_ms < g->base_mono_ms) {
    g->fault = CLOCK_GUARD_FAULT_BACKWARD;
    return g->fault;
  }

  /* (2) Healthy advance: rebase every baseline to "now" and clear the stall. */
  if (mono_ms > g->base_mono_ms) {
    g->base_mono_ms = mono_ms;
    g->base_ref_ms = ref_ms;
    g->base_real_ms = real_ms;
    g->stall_calls = 0u;
    return CLOCK_GUARD_OK;
  }

  /* (3) Stalled this update (mono_ms == base_mono_ms). Decide whether real time
   * has actually passed while the primary sat still, using signals independent
   * of the primary clock. Reference clocks that jump BACKWARD (e.g. an NTP
   * step on REALTIME) contribute 0 advance and can never manufacture a false
   * freeze — only forward motion counts as evidence that time elapsed. */
  g->stall_calls++;

  uint64_t ref_adv = (ref_ms > g->base_ref_ms) ? (ref_ms - g->base_ref_ms) : 0u;
  uint64_t real_adv = (real_ms > g->base_real_ms) ? (real_ms - g->base_real_ms) : 0u;
  uint64_t ref_adv_max = (ref_adv > real_adv) ? ref_adv : real_adv;

  /* An independent time source advanced past the window while the primary did
   * not move at all -> the primary clock is frozen. */
  if (ref_adv_max >= g->freeze_window_ms) {
    g->fault = CLOCK_GUARD_FAULT_FROZEN;
    return g->fault;
  }

  /* Backstop for the pathological case where EVERY clock source is wedged
   * (all three derive from a dead hardware clocksource) but the CPU loop keeps
   * spinning: the call-count proxy is the only remaining progress signal. */
  if (g->stall_calls >= g->freeze_max_calls) {
    g->fault = CLOCK_GUARD_FAULT_FROZEN;
    return g->fault;
  }

  return CLOCK_GUARD_OK;
}

const char *clock_guard_fault_name(clock_guard_fault_t f)
{
  switch (f) {
    case CLOCK_GUARD_OK:
      return "OK";
    case CLOCK_GUARD_FAULT_BACKWARD:
      return "BACKWARD";
    case CLOCK_GUARD_FAULT_FROZEN:
      return "FROZEN";
    default:
      return "?";
  }
}
