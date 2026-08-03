/* SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. */
/* SPDX-License-Identifier: Apache-2.0 */

/* ============================================================================
 * clock_guard — machine-side monotonic-clock freeze / backward detector.
 *
 * Discharges SR-H-04b / FMEA DU-2 (the FMEDA's dominant lambda_DU term): the
 * host machine's liveness watchdog (pstop_c check_heartbeats, machine.c) runs
 * entirely on the value returned by env.get_time_cb == the machine's
 * CLOCK_MONOTONIC. If that clock FREEZES or jumps BACKWARD, check_heartbeats
 * silently stops timing remotes out (machine.c:282 skips when
 * now <= last_timestamp) -> a dead/silent remote keeps looking alive -> the
 * robot keeps running on a stale liveness case. That is a dangerous-undetected
 * fault: detecting a clock's own freeze needs an INDEPENDENT reference.
 *
 * This module is PURE LOGIC: it takes three externally-sampled readings and a
 * monotonically-increasing call count, and returns a sticky fault verdict. It
 * performs no syscalls, so it is fully host-unit-testable with an injected
 * mock/stuck/backward clock (see host/test_clock_guard.c). The framework glue
 * that reads the three real clocks lives in machine_app_runner.c.
 *
 *   primary   : CLOCK_MONOTONIC   (the value fed to get_time_cb)
 *   reference : CLOCK_BOOTTIME    (independent monotonic source; keeps
 *               advancing even if MONOTONIC is administratively slewed)
 *   reference : CLOCK_REALTIME    (second independent source)
 *   proxy     : the loop/call count (advances with CPU progress even if EVERY
 *               clock source is wedged -> the truly independent liveness proof)
 *
 * Fail-safe reaction is always STOP: on any latched fault the runner forces
 * machine_stop_robot() and refuses to emit OK.
 * ========================================================================== */

#ifndef PSTOP_HOST_CLOCK_GUARD_H
#define PSTOP_HOST_CLOCK_GUARD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  CLOCK_GUARD_OK = 0,             /* primary clock healthy */
  CLOCK_GUARD_FAULT_BACKWARD = 1, /* primary clock jumped backward */
  CLOCK_GUARD_FAULT_FROZEN = 2,   /* primary clock stalled while time really passed */
} clock_guard_fault_t;

typedef struct {
  /* --- configuration (thresholds) --- */
  uint64_t freeze_window_ms; /* independent-reference advance, while the primary
                              * is stalled, that trips a FROZEN fault */
  uint32_t freeze_max_calls; /* call-count-proxy backstop: this many consecutive
                              * updates with no primary advance also trips FROZEN
                              * (covers an all-clocks-wedged silicon fault where
                              * only the spinning loop still makes progress) */

  /* --- state --- */
  int initialized;
  uint64_t base_mono_ms; /* highest primary value seen so far (advance baseline) */
  uint64_t base_ref_ms;  /* reference (BOOTTIME) at the last primary advance */
  uint64_t base_real_ms; /* reference (REALTIME) at the last primary advance */
  uint32_t stall_calls;  /* consecutive updates with no primary advance */
  clock_guard_fault_t fault; /* latched, sticky once tripped */
} clock_guard_t;

/* Initialize a guard. Pass 0 for either threshold to use the built-in default
 * (freeze_window_ms=500, freeze_max_calls=2000). */
void clock_guard_init(clock_guard_t *g, uint64_t freeze_window_ms, uint32_t freeze_max_calls);

/* Feed one fresh reading triple. mono_ms is the primary (get_time_cb) value;
 * ref_ms / real_ms are the two independent references. Returns the (sticky)
 * fault verdict. Once a fault is latched it is returned on every subsequent
 * call unchanged (fail-safe: a clock fault is never "un-seen"). */
clock_guard_fault_t clock_guard_update(clock_guard_t *g, uint64_t mono_ms, uint64_t ref_ms, uint64_t real_ms);

/* Current latched verdict without feeding a new sample. */
clock_guard_fault_t clock_guard_fault(const clock_guard_t *g);

/* Human-readable name for logging. */
const char *clock_guard_fault_name(clock_guard_fault_t f);

#ifdef __cplusplus
}
#endif

#endif /* PSTOP_HOST_CLOCK_GUARD_H */
