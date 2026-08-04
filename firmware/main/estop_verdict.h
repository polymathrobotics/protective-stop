// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
//
// Pure E-stop decision core — no HAL, no I/O, no ESP-IDF. Extracted from
// main.c so the SIL-critical verdict logic is unit-testable to branch + MC/DC
// on the host (the on-target Xtensa build cannot be gcov-instrumented: its JTAG
// pins ARE the E-stop loop pins GPIO39-42, and USB-Serial-JTAG is displaced by
// the USB-NCM tether). See firmware/test/ and docs/safety/COVERAGE.md.

#ifndef ESTOP_VERDICT_H
#define ESTOP_VERDICT_H

#include <stdbool.h>
#include <stdint.h>

// Release-direction debounce: after ANY unhealthy read, this many consecutive
// healthy ticks are required before the channel reports closed again. The
// open->STOP edge stays SINGLE-TICK — the stop path is never filtered; this
// only extends how long a STOP episode lasts, so EMC-induced blips (measured
// 100-200 ms both-channel flaps under WiFi TX, 2026-07-21) produce a >=300 ms
// episode instead of chattering — defence-in-depth under the machine-side
// min-STOP-duration arming policy (which is the enforcement point).
#define LOOP_RECLOSE_DEBOUNCE_TICKS 3U

// Boot warm-up: the comparator sends NOTHING until each channel has settled —
// either one full closed-debounce cycle (loops proven healthy) or this many
// CONSECUTIVE open reads (button genuinely held at boot -> STOP flows, just
// ~500 ms later, well before the pstop bond completes). Without the
// consecutive-open requirement, the first loop sample glitching open for a tick
// (observed on every boot of this board) put a ~200 ms STOP->OK episode on the
// wire — the arming gesture — at every power-on.
#define LOOP_BOOT_OPEN_CONFIRM_TICKS 5U

// Per-core E-stop channel state, carried across ticks. One instance per core.
typedef struct
{
  bool high_ok;  // most recent drive-high tick read IN==1 (closed)
  bool low_ok;  // most recent drive-low  tick read IN==0 (not shorted)
  bool primed_high;  // a drive-high sample has been taken since boot
  bool primed_low;  // a drive-low  sample has been taken since boot
  uint8_t closed_streak;  // consecutive healthy ticks (release debounce)
  uint8_t open_streak;  // consecutive unhealthy ticks (boot warm-up only)
  bool settled;  // debounce warm-up done: one full closed-debounce cycle
    // observed, OR LOOP_BOOT_OPEN_CONFIRM_TICKS consecutive
    // open reads (button held at boot). Until BOTH channels
    // settle, the comparator's boot-priming hold sends nothing.
} estop_state_t;

// Decide this tick's pstop message byte from the two FRESH both-phase reads
// (rb_hi = readback after driving the loop HIGH; rb_lo = after driving it LOW),
// updating the per-core state (health flags, debounce, priming). Returns
// PSTOP_MESSAGE_OK or PSTOP_MESSAGE_STOP. Pure: no HAL, no globals.
//
// core_id selects the diverse verdict expression (Option B): core 0 by
// arithmetic image (table-index), core 1 by boolean — logically identical when
// correct, so a systematic bug in one diverges the lockstep comparator.
uint8_t estop_decide(estop_state_t * st, int core_id, int rb_hi, int rb_lo);

// True once BOTH cores have sampled both phases AND settled (see the boot
// warm-up rationale above). The comparator holds off sending until then.
bool estop_channels_primed(const estop_state_t st[2]);

#endif  // ESTOP_VERDICT_H
