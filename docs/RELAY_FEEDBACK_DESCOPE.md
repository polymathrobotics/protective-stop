<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Machine relay-feedback monitoring — DESCOPE (reversible)

Status: **DESCOPED (default OFF), pending a new machine hardware revision.**
Date: 2026-08. Scope: **machine role (`machn`) only.** The remote role is
unaffected.

This document is the single source of truth for the change; the in-code gate
comment (`machn/main/main.c`, `MACHN_RELAY_FEEDBACK_ENABLED`) and the safety-doc
banners all point here.

---

## 1. What was removed, and why

`machn` drives two stop-circuit relay coils (one per safety core) and — until
this change — also **read back** each relay's switched output through a
resistor-divider "sense" input, compared commanded-vs-observed every 100 ms
tick, raised a self-clearing relay fault on a persistent contradiction, and
after ~1 s of contradiction forced the robot STOPPED ("degraded redundancy =
don't run", operator decision 2026-07-31).

**That read-back (feedback) path is now compiled out.** The relays are still
**driven** exactly as before.

**Why:** a new machine-specific hardware revision and wiring diagram is
incoming on which the divider **feedback voltages cannot be read sanely**. Left
enabled, the sense inputs would return meaningless levels and manufacture
spurious relay faults (and, via the ~1 s contradiction rule, spurious STOPs).
Self-monitoring relays are expected to return on the new hardware, so this is a
**reversible gate, not a deletion** — the entire feedback implementation is
retained in-tree under `#if MACHN_RELAY_FEEDBACK_ENABLED`.

---

## 2. What changed (and what did NOT)

### Gated OFF (compiled out when `MACHN_RELAY_FEEDBACK_ENABLED == 0`)

- `relay_gpio_init()` — the `RELAY_A_SENSE` / `RELAY_B_SENSE` GPIOs are no
  longer configured (left untouched for the incoming wiring rework).
- `relay_feedback_check()` — no sense read, no commanded-vs-observed compare,
  no fault accumulation.
- The **feedback-driven STOP**: `if (g_relay_fault_stop) machine_stop_robot()`
  in `core_task()`. This was the **only** point where relay feedback fed back
  into the safety verdict; it is compiled out, so a feedback contradiction can
  no longer force STOP.
- The comparator's fault-latch / persistent-fault logging / `g_relay_fault_stop`
  assignment.

### UNCHANGED — the safety-critical drive path is intact

- **Relay drive**: each core still commands its own coil from its own
  `robot_state` verdict every tick (`gpio_set_level(drive, OK ? 1 : 0)`).
- **De-energize-to-safe series stop**: the two relay contacts remain wired in
  series in the stop circuit; **either** core opening its contact opens the
  chain and de-energizes the load. HFT = 1 on actuation is a **wiring**
  property and does not depend on feedback.
- **l0/l1 dual-core lockstep** (per-core `ROBOT_STATE_OK`, comparator
  agreement, arming/`restart_state`): **completely unchanged.** The machine's
  ARMED/green metric (`l0==1 && l1==1`) is independent of `relay_stop`, which
  was the feedback FAULT indicator — see also the `machn armed metric` note.
- **Clock-sanity fail-safe** (`MACHN_CLK_*`): unchanged; on a clock fault it
  still opens both relays directly and forces STOP on both cores.
- **Heartbeat/liveness timeouts, comparator withhold-on-disagreement,
  operator-allowlist arming policy**: all unchanged.

Net safety-behaviour delta: the machine **loses the diagnostic that detected a
stuck/welded contact and the associated stop-on-contradiction**. It does **not**
lose any ability to stop on demand. See §5.

---

## 3. Telemetry disposition when descoped

Existing `/state.json` fields keep their names and are reported as
**deterministically no-fault** so no downstream consumer misreads or
mis-triggers:

| field | descoped value | rationale |
|---|---|---|
| `relay_fault_a` / `relay_fault_b` | `0` | no fault computed; ROS 2 parses these as bools, `0` = no phantom fault |
| `relay_stop` | `0` | the ROS 2 hardware backend derives `need_stop` from `relay_stop` (`hardware_backend.cpp`), so it **must** be 0 to avoid a spurious stop |
| `relay_feedback_monitored` | `0` (**new field**) | lets consumers tell "monitored, no fault" (`=1`) apart from "not monitored" (`=0`). `0` on remotes too (they never publish it). |
| `e_hi0` / `e_hi1` (coil commanded) | live | still a real DRIVE-path signal — kept |
| `e_lo0` / `e_lo1` (divider observed) | `0` | not read; the admin page renders "feedback: not monitored" when `relay_feedback_monitored == 0` rather than showing a fake reading |

Rationale for **fixed 0/OK rather than a sentinel** in the legacy fields: the
ROS 2 machine bridge treats a truthy `relay_stop` as a stop demand and a truthy
`relay_fault_*` as a fault, and its JSON reader coerces any non-zero to `true`.
A sentinel (e.g. `0xFFFFFFFF`) would therefore have caused a **spurious stop and
phantom faults**. The explicit new `relay_feedback_monitored` flag carries the
"not monitored" meaning without that hazard.

**Downstream not yet updated (follow-up):** `ros2/protective_stop_machine`
(`MachineRelayStatus.msg` / `hardware_backend.cpp`) and the HIL suite
(`tools/hil/`) can adopt `relay_feedback_monitored` to label relay status as
n/a. Until then they see fault=0/stop=0 (benign).

---

## 4. How to reverse (re-enable on the new hardware)

1. In `machn/main/main.c`, set the gate to **1**:
   `#define MACHN_RELAY_FEEDBACK_ENABLED 1`
   (or define it `1` from the build, e.g. a CMake `target_compile_definitions`
   / Kconfig if that is preferred later — see the open question in §7).
2. **Re-verify for the new wiring** before trusting it: the `RELAY_A_SENSE` /
   `RELAY_B_SENSE` pin numbers, the divider ratios / logic levels, and
   `RELAY_FEEDBACK_MS` (the commanded→observed settle window) against the new
   hardware's actual part values. Update those constants as needed.
3. Rebuild and bench-verify: drive each relay open/closed and confirm the
   admin page shows `feedback: live/dead` matching the commanded state, and
   that an injected contradiction raises `relay_fault_*` and (after
   `RELAY_FAULT_STOP_TICKS`) `relay_stop`.
4. Revert the safety-doc descope banners (§6) and re-instate the FMEDA
   diagnostic-coverage credit once the feedback is re-validated on hardware.

No other code change is required to re-enable — the implementation is present,
only gated.

---

## 5. Safety-case impact

**Fail-safe stop capability is UNCHANGED.** The series two-relay architecture
still gives HFT = 1 on actuation: either contact opening de-energizes the stop
circuit. A single welded/stuck contact still results in a safe stop on the next
demand because the partner contact opens the chain.

**What is lost is diagnostic coverage of a stuck/welded final element:**

- Previously, a single welded contact produced a commanded-vs-observed
  contradiction that was (a) surfaced (`relay_fault_*`, log, ROS 2 diag) and
  (b) after ~1 s forced the robot STOPPED (don't run degraded). **Now a single
  weld is dangerous-UNDETECTED (latent)** until the next stop demand reveals it
  via the partner. The robot can therefore keep running on a single,
  non-redundant contact with no indication.
- The feedback resistor-divider self-diagnosis is also gone (the divider is not
  read).

**FMEDA effect (recompute required, not done here):** the relay-weld element
(`MC-2`) loses its ~90 % diagnostic coverage → its dangerous-undetected rate
rises accordingly, and the feedback-divider element (`MC-3`) drops out of the
diagnostic. SPFM / PMHF for the machine channel must be **recomputed**; expect a
reduced metric. This detailed recompute is deferred (see §6) and must be signed
off by the safety owner.

**Requirement status:** `SG-5` / `SR-SYS-05` are written with an **OR**:
"de-energize-to-safe with HFT = 1 **and diagnosable stuck-on (relay-output
feedback)**, **OR** the integrator provides an element of ≥ the allocated
integrity." With feedback descoped, `machn` no longer self-satisfies the
"diagnosable stuck-on" limb. The requirement now rests on either (a) the
integrator-supplied final element, or (b) restoration of feedback on the new
hardware. Until then the stuck-on **diagnosis** is a documented **gap**; the
**stop-on-demand** obligation remains met by the series relays.

**Compensating measures while descoped:**

- The **proof-test / commanded-open cycle** already recommended in FMEA §5 for
  the latent *double*-weld now also bounds *single*-weld detection and becomes
  more important. Recommend the integrator perform a periodic commanded-open
  proof test of the stop circuit.
- No change to the always-safe direction: de-energize on any demand, silence,
  disagreement, or clock fault continues to work.

---

## 6. Safety documents touched (descope banners)

Per the lighter-touch reconciliation option chosen for this change (a prominent
descope banner now; full numeric reconciliation to follow with safety-owner
sign-off), the following were annotated — **not rewritten** — to mark the
feedback credit as descoped-pending-new-hardware, reversible, with the residual
risk noted, and to point here:

- `docs/safety/FMEA.md` — §5 relay-weld narrative.
- `docs/safety/FMEDA.md` — `MC-2` (relay-weld DC) and `MC-3` (feedback divider)
  rows; the credited DC is now stale pending recompute.
- `docs/safety/HARA.md` — `H-10` residual note and `SG-5`.
- `docs/safety/SAFETY_REQUIREMENTS.md` — `SR-SYS-05` status.
- `docs/safety/OPEN_ITEMS.md` — new open item registering the descope +
  pending FMEDA recompute.

The full numeric FMEDA recompute (SPFM/PMHF with MC-2 DC → 0 % and MC-3 removed)
is intentionally **deferred** to a follow-up with safety-owner sign-off, rather
than guessed here.

---

## 7. Open questions for the safety owner (Ilia)

1. **Reversibility mechanism** — implemented as a compile-time
   `#define MACHN_RELAY_FEEDBACK_ENABLED` (recommended: single-file, no
   build-system surface, `#ifndef`-guarded so a build flag can still override).
   Confirm, or switch to a Kconfig `CONFIG_MACHN_RELAY_FEEDBACK` if menuconfig
   discoverability is preferred.
2. **Telemetry disposition** — implemented as fixed `0`/OK on the legacy fields
   **plus** a new `relay_feedback_monitored=0` flag (rationale in §3: a sentinel
   in the legacy fields would spuriously stop the ROS 2 bridge). Confirm, or
   drop the new flag for plain fixed-0.
3. **Safety-doc scope** — implemented as descope banners + an open-items entry
   now, with the numeric FMEDA recompute deferred. Confirm, or request the full
   reconciliation in this change.
