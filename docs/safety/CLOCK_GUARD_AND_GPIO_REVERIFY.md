<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Machine clock-freeze guard (SR-H-04b / DU-2) + E-stop GPIO pad re-verify (SR-R-09 / DU-1)

**Branch:** `safety/clock-guard-gpio-reverify`
**Standard frame:** IEC 61508 (SIL 3 / PL e allocated). Safe state = **STOP =
de-energized**. Fail-safe reaction of every mechanism below is **STOP**.

This note closes the **two highest-priority Dangerous-Undetected rows** in
`docs/safety/FMEA.md` §3 — the two the FMEDA (`docs/safety/FMEDA.md` §7.4)
ranks #1 and #2 by PFH contribution:

| DU | Failure | Requirement | Mechanism added here | Validation |
|---|---|---|---|---|
| **DU-2** | Machine `get_time_cb` frozen/backward → heartbeat watchdog blind → dead remote looks alive | **SR-H-04b**, SR-I-03 | Host: independent-reference clock guard on `machine_now_ms`. machn: esp_timer-vs-FreeRTOS-tick sanity + watchdogs. | Host **fault-injection unit test** (pass); machn build-verified, HW validation deferred |
| **DU-1** | Lost E-stop pull-down → open loop floats → false-CLOSED → **false OK** | **SR-R-09** | Remote: ~1 s GPIO pad-config register read-back on both IN pins → latch → halt TX → STOP | **On-target fault injection** on `10.43.0.195` (OTA), reverted |

The remote-side **frozen-clock** DU-2 (the chip's own `esp_timer`) was already
closed by the dual-core cross-check documented in
`docs/WATCHDOG_CLOCK_PROTECTION.md`. This note adds the **machine-side** clock
term (host runner + `machn`) — the FMEDA's *dominant* λ_DU (MC-4, β≈1) — and the
remote **GPIO** term.

---

## Part A — SR-H-04b: machine-side clock-freeze guard (DU-2)

### A.1 Why it is dangerous

The machine's liveness watchdog is `pstop_c`'s `check_heartbeats`
(`pstop_c/.../machine.c:267-320`), driven entirely by the value returned from
`env.get_time_cb`. On the host runner that callback is `machine_now_ms()` reading
`CLOCK_MONOTONIC`. `check_heartbeats` computes `diff = now - last_timestamp` and
**skips the remote entirely when `now <= last_timestamp`** (machine.c:282). So if
`CLOCK_MONOTONIC` **freezes** or steps **backward**:

- every remote's `diff` is 0 (or the branch is skipped) → the timeout can never
  be reached → **a silent/dead remote is never STOPped** → the robot keeps
  running on a stale liveness case. Dangerous, and **undetected** — the clock is
  upstream of the entire verdict, and *you cannot detect your own clock's freeze
  by reading that same clock*. An independent reference is required (FMEA
  H03-1 / F-P-03).

### A.2 Host runner — `host/machine_app_runner.c` + `host/clock_guard.{c,h}`

A small, pure, unit-tested module (`clock_guard.c`) cross-checks the primary
clock against **three independent progress signals**:

- **`CLOCK_BOOTTIME`** — an independent monotonic reference (keeps advancing even
  if `CLOCK_MONOTONIC` is administratively slewed);
- **`CLOCK_REALTIME`** — a second independent reference (only *forward* motion
  counts as evidence, so an NTP step-back can never manufacture a false fault);
- **loop/call-count proxy** — the truly independent signal: the CPU keeps
  executing and the guard keeps being called even if **every** clock source is
  wedged. This is what catches the classic DU-2 (a `get_time_cb` stuck at a
  constant).

`machine_now_ms()` still returns `CLOCK_MONOTONIC` exactly as before (the library
sees no change), but every call also feeds the guard; the main loop additionally
polls it once per iteration. Detection rules (all in the pure module):

- **Backward** (`mono < highest-seen`) → latch `FAULT_BACKWARD` immediately.
- **Frozen** (`mono` not advancing) **and** an independent reference advanced
  past the **500 ms window** → latch `FAULT_FROZEN`. 500 ms is well under the
  ~1.2 s heartbeat timeout, so a freeze is caught *before* it can mask a dead
  remote.
- **Frozen** and the **call-count backstop** (2000 stalled calls) is reached
  (all clocks wedged, only the loop advancing) → latch `FAULT_FROZEN`.

The fault is **sticky** (a clock fault is never un-seen). On any latched fault
the main loop **forces `machine_stop_robot()`** (STOP + `status_cb(STOP)`),
**refuses to process inbound traffic** (so no path can commit an OK), and holds
STOP; in a full deployment an external watchdog then restarts the host.

### A.3 machn (ESP32-S3 machine node) — `machn/main/main.c`

The machn's liveness runs on `esp_timer` via a **single latched** `g_tick_now_ms`
(FMEDA MC-4, β≈1 — the dominant term). The comparator now cross-checks
`esp_timer_get_time()` against `xTaskGetTickCount()` (a **different** time base):
a frozen or backward `esp_timer` while the scheduler still ticks → **force STOP**
(open both series relays + `machine_stop_robot` on both cores) → controlled
`esp_restart()` after a 500 ms STOP-settle. The full watchdog stack
(TWDT 5 s / IWDT 300 ms / bootloader-RTC) is enabled in `machn/sdkconfig.defaults`
and the comparator subscribes+feeds the TWDT — mirroring the remote
(`docs/WATCHDOG_CLOCK_PROTECTION.md`).

This is the direct machine-side analogue of the remote's dual-core clock
cross-check.

---

## Part B — SR-R-09: E-stop GPIO pad-config re-verification (DU-1)

### B.1 Why it is dangerous

The two E-stop IN pins (GPIO40 ch A, GPIO42 ch B) are configured **once** in
`estop_init()` as *input + pull-down ENABLED* (`firmware/main/main.c:215-222`), so
an **open** loop discharges through the pull-down and reads 0 → STOP. That config
was **never re-verified at runtime**. If the pull-down is lost (an SEU on the
IO_MUX config register, a stray pad reconfig, ESD), an open loop **floats** and
can echo 1 on the drive-high phase → **false CLOSED → false OK**, with no online
detection. This is **DU-1**, the FMEA's single most dangerous gap; the design doc
claimed a runtime "config-integrity check" that **never existed in code**
(RECONCILIATION R-02).

Option A (live both-phase sampling) does **not** help: it trusts the physical
read, and here the *sensor path physically lies*.

### B.2 Mechanism — `firmware/main/main.c`

Every **~1 s** the comparator reads back each IN pad's configuration registers
via `gpio_ll_get_io_config()` (ESP-IDF v5.5 `hal/gpio_ll.h`), which decodes the
same IO_MUX/GPIO registers `gpio_config()` wrote:

| Field | Register bit | Required | Meaning |
|---|---|---|---|
| `.pd` | IO_MUX `FUN_WPD` | **1** | pull-down enabled — **the DU-1 bit** |
| `.ie` | IO_MUX `FUN_IE` | **1** | input enabled (else the read is dead) |
| `.oe` | GPIO output-enable | **0** | input direction, not a driven output |
| `.pu` | IO_MUX `FUN_WPU` | **0** | no pull-up (a pull-*up* would actively hold an open loop CLOSED — worst case) |

Any mismatch **latches a sticky per-channel fault bit** (`g_gpio_cfg_fault`,
bit0 = ch A / GPIO40, bit1 = ch B / GPIO42). The comparator then fail-safes
**exactly like the clock cross-check**: it **halts all pstop TX**, so every
machine's heartbeat times out to **STOP**. There is **no auto-reset** — a lost
pull-down is a hardware fault a reboot would only *transiently* mask (because
`estop_init` re-enables it), so the device stays safely silent until serviced.
The state is exported to `/state.json` as `gpio_cfg_fault` for observability.

---

## Validation evidence

### V.1 Host clock guard — fault-injection unit test (Part A, host)

`make -C host test` builds and runs `host/test_clock_guard.c`, which injects
mock/stuck/backward clock triples into the pure detector:

```
clock_guard: 5117 checks, 0 failures
```

Cases asserted (each maps to an SR-H-04 clause):

- **Frozen monotonic + advancing reference** → `FAULT_FROZEN`, tripped within the
  ~500 ms window (the classic DU-2: `get_time_cb` stuck at a constant).
- **All clocks wedged, only the loop advancing** → `FAULT_FROZEN` via the
  call-count backstop.
- **Backward monotonic jump** → `FAULT_BACKWARD` immediately.
- **Healthy 5000-poll run incl. a REALTIME step-back** → never trips (no false
  positive).
- **Sticky latch** → a "recovered" clock still reports the fault.

The runner + guard build clean (`make -C host`) and a live smoke on **port 8896**
comes up with **no false clock fault**.

### V.2 Remote GPIO pad re-verify — on-target fault injection (Part B, `10.43.0.195`)

A **throwaway** build (never committed) cleared `FUN_WPD` on channel A's IN pad
(GPIO40) at runtime after ~20 s — simulating a lost pull-down — then reverted.
OTA-only, polling `/state.json` (`t0`=core-0 tick, `sent`=pstop TX count,
`rr`=reset_reason, `boot`=OTA boot-count):

```
t=21 up=32546 rr=3 boot=0 t0=194 gpio_cfg_fault=0 sent=0   healthy, ~1 s before injection
t=22 up=33597 rr=3 boot=0 t0=199 gpio_cfg_fault=1 sent=0   FUN_WPD cleared -> re-verify CAUGHT it
t=23 up=34635 rr=3 boot=0 t0=199 gpio_cfg_fault=1 sent=0   TX HALTED (core ticks frozen) = STOP, in reset grace
t=24 (no response — controlled reset firing)
t=25 (no response)
t=26 (no response)
t=27 up=9051  rr=3 boot=0 t0=0   gpio_cfg_fault=0 sent=0   back up: CLEAN SW reset (rr=3), boot=0, pad re-initialized
```

Proof points:

- **`gpio_cfg_fault` 0 → 1** the instant the pull-down was cleared — the re-verify
  detected the lost pull-down (the DU-1 signature) even though the loop was still
  *physically* closed (`e=11/11`): a config-integrity check, not a level check.
- **TX halted → STOP:** core ticks froze at 199 and `sent` stayed 0 — the
  comparator entered the fail-safe halt (every machine heartbeat-times-out).
- **Clean controlled reset:** the device returned with `reset_reason=3`
  (`ESP_RST_SW`) and `boot_count=0` — the intentional controlled reset, **not**
  the rollback-counted `ESP_RST_TASK_WDT`; on reboot `estop_init` re-wrote the
  pad and it ran healthy again (transient-SEU recovery).

**Injection reverted → clean build stable** (final committed firmware, no
injection): loops advance in lockstep, `gpio_cfg_fault=0`, `pstop_mismatch=0`, no
false fault, across a continuous run past the old injection point:

```
up=15632 t0=26  t1=26  gpio_cfg_fault=0 xfault=0 mm=0   loops in lockstep (t0==t1)
up=28110 t0=150 t1=150 gpio_cfg_fault=0 xfault=0 mm=0   past the old injection tick, no trip
up=55791 t0=427 t1=427 gpio_cfg_fault=0 xfault=0 mm=0   stable, no false fault
```

The re-verify runs every ~1 s (it re-reads the pads continuously) and does
**not** false-trip on a correctly-configured pad.

### V.3 machn clock guard — DEFERRED

`machn/main/main.c` builds clean (`machn_machine.bin`). On-hardware validation on
`100.84.155.111` is **deferred**: it is a **live machine node**, confirming the
STOP *actuation* needs relay HIL feedback, and a throwaway `esp_timer`-freeze
injection would induce a controlled-reset loop — not appropriate to flash onto a
live node without a bench. The logic is the direct analogue of the remote
dual-core cross-check that **is** fault-injection validated
(`docs/WATCHDOG_CLOCK_PROTECTION.md` §4). **Follow-up:** validate on a bench machn
with a 4-ch relay HIL (inject a frozen `esp_timer`, assert both relay drives go
low and the node controlled-resets).

---

## Requirement discharge

| Req | Before | After |
|---|---|---|
| **SR-H-04** (frozen/backward machine clock → STOP) | **Gap** | **Satisfied (host)** — clock guard + fault-injection unit test; machn analogue implemented (HW validation deferred) |
| **SR-I-03** (never-freezing `get_time_cb`) | Partially satisfied | Reinforced — freeze/backward now *detected*, not just assumed |
| **SR-R-09** (periodic GPIO pad re-verify → STOP on lost pull-down) | **Gap** (highest priority) | **Satisfied** — ~1 s read-back on both IN pins; on-target fault injection confirms trip → STOP; clean build stable |

FMEDA impact (`docs/safety/FMEDA.md` §7.4): closing DU-2 (rank 1) moves the
dominant ~3×10⁻⁹/h λ_DU term to λ_DD on the host; SR-R-09 (rank 2) moves the
DU-1 pull-down term. Both raise SFF toward the ≥ 99 % that gives unambiguous
SIL 3 with HFT margin. The machn term remains DU until its HW validation closes.

---

## Residual gaps

1. **machn HW validation** — code-complete + build-verified, on-target proof
   deferred (V.3). Until then the machine-side clock term stays DU on the machn.
2. **Genuine shared-clocksource freeze on the host.** `CLOCK_MONOTONIC`,
   `CLOCK_BOOTTIME`, and `CLOCK_REALTIME` all derive from one Linux clocksource;
   a dead clocksource freezes all three. The **call-count proxy** is the
   backstop for exactly that case, but it is best-effort — a true belt-and-braces
   solution is an **external hardware watchdog** on the host (already recommended
   in FMEA DU-2 / SR-H-04). Documented, not eliminated.
3. **DU-6 (both pull-downs lost together, common-cause).** SR-R-09 re-verifies
   *both* channels independently, so a common-cause loss is now *detected* — but
   a brownout detector wired to the safe state (SR-R-10) is still a separate open
   item for the sub-tick window before the next ~1 s re-verify.
4. **Re-verify latency.** The pad check runs at ~1 s cadence; a pull-down lost
   *and* an E-stop opened within the same ≤ 1 s window is covered by the machine
   heartbeat/PST budget (≤ 1.3 s, SR-SYS-01) but the pad-fault itself is detected
   only at the next tick. Tightening the cadence trades CPU for latency; 1 s
   matches the design doc's claim and is well inside the PST.
