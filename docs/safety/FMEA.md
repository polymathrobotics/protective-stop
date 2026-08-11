<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Protective-Stop — FMEA (first draft)

**Status:** Engineering first draft for safety-lead curation. Keyed to the
function IDs in `docs/safety/SYSTEM_DEFINITION.md` (F-R-xx remote firmware,
F-H-xx host machine, F-M-xx ROS2 machine, F-P-xx pre-qualified `pstop_c`
interface). Analysis is of the **code as it stands**, honoring the
reconciliation block: Option A + Option B **implemented**, Option C
**dropped**. Where `PSTOP_SAFETY_DESIGN.md` §6/§10 disagree with the code, the
code wins and the discrepancy is flagged (see §6 of this document).

**Standard frame:** IEC 61508 (systematic capability + FMEDA feed) + ISO 13849
(PL for the actuation side). Target **SIL 3 / PL e**. Safe state = **STOP =
de-energized**. Inherited fail-danger polarity: `OK == 0x00`, `STOP == 0x01`
(`pstop_c/pstop/include/pstop/pstop_msg.h:15-16`).

---

## 1. Method

Each **item** (a code unit or hardware element) realizes one or more
**functions** (F-xx). For every function I enumerate credible **failure modes**
and classify their end effect on the three safety functions (SF-1 stop-on-
demand, SF-2 no-spurious-run, SF-3 arming-integrity) along two axes: **Safe vs
Dangerous** (does the fault tend toward or away from the de-energized safe
state?) and **Detected vs Undetected** (is there an existing on-line mechanism
that catches it and forces the safe reaction?). Criticality is scored
**S**everity × **O**ccurrence × **D**etection, each 1–5 (5 = worst: most
severe / most frequent / least detectable), RPN = S·O·D. Severity is near-
uniformly high because every dangerous failure of a SIL 3 stop function is
potentially catastrophic; the discriminating axes are Occurrence and
Detection. **Dangerous-Undetected (DU)** rows carry D = 4–5 by definition and
are the population that governs the SIL 3 / PFH case — they are extracted into
§3. This qualitative FMEA is the input to (a) the safety-requirements set (the
"candidate requirement hook" column) and (b) a later quantitative **FMEDA**
(diagnostic-coverage / SFF apportionment); the S/O/D here are engineering
estimates, not measured failure rates.

Scoring key: **Class** ∈ {Safe-Det, Safe-Undet, **Dang-Det**, **Dang-Undet**}.
Line/file citations are to the analyzed tree.

---

## 2. FMEA table

### F-R-01 — Dual-channel E-stop loop sensing (`main.c:265` `estop_channel_closed`)

| FM ID | Failure mode | Cause | Local effect | System effect (SF) | Existing detection | Existing mitigation | Class | S/O/D | Residual gap | Candidate requirement |
|---|---|---|---|---|---|---|---|---|---|---|
| R01-1 | Button pressed / loop opened | Normal demand | drive-high echoes 0 → `rb_hi≠1` | SF-1 STOP as intended | `estop_channel_closed:274-308` both-phase read | pull-down input, STOP-only override `:345` | Safe-Det | 5/5/1 | — | Verify per-channel STOP latency ≤ PST |
| R01-2 | Single-channel wire break / one pole open | Random HW (connector, cable) | one core reads OPEN, other CLOSED → encodings diverge | SF-1 STOP (device silenced) | comparator memcmp `main.c:890` | transmit-on-agreement; machine heartbeat-timeout | Safe-Det | 5/3/1 | — | Confirm memcmp covers message+CRC bytes |
| R01-3 | Sense input **floats high on drive-high, low on drive-low** (pull-down lost) | Random HW / SEU on GPIO config reg / F-R-10 or other code reconfiguring the pad | `rb_hi==1 && rb_lo==0` on an **open** loop → false CLOSED → false OK | **SF-2 breach: spurious OK** | **NONE at runtime** — pad config set once at `estop_init:240-247`, never re-verified | one-time boot config only | **Dang-Undet** | 5/2/5 | pull-down integrity never re-checked; design §6/§10#8 claim a "config-integrity check" that **does not exist in code** | Periodic GPIO config-register read-back + drive-low-must-read-0 self-test on both channels; force STOP on mismatch |
| R01-4 | Both channels' pull-downs lost together | Common-cause: brownout, shared pad-mux bug, ESD | both cores false-CLOSED, **agree** → OK transmitted | **SF-2 breach, undetected by lockstep** | none (memcmp passes on agreement) | — | **Dang-Undet** | 5/1/5 | β≈1 for the shared config path; see §4 | Diverse pull-down verification + brown-out detector wired to safe state |
| R01-5 | `ESTOP_SETTLE_US`=10µs too short for wire+contact RC | Systematic (timing) / long cable | drive-level not yet propagated when sampled → intermittent false OPEN or false CLOSED | false OPEN=Safe; **false CLOSED=SF-2** | debounce only smooths OPEN→CLOSED | `LOOP_RECLOSE_DEBOUNCE_TICKS` (release only) | Dang-Undet | 4/2/4 | settle time is a bench constant, not validated against max cable length | Characterize loop RC vs deployed cable; derive settle time with margin |
| R01-6 | GPIO read returns stale/cached level | Silicon errata / driver | this-tick sample not fresh | defeats Option A live-sample premise | — | — | Dang-Undet | 4/1/5 | no read-back proof the sample is of *this* drive | Add read-after-write ordering barrier; document HW guarantee |

### F-R-02 — Per-core verdict formation, Option A + Option B (`main.c:373` `compute_verdict`, `:265-348`)

| FM ID | Failure mode | Cause | Local effect | System effect (SF) | Existing detection | Existing mitigation | Class | S/O/D | Residual gap | Candidate requirement |
|---|---|---|---|---|---|---|---|---|---|---|
| R02-1 | Stale/latched health flag manufactures OK | Systematic SW | — | (blocked) | Option A: OK codeword = arithmetic image of **this-tick** reads `:305`; overrides are STOP-only `:345-347` | live-sample derivation | Safe-Det | 5/2/2 | Compensates the `OK==0` polarity **for generation only** (see §3) | Assert OK is unreachable without a fresh matched sample (unit + MC/DC) |
| R02-2 | Systematic misinterpretation of `rb_hi/rb_lo` in one core's expression | Systematic SW | that core wrong, other correct → diverge | SF-1 STOP | Option B: core0 arithmetic-image `:305` vs core1 boolean `:307` → memcmp diverges | diverse expressions | Safe-Det | 4/2/2 | see R02-3 | Keep the two expressions provably non-convergent |
| R02-3 | Compiler collapses the two "logically identical" expressions to identical object code | Systematic (toolchain) | Option B diversity **evaporates** at object level → a shared codegen bug hits both identically and **agrees** | **SF-2 potential** | none (memcmp passes) | source-level diversity only | **Dang-Undet** | 4/2/4 | comment `:299` itself states they are "logically identical when correct"; single toolchain, same ISA — no compiler/ISA diversity | Compile-diverse the two verdict paths (different TU/flags) OR disassemble-and-diff as a build gate to prove non-identical machine code |
| R02-4 | Shared STOP-only override / debounce block buggy (`:317-347`) | Systematic SW (identical on both cores) | both cores mis-handle identically → agree | direction: override can only **raise** to STOP; a bug that fails to raise is masked by R02-1 (msg already STOP on mismatch) | memcmp cannot catch (identical) | STOP-biased structure | Safe-Undet | 3/2/4 | β≈1 here but fault direction is safe | MC/DC on the override block; prove monotonic (STOP-only) property by test |
| R02-5 | `k_estop_msg[]` table (core0 only) corrupted | Random RAM/flash | core0 wrong, core1 uses constants → diverge | SF-1 STOP | memcmp `:890` | table read by one core only | Safe-Det | 4/1/2 | — | — |

### F-R-03 — Release debounce + boot warm-up (`main.c:191,200,323-363`)

| FM ID | Failure mode | Cause | Local effect | System effect (SF) | Existing detection | Existing mitigation | Class | S/O/D | Residual gap | Candidate requirement |
|---|---|---|---|---|---|---|---|---|---|---|
| R03-1 | `estop_primed()` returns true too early | Systematic SW (single copy, `:359`) | boot-priming STOP→OK artifact reaches machine → auto-completes arming with no operator | **SF-3 breach** | machine `min_stop_ms`=500ms veto (boot glitch ~200ms) | comparator holds send until primed `:903` | Dang-Det | 5/2/3 | `estop_primed` is a **non-diverse single point**; SF-3 rests on machine-side min-stop | Cross-core-agreed prime flag; unit-test the boot-glitch-never-arms invariant (design §10 companion) |
| R03-2 | Debounce filters the OPEN→STOP edge | Systematic SW | STOP delayed/suppressed | **SF-1 delay** | — | code path resets `closed_streak` immediately on any unhealthy read `:332`; STOP is single-tick | Safe-Det | 5/1/2 | verify no path filters STOP direction | Test: STOP edge is never debounced |
| R03-3 | `closed_streak`/`open_streak` uint8 wrap | Systematic | counter saturates at 255 (guarded `:324,333`) | none | saturating guard | explicit `<255` clamp | Safe-Undet | 2/1/3 | — | — |

### F-R-04 — Lockstep comparator + relay send (`main.c:802` `comparator_task`)

| FM ID | Failure mode | Cause | Local effect | System effect (SF) | Existing detection | Existing mitigation | Class | S/O/D | Residual gap | Candidate requirement |
|---|---|---|---|---|---|---|---|---|---|---|
| R04-1 | One core task starves / misses 80ms publish | Scheduling, WG-task preemption | `both_in=false` | SF-1 STOP (send nothing) | `CORE_PUBLISH_TIMEOUT` `:881`, mismatch counter | safety tasks at prio 8 above WG `:1131` | Safe-Det | 4/2/2 | — | Monitor `pstop_mismatch` climb as a health signal |
| R04-2 | Stale `g_done` give satisfies next tick prematurely | Race (late publish) | comparator memcmps stale buffer | could compare wrong-tick data | pre-drain of both semaphores `:873-874` | explicit take-0 before notify | Safe-Det | 4/1/3 | correctness depends on drain ordering | Assert each take corresponds to this-tick encode |
| R04-3 | **RAM fault flips STOP→OK in `g_encoded[0][i]` after memcmp, before `sess_sendto`** | Random RAM/SEU | transmitted message differs from compared message | SF-2 window | CRC over [0..37] now stale → machine rejects INVALID_CHECKSUM → no state change | machine default-STOP, next-tick resend, heartbeat | Dang-Det | 4/1/3 | single-tick STOP could be lost if machine already OK; recovered next tick | Send the *compared* buffer via a read-once + re-verify, or CRC-check just before send |
| R04-4 | Comparator task itself hangs/deadlocks | Systematic SW / socket block | no sends | SF-1 STOP (machine timeout) | none in-remote; machine heartbeat | non-blocking socket discipline `:437`; TWDT (`dcs_safety.c`) covers CPU hang not logic hang | Safe-Det | 4/2/2 | a *logically* wedged-but-scheduled comparator is not TWDT-visible; machine timeout still saves it | Machine-side liveness is the backstop — keep heartbeat mandatory |
| R04-5 | `memcmp` returns 0 on non-equal (miscompiled/short len) | Systematic SW | divergence missed | SF-2 | none | uses `PSTOP_MESSAGE_SIZE` length | Dang-Undet | 5/1/4 | trust in libc memcmp; length constant correct (40) | Unit-test comparator with injected 1-byte diff at each offset |

### F-R-05 / F-R-06 — Encode + black-channel round-trip (`main.c:409-423`, `:724-794`)

| FM ID | Failure mode | Cause | Local effect | System effect (SF) | Existing detection | Existing mitigation | Class | S/O/D | Residual gap | Candidate requirement |
|---|---|---|---|---|---|---|---|---|---|---|
| R05-1 | Systematic encode bug produces identical wrong bytes on both cores | Systematic SW in `pstop_message_encode` (shared, pre-qualified) | both encodings identical-but-wrong → **agree** | SF-2 if the wrong byte is the message field | memcmp cannot catch (identical) | `[PQ]` pstop_c coverage/CI | Dang-Undet | 4/1/4 | Option B does **not** diversify encoding; lockstep is blind to common encode faults | Interface req F-P-01: layout/CRC-span unaltered; add a second independent encoder image on one core for the message byte |
| R06-1 | Remote adopts spoofed/replayed reply counter unconditionally (`:779`) | Comms (on-tunnel spoof/replay) | remote echoes attacker counter to machine | machine sees wrong `received_counter` → MSG_LOST → **no OK** | src IP/port check `:741`; CRC check `:746`; machine echo validation (F-P-04) | fail-safe direction (tends STOP) | Safe-Undet | 3/1/3 | black-channel confidentiality assumed (WG/Tailscale); no app-layer replay window on the reply path | Document black-channel integrity assumption; consider reply counter-monotonic guard |
| R06-2 | `sess_send_period_ms` adopts hostile `heartbeat_timeout` | Comms | send rate changed | bounded | clamp to [TICK_MS, 1000ms] `:489-495` | clamp | Safe-Det | 2/1/2 | — | — |
| R06-3 | Source-bound socket lost when VPN drops → plaintext fall-through | Config/transport | — (prevented) | — | `sess_ensure_socket` holds session dark when VPN IP=0 `:572-579` | source-bind = sendto fails, not fails-open `:568` | Safe-Det | 4/2/2 | — | Keep source-binding invariant under test (2026-07-20 regression) |

### F-R-07 — Network transport (`dcs_net_liveness.c`, `dcs_net_*`)

| FM ID | Failure mode | Cause | Local effect | System effect (SF) | Existing detection | Existing mitigation | Class | S/O/D | Residual gap | Candidate requirement |
|---|---|---|---|---|---|---|---|---|---|---|
| R07-1 | lwIP stack wedged (scheduler alive) | Systematic SW | no packets flow | SF-1 STOP (machine timeout) + self-reboot recovery | gateway-ping + pstop cross-check `dcs_net_liveness.c:288-331` | `abort()`+reboot, not rollback-counted | Safe-Det | 4/2/3 | watchdog only arms after pstop bonded (`ps_last>0`) — an *never-bonded* wedge stays up but machine already STOP | Liveness is availability, not safety; keep machine timeout authoritative |
| R07-2 | Liveness watchdog false-aborts a healthy chip | Systematic (ICMP rate-limit) | spurious reboot → brief STOP | Availability loss, **fail-safe** | dual arm (gateway AND pstop) `:311-319` | reboot not counted to rollback | Safe-Det | 2/2/2 | — | — |

### F-R-08 — Status indication (`dcs_pstop_ring.c`) — non-safety, must be non-interfering

| FM ID | Failure mode | Cause | Local effect | System effect (SF) | Existing detection | Existing mitigation | Class | S/O/D | Residual gap | Candidate requirement |
|---|---|---|---|---|---|---|---|---|---|---|
| R08-1 | Ring shows OK while device commands STOP | Systematic SW (display logic) | operator misled | no actuation effect (display only) | operator cross-check | reads published telemetry only, no verdict path write | Safe-Undet | 3/2/4 | a misleading indicator can defeat operator supervision (a PL e assumption) | Prove ring cannot read/write the verdict path; treat indicator integrity per HARA supervision credit |
| R08-2 | Locate/OTA spinner masks safety colour indefinitely | Systematic | STOP/OK colour hidden | display only | auto-expire `DCS_RING_LOCATE_TIMEOUT_MS` `:518` | bounded mask | Safe-Det | 2/2/2 | — | — |

### F-R-09 — Boot / NVS / rollback (`dcs_safety.c`)

| FM ID | Failure mode | Cause | Local effect | System effect (SF) | Existing detection | Existing mitigation | Class | S/O/D | Residual gap | Candidate requirement |
|---|---|---|---|---|---|---|---|---|---|---|
| R09-1 | NVS peer-slot config corrupted | Flash wear / bit-rot | wrong/absent machine target | session dark → machine STOP; or points at wrong machine | none on peer-slot integrity | fail-safe (no session = STOP) | Dang-Det | 3/2/3 | a *valid-but-wrong* machine_id bonds a machine that should not be armed by this remote | CRC/version the NVS peer slots; reject on mismatch → dark |
| R09-2 | Rapid-boot rollback bricks both partitions | Systematic | device down | STOP (no sends) | boot-count ladder `:72`; both-invalid guard `:88` | refuse-to-brick `:88-101` | Safe-Det | 3/1/2 | — | — |
| R09-3 | Boot loop from genuine code fault masked as "network" abort | Systematic (mis-tagged reset) | rollback ladder not advanced | delayed rollback of a bad image | `was_liveness_abort` consumed once `:45-46` | flag single-shot | Dang-Det | 3/1/3 | a code fault that also happens to abort via liveness path could dodge the counter once | Ensure only the liveness path can set the magic; audit all `abort()` sites |

### F-H-01..05 — Host machine (`host/machine_app_runner.c`)

| FM ID | Failure mode | Cause | Local effect | System effect (SF) | Existing detection | Existing mitigation | Class | S/O/D | Residual gap | Candidate requirement |
|---|---|---|---|---|---|---|---|---|---|---|
| H01-1 | Non-BOND from unknown/timed-out remote dereferenced in upstream | Systematic SW (upstream bug) | crash | process death → all remotes STOP | wrapper filters `:752-757` before library | drop non-BOND from unknown | Safe-Det | 4/1/2 | — | Interface req F-P-02: keep the pre-dispatch guard |
| H02-1 | `min_stop_ms` set to 0 in `machine.toml` | **Config (no validation)** | arming delay disabled → a STOP→OK transient arms | **SF-3 breach** | none — `cfg_load` uses raw `parse_uint`, **no floor** `:240-241` | operator discipline only | **Dang-Undet** | 5/2/4 | host runner has **no safety-floor validation** (ROS2 node does: `kMinStopFloorMs=100`) | Enforce a compiled min-stop floor in the host runner; reject config below it |
| H02-2 | `max_missed_heartbeats` set very high, or `default_heartbeat_ms` huge | **Config (no validation)** | heartbeat timeout → minutes | **SF-1 latency breach** (silence not caught in time) | none — unbounded `parse_uint` `:228,236` | pstop_c constants exist but not enforced here | **Dang-Undet** | 5/2/4 | no clamp to `PSTOP_MIN/MAX_HEARTBEAT`; comment `:721` even mis-states timeout as `hb×(max_missed+1)` (actual is `hb×max_missed`, machine.c:298) | Validate timing against a compiled envelope (mirror ROS2 `validate_timing`); correct the stale comment |
| H02-3 | Early-OK propagated before policy commit | Systematic SW | spurious arm | (blocked) | raw `status_cb(OK)` held; committed only via `robot_status_commit_ok` after STOPPED→OK `:365-379,834-844` | latch + library native `delay_between_stop_ms` (#59) | Safe-Det | 4/1/2 | double-guard (wrapper + library) | Keep both guards; test the latch |
| H02-4 | `remote_details_cb` returns `stop_only=false` for a **non-operator** (a remote absent from the machine's operator allowlist) | **Config: operator allowlist mis-set** / shell default wrong (same obligation on host `machine.toml`, ROS param, and `machn` `/api/operators`) | that remote is admitted as an **operator** → it may perform the STOP→OK re-arm | **SF-3 breach: unauthorized re-arm** (a stop-only/non-operator remote resumes the machine — HARA H-13) | no online diagnostic — the certified gate trusts the shell-supplied `is_stop_only` (`machine.c:16,31`); caught only by commissioning **config review** + the upstream stop-only gate tests (`machine_test.c:677,1165`) | **default `stop_only=true` + empty operator allowlist** (out of the box every remote is stop-only); certified gate `machine.c:135-142` refuses STOP→OK ownership when `is_stop_only==true`; the same remote's STOP still stops (`:131,155-156`) and it is heartbeat-monitored (`:266-320`) | Dang-Det | 5/1/3 | a *valid-but-wrong* allowlist entry (a genuine non-operator deliberately listed) is not machine-detectable — rests on config discipline | **SR-SYS-09**; commissioning review of the operator allowlist; per-machine operator-list config-integrity check |
| H03-1 | `machine_now_ms` (CLOCK_MONOTONIC) frozen | Silicon/kernel | `check_heartbeats` sees `diff=0` → never times out | **SF-1 breach: silence undetected** | none (`now<=last_timestamp` path just skips, machine.c:282) | monotonic clock assumed sound | **Dang-Undet** | 5/1/5 | F-P-03 obligation: "supply a correct `get_time_cb`; never freeze it" — no independent clock check | Independent clock sanity (compare two time sources) or external watchdog on the machine host |
| H05-1 | Many-to-one OR aggregation fails to STOP on one remote's STOP | Systematic SW (library) | one remote's STOP lost | SF-1 breach | `[PQ]` pstop_c `handle_stop_msg`/`machine_stop_robot` (machine.c:127-156) | fail-safe OR is a library property | Dang-Det | 5/1/3 | out of scope internally (pre-qualified); interface only | F-P interface: verify OR semantics at integration (multi-remote E2E) |
| H04-1 | Announce thread config injects into safety path | Config/comms | — | none (separate thread, log-only) | isolated `:484-601` | outside safety loop | Safe-Undet | 1/1/2 | — | Confirm non-interference in review |

### F-M-01..08 — ROS2 machine (`ros2/protective_stop_machine/`)

| FM ID | Failure mode | Cause | Local effect | System effect (SF) | Existing detection | Existing mitigation | Class | S/O/D | Residual gap | Candidate requirement |
|---|---|---|---|---|---|---|---|---|---|---|
| M01-1 | Error transition leaks live publish timer | Systematic SW | stale publishes after teardown | misleading state, no actuation | `on_error` cancels+resets timer `:220`; backend stopped | explicit teardown, one UNSTABLE emitted | Safe-Det | 3/1/2 | — | — |
| M02-1 | `build_backend` selects wrong backend | Config | wrong observation source | fail on unknown → configure FAILURE `:121` | validated string | default software | Safe-Det | 2/1/2 | — | — |
| M03-1 | Software backend `cb_status` no-op + state from `robot_state` | Design | relies entirely on library min-stop native enforcement | if `delay_between_stop_ms`=0 → transient arms | ROS `validate_timing` floor 100ms `:84` blocks 0 | node-level floor | Safe-Det | 4/1/3 | software backend has **no physical relay** (`relay.applicable=false` `:199`) — actuation is a downstream ROS consumer's duty | State the actuation boundary explicitly; downstream consumer must treat STOP/UNSTABLE as de-energize |
| M03-2 | Second concurrent software backend clobbers `g_impl` | Systematic SW | callback context corruption | refused: `start()` returns false `:103` | file-scope guard | single-instance refusal | Safe-Det | 3/1/2 | — | — |
| M04-1 | `/state.json` unreachable / HTTP fail | Comms | no fresh state | `reachable=false` → UNSTABLE; diag ERROR "ROS blind" `:286` | curl status check `:74`; parse check | ESP32 `machn` enforces STOP **independently** | Safe-Det | 4/2/2 | ROS is **observation-only** on hardware backend; safety rests on `machn` | Keep ESP32 relay enforcement authoritative; ROS must never be in the actuation loop |
| M04-2 | `relay_stop` key missing/garbled in JSON | Comms/parse | — | defaults **true** (STOP) `hardware_backend.cpp:119` | `bool_at("relay_stop", true)` fail-safe default | safe default | Safe-Det | 3/2/2 | — | — |
| M05-1 | Deeply-nested hostile JSON overflows stack | Comms (compromised device) | crash | node down → observation lost (machn still enforces) | depth cap 32 `json_lite.hpp:87,97` | bounded recursion | Safe-Det | 3/1/2 | — | — |
| M05-2 | Tolerant number/string parse mis-reads a field | Systematic SW | wrong numeric telemetry | display/diag only (relay_stop is bool, defaults safe) | — | safe defaults on the safety-relevant bool | Safe-Undet | 2/2/3 | number parser accepts `1.2.3`-style junk `:185-198` | Constrain parser or validate the specific fields consumed |
| M07-1 | Runtime timing param loosens envelope | Config | longer timeout / shorter min-stop | (blocked) | `validate_timing` on set-param and service `:305-342` | compiled floor envelope `:28-35` | Safe-Det | 4/1/2 | envelope only on ROS2 path, **not** host runner (see H02) | Unify the validation envelope across host + ROS |
| M06-1 | Publish tick reads torn snapshot | Race | inconsistent published fields | display/diag only | mutex-guarded `snapshot()` `software_backend.cpp:235` | lock | Safe-Undet | 2/1/3 | — | — |
| M08-1 | Diagnostics under-reports a relay feedback fault | Systematic SW | fault not surfaced to ROS | observation gap (machn still STOPs) | `diagnostics()` maps `fault_a/b` `:289` | reads snapshot | Safe-Undet | 3/1/3 | ROS diag is advisory | — |

### F-P-01..04 — Pre-qualified `pstop_c` interface obligations (referenced, not re-verified)

| FM ID | Interface obligation | If the app shell violates it | System effect (SF) | Where the shell must hold the line | Class if violated | Candidate requirement |
|---|---|---|---|---|---|---|
| P01-1 | 40-byte layout + CRC-16 over bytes **[0..37]** (`pstop_msg.c:238,254`; size=40 `config.h:12`) | shell alters layout/CRC span → machine mis-validates | SF-2/SF-1 | remote encode `main.c:422`, decode `:745`; never repack | **Dang-Undet** | Freeze wire layout; golden-vector test both ends |
| P02-1 | Machine dispatch **default→STOP** (`machine.c:196-201`) | shell adds a path that answers OK on unknown | SF-2 | shell only forwards to `machine_process_message` | **Dang-Undet** | Assert every non-OK/BOND/UNBOND ⇒ STOP at integration |
| P03-1 | Correct, never-frozen `get_time_cb` on machine monotonic clock (`machine.c:218,269`) | frozen/backward clock → heartbeat check disabled | SF-1 (see H03-1) | `machine_now_ms`/`now_ms` on CLOCK_MONOTONIC | **Dang-Undet** | Independent clock cross-check |
| P04-1 | Counter/stamp echo + MSG_LOST/OUT_OF_ORDER (`machine.c` seq checks) | shell spoofs/bypasses echo | SF-2 (replay-to-run) | remote must not fabricate echo `main.c:767,779` | Dang-Det | Never synthesize `received_counter`; integration replay test |

---

## 3. Dangerous-Undetected (DU) register

These rows dominate the SIL 3 case. DU = the fault moves away from the safe
state (toward spurious RUN, or defeats a stop) **and** no on-line diagnostic
forces the safe reaction.

| DU ID | Function | Failure mode | Why undetected | Does Option A/B compensate? | S/O/D | Priority |
|---|---|---|---|---|---|---|
| **DU-1** | F-R-01 (R01-3) | Lost pull-down → open loop reads as closed → **false OK** | GPIO config set once at boot, never re-verified; the "config-integrity check" in design §6/§10#8 **is not implemented** | **No.** Option A trusts the live read; if the *sensor path* lies, the fresh sample is a false positive. Option B does not read the pad config. | 5/2/5 | **Highest** |
| **DU-2** | F-H-03 / F-P-03 (H03-1) | Machine `get_time_cb` frozen → remote silence never times out | `check_heartbeats` skips when `now<=last_timestamp` (machine.c:282); no independent clock check | No — clock is upstream of the verdict entirely | 5/1/5 | **Highest** |
| **DU-3** | F-R-02 (R02-3) | Compiler collapses the two diverse verdict expressions to identical code → common codegen fault agrees | lockstep memcmp passes on identical (wrong) output | **Partially self-defeating.** Option B's diversity is source-level only; single toolchain/ISA. The compensation for `OK==0` polarity leans on this diversity that the compiler may erase. | 4/2/4 | High |
| **DU-4** | F-H-02 (H02-1/2) | `machine.toml` sets `min_stop_ms=0` or huge heartbeat/`max_missed` | host runner performs **no** safety-floor validation (unlike ROS2 node) | N/A — config layer, upstream of A/B | 5/2/4 | High |
| **DU-5** | F-R-05 / F-P-01 (R05-1) | Systematic `pstop_message_encode` fault → identical wrong bytes on both cores | Option B diversifies the verdict *selection* but **not** the encode; memcmp blind to identical faults | No — encode is shared, pre-qualified but common-mode | 4/1/4 | High |
| **DU-6** | F-R-01 (R01-4) | Both pull-downs lost together (brownout/ESD/shared mux bug) → both cores false-OK and agree | common-cause defeats the cross-check (β≈1 on shared config path) | No | 5/1/5 | High |
| **DU-7** | F-R-04 (R04-5) | `memcmp` mis-returns equal for non-equal buffers | trust in libc; no positive-diff self-test | No — comparator is the diagnostic itself | 5/1/4 | Medium-High |
| **DU-8** | F-R-01 (R01-5/6) | Settle-time too short / stale GPIO read → this-tick sample not truly fresh | Option A assumes the read reflects *this* drive | Undermines Option A's premise if HW timing is off | 4/2/4 | Medium |
| **DU-9** | F-M-05 (M05-2) | Tolerant JSON parser mis-reads a consumed field | no schema validation | N/A (observation path; `relay_stop` bool defaults safe, limits blast radius) | 2/2/3 | Low |

**On the inherited `OK==0x00` fail-danger polarity:** an all-zeros artefact
decodes toward OK. The compensating measure (SYSTEM_DEFINITION §5) is the
Option A live-sample derivation: OK is only emitted when *this tick's* physical
both-phase read matches the driven level (`main.c:281-308`). That fully
addresses the **generation-side** danger — a zeroed flag, a latched state, or a
skipped branch cannot manufacture OK, and an all-zeros packet on the wire is
rejected by CRC at the machine. It does **not** address (a) a **sensor path
that physically lies** (DU-1/DU-6 — the live read is a false positive), (b)
**common-mode faults downstream of verdict selection** (encode DU-5, comparator
DU-7, the non-diverse `estop_primed` gate R03-1), or (c) **erosion of the
Option B diversity by the toolchain** (DU-3). Net: Option A/B compensate the
polarity for the fault classes they were scoped to (stuck/latched/interpretation
of the reads) and **do not** constitute a general defence against the fail-
danger polarity. A far-Hamming OK/STOP codeword upstream in `pstop_c` would
retire the polarity concern structurally; the code is written to survive that
change (the physical image is only a table index, `main.c:288`).

---

## 4. Common-cause / systematic analysis

**Does Option B actually break dual-core common-cause?** **Partially, and only
for one narrow fault class.** Option B diversifies *the expression that selects
the OK/STOP codeword from the two reads* (core0 arithmetic-image
`k_estop_msg[((rb_hi^1)|rb_lo)&1]` vs core1 boolean `(rb_hi==1)&&(rb_lo==0)`,
`main.c:304-308`). That genuinely catches a systematic misinterpretation of the
read variables in one expression. **It does not** provide:

- **Compiler/ISA diversity.** Both expressions are compiled by one toolchain
  for two identical Xtensa cores. The comment itself concedes they are
  "logically identical when correct." A modern optimizer may emit identical
  machine code → diversity erased (**DU-3**). This is the weakest link in the
  Option B claim and should be either build-gated (disassemble-and-diff) or
  made true structural diversity (separate translation units / flags, ideally a
  second implementation).
- **Diversity across the shared surface.** Everything after codeword selection
  is byte-identical code on both cores: the STOP-only override/debounce block
  (`:317-347`), `pstop_message_encode` (`:422`), the per-slot field assembly
  (`:409-423`). β≈1 there. The override block is STOP-biased (safe direction),
  but **encode is not** (DU-5).
- **Non-diverse singletons.** `estop_primed()` (`:359`), the comparator/memcmp
  (`:890`), the boot-priming send gate (`:903`), and the machine-side clock
  (H03-1) are single instances with no redundancy at all.

**Where β≈1 (identical code path, no diversity):** the override/debounce block;
message encode; comparator memcmp; `estop_primed`; the entire machine-side
(host/ROS) processing (single-threaded, single copy) — machine redundancy is
achieved by the *series relay chain on the `machn` device*, not by the app.

**Specific common-cause vectors requested:**

- **Boot transient.** The first loop samples glitch open on this board; the
  boot-priming hold (`estop_primed`, `LOOP_BOOT_OPEN_CONFIRM_TICKS`) suppresses
  the artefact STOP→OK so it cannot auto-arm (SF-3). Correct — **but the gate is
  a single non-diverse function** (R03-1); SF-3 also rests on machine-side
  `min_stop_ms`. Two independent barriers, one of which (host runner) can be
  config-disabled (**DU-4**).
- **NVS corruption.** Peer slots are not integrity-checked (R09-1). Fail-safe if
  a slot goes absent (session dark → STOP), but a *valid-but-wrong* `machine_id`
  is a silent mis-target. Boot-count NVS is guarded against bricking (`:88`).
- **Clock / `get_time_cb`.** Machine monotonic clock is the sole liveness time
  base by deliberate design (never anchored to remote stamp — correct, else a
  silent remote would freeze the watchdog). A frozen local clock is **DU-2**
  with no cross-check.
- **Relay weld (ESP32 `machn`, hardware-backend actuator).**
  **[DESCOPED 2026-08 — relay feedback OFF by default, pending new machine
  hardware; REVERSIBLE via `CONFIG_MACHN_RELAY_FEEDBACK`. See
  docs/RELAY_FEEDBACK_DESCOPE.md.** The series two-relay de-energize-to-safe stop
  (HFT=1, either contact opens the chain) is UNCHANGED and still safe on demand.
  What is descoped is the resistor-divider **feedback/diagnosis** below: a single
  weld is now **latent (dangerous-undetected)** until the next stop demand, and
  the ~1 s stop-on-contradiction no longer fires. FMEDA MC-2 DC credit is stale
  pending recompute (§FMEDA). Compensating measure: periodic commanded-open
  proof test (see the double-weld recommendation at the end of this bullet).]**
  Two relay contacts
  are **wired in series** in the stop circuit and each core drives its own coil
  from its own verdict (`machn/main/main.c:19-25,65-68`). A single welded
  contact still lets the other open the series chain → load de-energizes (safe).
  A resistor-divider feedback per contact detects commanded-vs-observed
  contradiction and, after `RELAY_FAULT_STOP_TICKS` (~1 s), forces STOPPED. The
  residual DU is a **double weld** (both contacts welded closed) — the series
  chain can no longer open; feedback would show a contradiction only if a core
  *commands* open, so a double weld while commanded-closed is latent until the
  next stop demand. Recommend periodic proof-test (commanded open cycle) to
  reveal a latent double weld — this is the classic PL e proof-test-interval
  requirement.
- **Network black-channel spoof/replay.** Confidentiality/integrity are provided
  by WireGuard/Tailscale (out of scope), plus app-layer CRC-16, per-session src
  IP/port filtering (`:741`), and `pstop_c` counter/stamp echo + MSG_LOST/
  OUT_OF_ORDER (F-P-04). All identified spoof/replay effects push the system
  toward STOP (fail-safe), so this is Safe-Undet, contingent on the black-
  channel assumption being stated as a safety requirement.

**Systematic-capability note (IEC 61508-3):** the dominant systematic risk is
**shared source code across the two cores** (single-version programming with
cosmetic expression diversity), compiled by a single toolchain. For a SIL 3
claim this needs either genuine N-version diversity on the safety decision or a
strong argument (MC/DC + toolchain qualification + object-code diff) that the
residual systematic capability is adequate. The current Option B, as written,
is **not** by itself sufficient evidence of common-cause independence.

---

## 5. Top findings (ranked)

1. **DU-1 — Lost GPIO pull-down reads as a closed loop → false OK, no runtime
   detection (F-R-01).** The single most dangerous gap: the pad configuration is
   set once at `estop_init` and never re-verified, yet `PSTOP_SAFETY_DESIGN.md`
   §6 and §10 item 8 both claim a runtime "config-integrity check" that **does
   not exist in the code**. Add a periodic config-register read-back plus a
   drive-low-must-read-0 self-test on both channels.
2. **DU-2 / F-P-03 — Frozen machine clock disables the entire heartbeat
   watchdog (F-H-03).** The whole liveness case rests on one monotonic clock
   with no independent cross-check; `check_heartbeats` silently skips when time
   does not advance. Add a second time source or an external hardware watchdog
   on the machine host.
3. **DU-4 — Host runner accepts unsafe timing config (F-H-02).** `machine.toml`
   can set `min_stop_ms=0` (defeats SF-3 arming) or an arbitrarily long
   heartbeat/`max_missed` (defeats SF-1 latency), because the host runner does
   **no** safety-floor validation — while the ROS2 node does. Port the ROS2
   `validate_timing` envelope into the host runner. (Also: the timeout comment
   at `machine_app_runner.c:721` mis-states the formula as `hb×(max_missed+1)`;
   the library uses `hb×max_missed`.)
4. **DU-3 — Option B diversity is erasable by the compiler (F-R-02).** The two
   "diverse" verdict expressions are logically identical and share one toolchain
   and ISA; an optimizer may collapse them, and the shared code surface after
   codeword selection is β≈1. Option B, as written, is not sufficient
   common-cause evidence for SIL 3. Build-gate an object-code diff or introduce
   real structural diversity.
5. **DU-5 / DU-7 — Lockstep is blind to common-mode faults in its own shared
   code (F-R-04/05).** A systematic `pstop_message_encode` bug (DU-5) or a
   `memcmp` that mis-returns equal (DU-7) produces identical wrong output on
   both cores and passes the comparator. Add golden-vector encode tests and a
   comparator positive-diff self-test (inject a 1-byte diff at every offset).

**Latent-fault proof-test flag (ISO 13849 / PL e):** the **double relay weld**
on the `machn` actuator (§4) is a latent dangerous fault revealed only by a
commanded-open cycle — specify a proof-test interval.

**Doc-vs-code inconsistencies found (trust the code):**
- §6/§10#8 assert a runtime GPIO "config-integrity check" — **not implemented**
  (root of DU-1).
- §6 attributes replay/stale-packet detection to a "stamp signature" — that was
  **Option C, dropped**; actual protection is `pstop_c` counter/stamp echo +
  MSG_LOST (F-P-04) over a CRC/black channel.
- §6/§10 "challenge" language reflects the dropped unpredictable-challenge
  design; the code uses deterministic both-phase sampling (Option A), which is
  reproducible by a same-firmware fault — recorded as a residual, not a control.
- `machine_app_runner.c:721` heartbeat-timeout comment (`hb×(max_missed+1)`)
  contradicts the library math and SYSTEM_DEFINITION (`hb×max_missed`).

---

*End of first draft. Every **Dang-Undet** row and the top-5 findings require
safety-lead disposition before this feeds the requirements set and the FMEDA.*
