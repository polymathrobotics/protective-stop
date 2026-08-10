<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Protective-Stop — Safety Requirements Specification (first draft)

**Status:** Engineering first draft for **safety-lead curation**. Derived
top-down from the safety goals **SG-1..SG-6** (`docs/safety/HARA.md` §5) and
bottom-up from the **Dangerous-Undetected register DU-1..DU-9**
(`docs/safety/FMEA.md` §3), allocated to the function IDs of the authoritative
backbone (`docs/safety/SYSTEM_DEFINITION.md` §4). Numbers, formulas, and
Option-A/B/C status follow `docs/safety/RECONCILIATION.md` (**code wins**):
Option A + Option B **live**, Option C **dropped**, GPIO config-integrity
re-verification **not implemented**, heartbeat timeout = `heartbeat_ms ×
max_missed` (400 × 5 ≈ 2.0 s; `max_missed` raised 3 → 5 on 2026-08-04).

**Standard frame:** IEC 61508 (SIL / systematic capability) + ISO 13849 (PL for
the actuation side). Allocated target: **SIL 3 / PL e** for the on-demand stop —
**allocated, not achieved** (HARA §6: FMEDA, Option-B fault-injection, and
SC 3 process evidence still gate the numeric claim). Safe state = **STOP =
de-energized**. Inherited fail-danger polarity: `OK == 0x00`, `STOP == 0x01`.

---

## 1. Numbering, method, and conventions

Requirement IDs are **`SR-<area>-<nn>`**:

| Area | Scope | Allocation base |
|---|---|---|
| **SR-SYS** | System / safety-goal level | SG-1..SG-6 |
| **SR-R** | Remote firmware (`firmware/`) | F-R-xx |
| **SR-H** | Host machine (`host/machine_app_runner.c`) | F-H-xx |
| **SR-M** | ROS 2 machine (`ros2/protective_stop_machine/`) | F-M-xx |
| **SR-I** | `pstop_c` interface / integration boundary | F-P-xx |

These IDs are **distinct** from the pre-qualified `pstop_c` requirement set
(`req_2_xx` / `req_3_xx` under `pstop_c/.../requirements/`), which is referenced,
not re-derived. Where an SR is verified by a test, that test is authored in the
**same style** as the `pstop_c` suite — one requirement per test, a header
comment `// SR-<area>-<nn>-<k>: <shall-statement>` + a Description, one behaviour
asserted (see `pstop_c/.../requirements/req_3_04_test.c` for the template).

**Each requirement carries:** ID · shall-statement (singular, verifiable) ·
**Derived from** (SG / H / DU — traceability up) · **Allocated to** (F-xx —
traceability down) · **Integrity** (SIL 3 / PL e, or a decomposed level with
rationale) · **Verification** (Test / Fault-injection / Analysis / Inspection) ·
**Status**:

- **Satisfied** — implemented; cited `file:line` (line numbers per the analyzed
  tree, ±a few lines as the tree evolves).
- **Gap** — no implementation yet.
- **Residual-accepted** — deliberately not implemented; rationale given.

**Integrity-decomposition note.** Several SRs are marked *decomposed*. Per
IEC 61508-2 the SIL 3 stop function is realized by an architecture whose
redundant/diverse legs may each carry a lower systematic-capability burden. Two
independent-and-diverse legs that both fail-safe (e.g. remote fail-safe silence
**and** machine heartbeat timeout, or remote priming hold **and** machine
min-stop veto) support a **SIL 3(dec) = SIL 2 + SIL 2** decomposition **only if
independence is demonstrated**. Independence is currently **argued, not
quantified** (no FMEDA, β unquantified — HARA §6); every decomposition claim
below is therefore conditional and tagged as such.

---

## 2. System-level requirements (SR-SYS — from the safety goals)

| ID | Requirement (shall) | Derived from | Allocated to | Integrity | Verify | Status |
|---|---|---|---|---|---|---|
| **SR-SYS-01** | On any stop demand **or** any detected fault, the pstop system shall drive the machine to de-energized STOP such that the end-to-end reaction time (sense → verdict → transmit → machine detection → final-element de-energize) does not exceed the vehicle Process Safety Time; the worst-case internal budget shall be **≤ 2.1 s** (heartbeat `400 ms × max_missed 5` = 2.0 s + one 100 ms tick, HARA A-05). `max_missed` is a tunable, raised 3 → 5 on 2026-08-04 (was ≤ 1.3 s at `max_missed 3`); the PST budget must accommodate the ~2.0 s detection time. | SG-1, H-01/H-03/H-09/H-11 | F-R-01..05, F-H-03, F-M-03/04, final element | SIL 3 / PL e | Analysis (timing budget) + Test (injected demand → STOP latency) | **Gap** (budget asserted; PST is integrator input A-05; measured latency not yet evidenced) |
| **SR-SYS-02** | The machine shall hold OK **only** while a sustained OK is freshly and causally derived from a correct live both-channel read on the remote **and** a valid black-channel round-trip; any staleness, replay, or forgery shall resolve to STOP within the heartbeat timeout (≤ 2.0 s at `max_missed` 5, since 2026-08-04). | SG-2, H-05/H-06/H-11 | F-R-02/06, F-H-03, F-P-03/04 | SIL 3 / PL e | Test + Fault-injection (stale/replayed/forged OK) | **Gap** at system level (constituent SRs partly Satisfied; end-to-end staleness test outstanding) |
| **SR-SYS-03** | The machine shall transition STOP→OK **only** via a deliberate arming gesture (STOP held ≥ `min_stop_ms` = 500 ms, then released) — never spontaneously, on a transient, or at boot. | SG-3, H-04/H-08 | F-R-03, F-H-02, F-M-03 | SIL 3 / PL e | Test (transient/boot-glitch never arms) | **Partially satisfied** (machine min-stop veto live; boot hold live; host floor is a Gap — see SR-H-03) |
| **SR-SYS-04** | In many-remotes→one-machine, **any** bonded remote's STOP or silence shall force STOP regardless of other remotes (fail-safe OR); arming shall require a single owning gesture, and a lost/ghost bond shall not read as present-and-OK. | SG-4, H-07 | F-H-05, F-P-04 | SIL 3 / PL e | Test (multi-remote E2E; 1 STOP masks N OK) | **Partially satisfied** (`pstop_multi_remote_test.py` 32/32; ghost-bond random-fault masking unquantified) |
| **SR-SYS-05** | The final actuation element shall de-energize-to-safe with **HFT = 1** and diagnosable stuck-on (output feedback), **or** the integrator shall supply an element of ≥ the allocated integrity. Software/ROS 2 machine outputs are non-safety and shall not be the sole enforcement path. | SG-5, H-10/H-12 | F-M-03/04, `machn` relay, A-07 | SIL 3 / PL e (allocated to integrator for software machine) | Analysis (architecture) + Test (`machn` series-relay + feedback) | **Partially satisfied** (`machn` dual series relay + feedback live; software-machine final element is an allocated **Gap** to the integrator) |
| **SR-SYS-06** | The system shall bound the spurious-STOP (nuisance) rate so it introduces no secondary hazard, **without ever filtering, delaying, or debouncing the OPEN→STOP direction**; only the re-arm (reclose) direction may be gated. | SG-6, H-02 | F-R-03 | **No de-energize SIL** — availability goal, fail-safe by construction (asymmetric debounce); managed per A-06 | Test (STOP edge never debounced) | **Satisfied** (`firmware/main/main.c` reclose-only debounce; STOP is single-tick — see SR-R-05) |
| **SR-SYS-07** | The machine's default and power-on state shall be STOP; OK shall be the exceptional, continuously-justified state, reachable only through the arming path. | SG-1/SG-3, H-08 | F-P-02, F-H/M dispatch | SIL 3 / PL e | Inspection + Test (default→STOP) | **Satisfied** (`pstop_c` `machine.c` default→STOP; see SR-I-02) |
| **SR-SYS-08** | The transport shall be treated as an **untrusted black channel**; all safety detection (CRC-16, counter/stamp echo, heartbeat/liveness) shall reside in the endpoints and no timing/config value received over the channel shall widen the safe envelope. | SG-2, A-08, H-06/H-11 | F-R-06/07, F-P-01/04 | SIL 3 / PL e | Analysis (black-channel argument, FMEDA feed) | **Partially satisfied** (endpoint CRC/echo/clamp live — SR-R-13/14; residual bit-error/masquerade rate is an FMEDA **Gap**) |
| **SR-SYS-09** | STOP→OK (re-arm) shall be permitted **only** for remotes on the machine's **operator allowlist** (`is_stop_only == false`); every other accepted remote shall be **stop-only** — able to command STOP and heartbeat-monitored (its silence ⇒ fail-safe STOP), but unable to re-arm. The operator allowlist shall **default to empty**, so out of the box every bonded remote is stop-only (maximally safe) and operators are added only during configuration. | SG-3, **H-13**, FMEA H02-4 | F-P-02, F-H-05 (+ F-M / `machn` operator-list config) | SIL 3 / PL e | Test (stop-only remote's OK never re-arms; operator's does) + Inspection (default-empty allowlist) | **Satisfied** (`pstop_c` gates STOP→OK ownership on `is_stop_only==false` — `pstop_c/.../machine.c:135-142`; a stop-only remote's STOP still forces STOP `:131,155-156` and it is heartbeat-monitored `machine_check_heartbeats:266-320`; `remote_details_cb`→`is_stop_only` `:16,31`; upstream `machine_test.c:677,1165`) — residual: the newly-added shell operator-list config surface (machn `/api/operators`) wants its own plumbing test |

---

## 3. Remote-firmware requirements (SR-R — `firmware/main/main.c`)

### 3.1 SF-1 stop-on-demand + SF-2 no-spurious-OK sensing/verdict path

| ID | Requirement (shall) | Derived from | Allocated to | Integrity | Verify | Status |
|---|---|---|---|---|---|---|
| **SR-R-01** | Each core shall, **every tick**, drive its loop HIGH then read it back as 1 and drive it LOW then read it back as 0, and shall declare the channel closed-and-healthy only if both fresh reads match the driven level; it shall ignore the tick counter for phase selection. | SG-1/SG-2, H-01/H-11, DU-8 | F-R-01 | SIL 3 / PL e | Test (both-phase truth table) + Inspection | **Satisfied** (`estop_channel_closed`, both-phase drive/read; `(void)counter` — `firmware/main/main.c:265-290`) |
| **SR-R-02** | The OK codeword shall be selected **solely** by the arithmetic image of this tick's two fresh reads (index 0 = OK iff `rb_hi==1 && rb_lo==0`); no stale, latched, or default flag and no health/debounce branch shall be able to produce OK. | SG-2, H-11, DU-1 note, FMEA R02-1 | F-R-02 (Option A) | SIL 3 / PL e | Test + **MC/DC** (OK unreachable without fresh matched sample) | **Satisfied** (arithmetic-image select `firmware/main/main.c:304-306`; STOP-only overrides `:340-348`) |
| **SR-R-03** | The two cores shall form the verdict by **provably non-convergent** expressions (core 0 arithmetic-image vs core 1 boolean), and the build shall guarantee this diversity survives optimization — verified by an object-code diff gate **or** compile-diverse translation units. | SG-1, **H-09**, **DU-3** | F-R-02 (Option B) | SIL 3 / PL e (systematic; common-cause leg) | **Fault-injection** (identical logic fault → encodings still diverge) + object-code diff | **Satisfied** [Reconciled 2026-08-07] — source diversity `:305/:307` **and** the object-code diff gate `scripts/check_estop_diversity.sh` (CI-wired in `firmware-build.yml`; commit `d96129c`; DU-3 CLOSED — `docs/safety/DU3_OBJECT_CODE_DIVERSITY.md`). Residual: the identical-logic-fault-injection divergence test is still owed (HARA §7 item 5) |
| **SR-R-04** | The health, integrity, and debounce logic downstream of codeword selection shall be **monotonic toward STOP** — able only to raise a verdict to STOP, never to lower it to OK. | SG-1, FMEA R02-4 | F-R-02 | SIL 3 / PL e | Test + MC/DC (prove STOP-only override) | **Satisfied** (STOP-only override block `firmware/main/main.c:340-348`) |
| **SR-R-05** | The release/reclose debounce shall gate **only** the STOP→OK (reclose) direction; it shall never filter, delay, or suppress an OPEN→STOP edge, which shall propagate on the same tick it is sampled. | SG-6, H-02, FMEA R03-2 | F-R-03 | Availability (STOP path is SIL 3 / PL e, must stay unfiltered) | Test (STOP edge never debounced; reclose gated) | **Satisfied** (`closed_streak` reset on any unhealthy read; `LOOP_RECLOSE_DEBOUNCE_TICKS` guards reclose only — `firmware/main/main.c:323-347`) |
| **SR-R-06** | Until both cores have sampled both loop phases (boot warm-up), the remote shall transmit **nothing**, so the boot-priming STOP artifact cannot reach a machine and auto-complete its arming; the priming gate shall require agreement of both cores. | SG-3, **H-08**, FMEA R03-1 | F-R-03 | SIL 3 / PL e (SF-3; decomposed with SR-H-02 machine min-stop — **independence conditional**) | Test (boot glitch never arms) | **Satisfied** (`estop_primed` both-core latch `:359-363`; comparator send-hold `:903`) — residual: `estop_primed` is a non-diverse singleton (FMEA R03-1) |

### 3.2 Comparator, encode, black-channel (SF-1/SF-2)

| ID | Requirement (shall) | Derived from | Allocated to | Integrity | Verify | Status |
|---|---|---|---|---|---|---|
| **SR-R-07** | The comparator shall `memcmp` both cores' full `PSTOP_MESSAGE_SIZE` (40-byte) encodings per machine and shall transmit **only on exact agreement**; on any divergence, core-publish timeout, or un-primed state it shall send nothing, letting every machine fail-safe on its heartbeat timeout. | SG-1/SG-2, H-01/H-09, FMEA R01-2/R04-1 | F-R-04 | SIL 3 / PL e | Test + Fault-injection (1-byte diff at each offset → no send) | **Satisfied** (transmit-on-agreement `firmware/main/main.c:887-903`) |
| **SR-R-08** | The comparator shall be verified by a **positive-diff self-test**: an injected single-byte difference at every one of the 40 offsets shall be detected as a mismatch (guards against a `memcmp` that mis-returns equal). | **DU-7**, FMEA R04-5 | F-R-04 | SIL 3 / PL e | Fault-injection (per-offset diff) | **Gap** (relies on libc `memcmp`; no positive-diff test yet) |
| **SR-R-09** | The remote shall **periodically re-verify** each channel's GPIO pad configuration (direction, mux, pull-down) by register read-back **and** a drive-low-must-read-0 self-test, and shall force STOP on any mismatch — closing the lost-pull-down → false-OK path. | SG-1/SG-2, **H-01**, **DU-1**, RECONCILIATION R-02 | F-R-01 | SIL 3 / PL e | Fault-injection (corrupt pad config → STOP) | **Satisfied** [Reconciled 2026-08-07] — remote now periodically re-verifies each channel's pad config (`gpio_ll_get_io_config` read-back) + drive-low self-test and forces a controlled reset on mismatch (`gpio_cfg_fault`); commit `1cfbd9a`, DU-1 CLOSED (`docs/safety/CLOCK_GUARD_AND_GPIO_REVERIFY.md`, on-target fault-injection validated). Residual: on-bench corruption of a *live* pad config is bench-blocked (logic host-reasoned) |
| **SR-R-10** | The remote shall detect **common-cause loss of both channels' pull-downs** (brownout / shared pad-mux / ESD) — e.g. via a brownout detector wired to the safe state and/or diverse pull-down verification — and force STOP. | SG-1, **DU-6**, FMEA R01-4 | F-R-01 | SIL 3 / PL e (common-cause; β≈1 on shared config path) | Fault-injection + Analysis (FMEDA β) | **Gap** (no common-mode pull-down detection; brownout not wired to verdict) |
| **SR-R-11** | The loop settle time (`ESTOP_SETTLE_US`) shall be characterized against the maximum deployed cable/contact RC and set with margin, so that a fresh both-phase read always reflects this tick's drive. | SG-2, **DU-8**, FMEA R01-5/R01-6 | F-R-01 | SIL 3 / PL e | Analysis (RC vs settle) + Test (max-cable bench) | **Gap** (`ESTOP_SETTLE_US` is a bench constant; not validated vs deployed cable length) |
| **SR-R-12** | The message encode shall be protected against a systematic common-mode fault producing identical wrong bytes on both cores: a golden-vector encode test shall pin the 40-byte layout, and the message field shall be cross-checked by a second independent encoder image on at least one core. | SG-2, **DU-5**, FMEA R05-1 | F-R-05, F-P-01 | SIL 3 / PL e (common-cause; encode is shared/β≈1) | Test (golden vectors) + Fault-injection | **Gap** (encode is single shared image; Option B does not diversify it; golden-vector test not present in-shell) |
| **SR-R-13** | The remote shall **never fabricate or unconditionally adopt** the black-channel echo (`received_counter` / `received_stamp`); it shall relay only the machine-supplied echo, and a spoofed/replayed reply shall tend the system toward STOP, not OK. | SG-2, **H-06**, FMEA R06-1, F-P-04 | F-R-06 | SIL 3 / PL e | Test (replay reply → no OK) + Inspection | **Satisfied** (src IP/port filter `:741`, CRC check `:746`, echo relayed not synthesized `:767/:779`) |
| **SR-R-14** | The remote's send socket shall be **source-bound** such that loss of the VPN interface causes `sendto` to fail (session held dark) rather than fall through to a plaintext/unbound path; absence of a valid bound socket shall yield no OK on the wire. | SG-2, A-08, FMEA R06-3 | F-R-06/07 | SIL 3 / PL e | Test (VPN-drop regression) | **Satisfied** (`sess_ensure_socket` holds dark when VPN IP=0 `:572-579`; source-bind `:568`) |
| **SR-R-15** | The NVS peer-slot configuration shall be integrity-checked (CRC + version); a corrupt or version-mismatched slot shall be rejected (session dark → STOP) rather than used to target a machine. | SG-1, FMEA R09-1 | F-R-09 | SIL 2 (config-integrity; fail-safe-dark on reject — decomposed below the stop path) | Test (corrupt slot → dark) | **Gap** (peer slots not integrity-checked; absent slot already fails safe, valid-but-wrong slot mis-targets) |

---

## 4. Host-machine requirements (SR-H — `host/machine_app_runner.c`)

| ID | Requirement (shall) | Derived from | Allocated to | Integrity | Verify | Status |
|---|---|---|---|---|---|---|
| **SR-H-01** | The host runner shall monitor per-remote liveness on the machine's own `CLOCK_MONOTONIC` and command STOP when a remote's silence reaches `heartbeat_ms × max_missed_heartbeats` (**not** `× (max_missed+1)`), i.e. ≈ 2.0 s at defaults (`max_missed` = 5 since 2026-08-04; was ≈ 1.2 s at 3). | SG-1/SG-2, H-05, RECONCILIATION R-08 | F-H-03, F-P-03 | SIL 3 / PL e | Test (silence → STOP at boundary) | **Satisfied** (library `check_heartbeats` on `machine_now_ms`; timeout `hb×max_missed` — `pstop_c/.../machine.c:290-298`; host wires `get_time_cb` and the corrected comment at `machine_app_runner.c:721`) |
| **SR-H-02** | The host runner shall enforce the arming policy: an OK from the library shall be committed to the robot only **after** a STOPPED→OK transition that followed a STOP held ≥ `min_stop_ms`; short STOP→OK cycles shall be vetoed. | SG-3, H-04, FMEA H02-3 | F-H-02 | SIL 3 / PL e | Test (short cycle vetoed; latch) | **Satisfied** (`robot_status_commit_ok` latch `:375-379,834-844`; library native `delay_between_stop_ms` = `min_stop_ms` `:666`) |
| **SR-H-03** | The host runner shall validate all timing config (`min_stop_ms`, `default_heartbeat_ms`, `max_missed_heartbeats`) against a **compiled safety envelope** and refuse to start (or clamp to safe) any value outside it — at parity with the ROS 2 node's `validate_timing` floors (`min_stop_ms ≥ 100`, `heartbeat_ms ∈ [50,1000]`, `max_missed ∈ [1,5]`). | SG-1/SG-3, **H-01/H-04**, **DU-4** | F-H-02 | SIL 3 / PL e | Test (reject `min_stop_ms=0`, huge heartbeat) | **Satisfied** [Reconciled 2026-08-07] — host `cfg_validate` refuses unsafe timing at startup (`machine_app_runner.c` SR-H-03 block :656+; floors mirror the ROS 2 node's `validate_timing`: `min_stop_ms ≥ 100`, `heartbeat_ms ∈ [50,1000]`, `max_missed ∈ [1,5]`); test `tools/test_config_floor.py` 7/7; DU-4 CLOSED |
| **SR-H-04** | The host machine shall detect a **frozen/backward monotonic clock** (e.g. an independent second time source or an external hardware watchdog) so that a stalled `get_time_cb` cannot silently disable heartbeat detection; a non-advancing clock shall force STOP. | SG-2, **H-05**, **DU-2**, F-P-03 | F-H-03, F-P-03 | SIL 3 / PL e | Fault-injection (freeze clock → STOP) | **Satisfied** [Reconciled 2026-08-07] — machine-side clock-freeze guard (`clock_guard`, host + machn `get_time_cb`; SR-H-04b at `machine_app_runner.c:332`) forces STOP on a non-advancing clock; HIL-validated (`docs/safety/MACHN_CLOCK_GUARD_HIL.md`), DU-2 CLOSED. Residual: genuine esp_timer-freeze *detection* is fault-injection-proven on the remote analog + host unit tests (can't inject on unmodified silicon) |
| **SR-H-05** | In many-to-one, the host runner shall aggregate bonded remotes by **fail-safe OR** (any STOP or any silence forces STOP) and shall enforce single-owner arming; a dropped/ghost bond shall not be counted as present-and-OK. | SG-4, H-07 | F-H-05 | SIL 3 / PL e | Test (multi-remote E2E) | **Partially satisfied** (OR is a `pstop_c` property; validated 32/32; random-fault ghost-bond masking unquantified — SR-SYS-04) |
| **SR-H-06** | The host runner shall drop any non-BOND message from an unknown or timed-out remote **before** dispatching to the library, preventing an upstream null-deref and preserving default→STOP. | SG-1, FMEA H01-1, F-P-02 | F-H-01 | SIL 2 (defensive pre-guard; downstream still default→STOP) | Inspection + Test | **Satisfied** (pre-dispatch filter `machine_app_runner.c:752-757`) |

---

## 5. ROS 2-machine requirements (SR-M — `ros2/protective_stop_machine/`)

| ID | Requirement (shall) | Derived from | Allocated to | Integrity | Verify | Status |
|---|---|---|---|---|---|---|
| **SR-M-01** | The node shall validate every timing parameter (initial, `on_set_parameters`, and reconfig service) against the compiled floor envelope and **reject** any value that loosens the safe window; rejection shall not alter the running safe config. | SG-1/SG-3, H-04, FMEA M07-1/DU-4 | F-M-07 | SIL 3 / PL e | Test (reject loosening set-param) | **Satisfied** (`validate_timing` on config/set-param/service — `machine_bridge_node.cpp:72-97,323,354`; floors `:28-35`) |
| **SR-M-02** | On the hardware backend, a missing or malformed `relay_stop` field in `/state.json` shall default to **STOP** (true). | SG-1, FMEA M04-2 | F-M-04/05 | SIL 3 / PL e | Test (missing key → STOP) | **Satisfied** (`bool_at("relay_stop", true)` — `hardware_backend.cpp:119`) |
| **SR-M-03** | On the hardware backend, an unreachable or unparseable `/state.json` shall drive the observed state to UNSTABLE and raise a diagnostic ERROR; the ESP32 `machn` relay shall remain the authoritative enforcement, never the ROS node. | SG-1/SG-5, H-12, FMEA M04-1 | F-M-04, F-M-08 | SIL 3 / PL e (enforcement allocated to `machn`) | Test (drop `/state.json` → UNSTABLE) | **Satisfied** (`reachable=false`→UNSTABLE, diag ERROR — `hardware_backend.cpp` / `machine_bridge_node.cpp:286`) |
| **SR-M-04** | The software backend's STOP/UNSTABLE output shall be documented and treated as a **non-safety** signal; a software-machine deployment shall not be credited as a rated final element, and the actuation duty shall be explicitly allocated to a downstream consumer or to `machn`. | SG-5, **H-12**, FMEA M03-1, A-07 | F-M-03 | **Residual-accepted** (architectural; final-element integrity allocated to integrator) | Inspection (safety-case scope statement) | **Residual-accepted** — `relay.applicable=false` for software backend (`software_backend.cpp:199`); rationale: no in-scope actuator exists for this form factor (HARA H-12, A-07) |
| **SR-M-05** | The JSON parser shall bound recursion depth (≤ 32) so a hostile/compromised device cannot crash the node by deep nesting; a rejected parse shall fail safe (observation lost, `machn` still enforces). | SG-1, FMEA M05-1 | F-M-05 | SIL 2 (observation path; enforcement elsewhere) | Test (deep JSON → bounded reject) | **Satisfied** (depth cap 32 — `json_lite.hpp:87,97`) |
| **SR-M-06** | The tolerant JSON number/string parser shall not mis-read a **safety-consumed** field; safety-relevant fields shall be booleans with fail-safe defaults, and any relaxed numeric parse shall be confined to display/diagnostic fields. | **DU-9**, FMEA M05-2 | F-M-05 | **Residual-accepted** (blast radius limited: only `relay_stop` bool is safety-consumed and defaults STOP) | Inspection + Test (field-level) | **Residual-accepted** — parser accepts junk numerics (`json_lite.hpp:185-198`) but no safety field is numeric; rationale recorded, low DU |

---

## 6. `pstop_c` interface / integration requirements (SR-I — F-P-01..04)

These bound the **assumptions-of-use** the app shell must honor for the
pre-qualified SEooC (HARA A-09). `pstop_c` internals are **not** re-verified;
these are integration obligations only.

| ID | Requirement (shall) | Derived from | Allocated to | Integrity | Verify | Status |
|---|---|---|---|---|---|---|
| **SR-I-01** | The shell shall **not** alter the 40-byte message layout or the CRC-16 span over bytes [0..37]; a golden-vector test shall pin the encode/decode at both ends. | SG-2, H-11, FMEA P01-1, RECONCILIATION | F-P-01, F-R-05 | SIL 3 / PL e | Test (golden vectors both ends) | **Satisfied by contract** (encode `main.c:422`, decode `:745` use library size/span) — **golden-vector test is a Gap** (see SR-R-12) |
| **SR-I-02** | The shell shall preserve the library's **default→STOP** dispatch: every unknown, malformed, or non-OK/BOND/UNBOND message shall resolve to STOP; the shell shall add no path that answers OK on an unhandled input. | SG-1, H-01, FMEA P02-1 | F-P-02, F-H-01/F-M-02 | SIL 3 / PL e | Test (fuzz unknown → STOP) + Inspection | **Satisfied** (library default→STOP `machine.c:196-201`; shell only forwards — host `:752-757`, ROS `build_backend`) |
| **SR-I-03** | The shell shall supply a correct, monotonic, **never-freezing** `get_time_cb` bound to the machine's monotonic clock; it shall not anchor liveness time to any remote-supplied stamp. | SG-2, **H-05**, **DU-2**, FMEA P03-1 | F-P-03, F-H-03 | SIL 3 / PL e | Fault-injection (freeze/reverse clock) | **Partially satisfied** (host binds `CLOCK_MONOTONIC`; correct by construction) — **frozen-clock diagnostic is a Gap** (SR-H-04) |
| **SR-I-04** | The shell shall **not spoof or bypass** the counter/stamp echo or the `MSG_LOST`/`OUT_OF_ORDER` validation; it shall never synthesize `received_counter`/`received_stamp`, so anti-replay rests on native library monotonicity. | SG-2, **H-06**, FMEA P04-1 | F-P-04, F-R-06 | SIL 3 / PL e | Test (integration replay) + Inspection | **Satisfied by contract** (remote relays echo, does not fabricate `main.c:767,779`) — replay integration test is a Gap; anti-replay residual per HARA §7 item 6 |

---

## 7. DU → requirement discharge (bottom-up completeness check)

Every Dangerous-Undetected row generates ≥ 1 requirement (mitigation) or an
explicit accepted residual.

| DU | Failure | Discharged by | Disposition |
|---|---|---|---|
| **DU-1** | Lost pull-down → false OK | **SR-R-09** | Mitigation — **Gap** (highest priority) |
| **DU-2** | Frozen machine clock disables heartbeat | **SR-H-04**, SR-I-03 | Mitigation — **Gap** (highest priority) |
| **DU-3** | Compiler erases Option B diversity | **SR-R-03** | Mitigation (verification req) — **Gap** |
| **DU-4** | Host accepts unsafe timing config | **SR-H-03** | Mitigation — **Gap** (cheap/high value) |
| **DU-5** | Common-mode encode fault | **SR-R-12**, SR-I-01 | Mitigation — **Gap** |
| **DU-6** | Both pull-downs lost (common-cause) | **SR-R-10** | Mitigation — **Gap** |
| **DU-7** | `memcmp` mis-returns equal | **SR-R-08** | Mitigation — **Gap** |
| **DU-8** | Settle-time / stale read not fresh | **SR-R-11**, SR-R-01 | Mitigation — **Gap** (analysis) |
| **DU-9** | Tolerant JSON parser mis-reads a field | **SR-M-06** | **Residual-accepted** (bool safety field defaults safe) |

**Fail-danger `OK==0x00` polarity** (standing anchor, HARA H-11 / FMEA §3): the
generation-side danger is discharged by **SR-R-02** (Option A live-sample) +
**SR-I-01** (CRC rejects all-zeros on the wire). It does **not** cover a
physically-lying sensor path (SR-R-09/10), common-mode downstream faults
(SR-R-08/12), or toolchain erosion (SR-R-03). A far-Hamming OK/STOP codeword
upstream in `pstop_c` would retire it structurally — recorded as an upstream
recommendation, not an in-scope requirement.

**Latent double relay-weld** (`machn`, FMEA §5 / ISO 13849 PL e proof test):
covered by **SR-SYS-05** with a required **proof-test interval** (commanded-open
cycle) — the interval itself is an integrator/commissioning parameter, flagged
here as an open value.

---

## 8. Traceability preview (SG → SR) and test-gap map

**Safety-goal discharge.** Full = every derived SR Satisfied or accepted;
Partial = one or more derived SRs are Gaps.

| SG | Discharged by (SR) | Discharge state |
|---|---|---|
| **SG-1** (stop on demand/fault ≤ FTTI) | SR-SYS-01/07, SR-R-01/04/05/07/09/10/11, SR-H-01/03/06, SR-M-01/02/03/05, SR-I-01/02 | **Partial** (Gaps: SR-R-09/10/11, SR-H-03) |
| **SG-2** (no spurious OK; freshness) | SR-SYS-02/08, SR-R-02/08/12/13/14, SR-H-04, SR-I-01/03/04 | **Partial** (Gaps: SR-R-08/12, SR-H-04) |
| **SG-3** (arming integrity + operator authorization) | SR-SYS-03/09, SR-R-06, SR-H-02/03, SR-M-01 | **Partial** (Gap: SR-H-03 host floor; SR-SYS-09 authorization Satisfied) |
| **SG-4** (many→one fail-safe OR) | SR-SYS-04, SR-H-05, SR-I-04 | **Partial** (ghost-bond masking unquantified) |
| **SG-5** (final element HFT=1 / integrator) | SR-SYS-05, SR-M-03/04 | **Partial** (software-machine final element allocated to integrator) |
| **SG-6** (bounded spurious STOP, fail-safe debounce) | SR-SYS-06, SR-R-05 | **Full** (Satisfied by construction) |

**Common-cause SG-1/SG-2 systematic leg** (H-09) is carried by SR-R-03 and
remains the dominant unproven item (no FMEDA, Option-B fault-injection pending).

**SRs still needing tests (input to the later traceability/coverage task).**
Distinct from implementation Gaps: some Satisfied SRs still lack a
requirement-traced test.

| Needs a `SR-*-test.c` (style of `req_3_04_test.c`) | Why |
|---|---|
| SR-R-01, SR-R-02, SR-R-04 | Live logic Satisfied; MC/DC + truth-table tests not yet authored |
| SR-R-05, SR-R-06 | STOP-never-debounced / boot-never-arms invariants untested in-tree |
| SR-R-07, **SR-R-08** | Comparator agreement live; per-offset positive-diff fault-injection missing |
| SR-H-01, SR-H-02, SR-H-06 | Host coverage 70.4% line / 56.8% branch — logic-unit harness needed |
| SR-M-02, SR-M-03 | ROS 2 coverage task not started |
| SR-I-01, SR-I-04 | Golden-vector + replay integration tests to author |
| **All Gap SRs** (SR-R-03/09/10/11/12, SR-H-03/04, SR-R-15) | Test follows implementation |

---

## 9. Summary of status

_Status as of 2026-08-07 (reconciled with OPEN_ITEMS: DU-1/2/3/4 closures —
SR-R-03, SR-R-09, SR-H-03, SR-H-04 — moved Gap → Satisfied)._

| Area | Count | Satisfied | Partial | Gap | Residual-accepted |
|---|---|---|---|---|---|
| SR-SYS | 9 | 3 | 4 | 2 | 0 |
| SR-R | 15 | 10 | 0 | 5 | 0 |
| SR-H | 6 | 5 | 1 | 0 | 0 |
| SR-M | 6 | 4 | 0 | 0 | 2 |
| SR-I | 4 | 1 | 3 | 0 | 0 |
| **Total** | **40** | **23** | **8** | **7** | **2** |

"Partial" = the safety-argument leg is in place but a required test, quantification,
or a decomposed leg is still outstanding (e.g. SR-I-01/04 satisfied-by-contract but
golden-vector/replay tests missing; SR-SYS staleness/masking not yet quantified).

**Top-priority Gaps (implement first).** DU-1/2/3/4 (SR-R-09, SR-H-04, SR-R-03,
SR-H-03) are now **CLOSED** (see status cells + OPEN_ITEMS). The remaining
highest-value Gaps are **SR-R-08 / SR-R-12** (comparator positive-diff self-test
and encode golden-vector / second-encoder — DU-7/5 common-mode, β≈1), then
**SR-R-10 / SR-R-11 / SR-R-15** (pull-down CCF, settle-time characterization,
NVS peer-slot integrity) and the SR-SYS end-to-end tests (SR-SYS-01/02).

---

*End of first draft. SIL 3 / PL e is **allocated, not achieved**: the numeric
claim still gates on FMEDA, Option-B fault-injection (SR-R-03), and SC 3 process
evidence (HARA §6). Every decomposed-integrity claim is conditional on
independence being demonstrated. Curate against the vehicle HARA and the FMEDA
before any external safety claim.*
