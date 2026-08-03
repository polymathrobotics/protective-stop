<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Protective-Stop — Requirements Traceability Matrix & Requirements-Driven Coverage

**Status:** verification-engineering work product. Traces the 39 safety
requirements (`docs/safety/SAFETY_REQUIREMENTS.md`) bidirectionally to the code
that implements them and the tests that exercise them, then computes
requirements-driven coverage. Spine = the SR table; function IDs =
`docs/safety/SYSTEM_DEFINITION.md` §4.

> **Standing caveat (unchanged from the SRS).** Target integrity **SIL 3 / PL e**
> is **allocated, not achieved**. The numeric claim gates on FMEDA, Option-B
> fault-injection (SR-R-03), and SC 3 process evidence (HARA §6). Nothing in this
> matrix advances that claim — it measures *verification completeness against the
> requirement set*, not the systematic-capability argument. Every
> integrity-decomposition claim in the SRS remains conditional on independence
> being demonstrated (β unquantified).

---

## 1. Method

Trace is **bidirectional**: each SR → its allocated function(s) F-xx →
implementation `file:line` (taken from the SRS "Satisfied" citations, not
re-derived) → the specific test(s) that exercise it, and back (function → SR
reverse map, §5). A requirement is scored **Verified** only when a *real test in
this tree actually exercises the behaviour it claims* — implementation existing
is **not** verification. **Requirements coverage** is reported two ways: **(a)**
the fraction of SRs with ≥1 passing verifying test (test-existence), and **(b)**
the fraction of safety-relevant functions F-xx traced to ≥1 SR
(requirement-completeness); §5 exposes any function with **no** requirement as a
coverage hole. Structural coverage (statement/branch/MC-DC) is credited only
where a host harness measures it — 100% line/branch/MC-DC on
`firmware/main/estop_verdict.c` (the extracted decision core) and 70.4% line /
56.8% branch on `host/machine_app_runner.c` (`docs/safety/coverage/host-summary.json`);
the ROS 2 node has no coverage run yet. The pre-qualified `pstop_c` library is
referenced at its interface only (its own `req_*_test.c` suite is cited as
evidence for the SR-I integration obligations; internals are **not** re-verified).

**Status vocabulary**

| Status | Meaning |
|---|---|
| **Verified** | ≥1 real test exercises the full claimed behaviour (passing). |
| **Partially-verified** | The safety-argument leg is implemented and *some* test touches it, but a required test, quantification, or decomposed leg is outstanding. |
| **Unverified-gap** | No test exercises the requirement in-tree — whether unimplemented (SRS "Gap") **or** implemented-but-untested. Both are verification holes. |
| **Residual-accepted** | Deliberately not implemented / non-safety; rationale recorded in the SRS. |

---

## 2. Traceability matrix

Test-file shorthand:
`EV` = `firmware/test/test_estop_verdict.c`;
`MR` = `tools/pstop_multi_remote_test.py` (group letter in brackets);
`HIL10/20/30` = `tools/hil/test_10_button.py` / `test_20_discordance.py` / `test_30_power_cycle.py`;
`JL` = `ros2/protective_stop_machine/test/test_json_lite.cpp`;
`REQ n_nn` = `pstop_c/pstop/test/src/pstop/requirements/req_n_nn_test.c`.

### 2.1 System level (SR-SYS)

| SR | Alloc F-xx | Code (file:line) | Verifying test(s) | Method | Status |
|---|---|---|---|---|---|
| SR-SYS-01 | F-R-01..05, F-H-03, F-M-03/04, final elem | timing budget (HARA A-05) | **NO TEST** measures end-to-end latency; HIL20/30 wait *within* a heartbeat budget but assert STOP-occurred, not the ≤1.3 s internal budget | Analysis + Test (latency) | **Unverified-gap** |
| SR-SYS-02 | F-R-02/06, F-H-03, F-P-03/04 | endpoint freshness path | Constituents: EV (fresh-sample OK), MR[F] (bad-CRC→STOP), REQ 2_02/2_03 (replay/lost). **End-to-end staleness/forgery NO TEST** | Test + Fault-inj | **Partially-verified** |
| SR-SYS-03 | F-R-03, F-H-02, F-M-03 | arming path | HIL10 (fresh needs gesture; short-blip defers), HIL30 (boot no re-arm), MR[A] I3/I4 | Test | **Partially-verified** (host floor SR-H-03 unverified) |
| SR-SYS-04 | F-H-05, F-P-04 | many-to-one OR | MR[B/D/E/F] 32/32 (any-STOP OR, ownership) | Test (E2E) | **Partially-verified** (ghost-bond masking unquantified) |
| SR-SYS-05 | F-M-03/04, `machn` relay | `machn` series relay + fb | SR-M-03/04 tested (JL/backend default). **`machn` hardware final element NO TEST in this tree** | Analysis + Test | **Partially-verified** |
| SR-SYS-06 | F-R-03 | `main.c` reclose-only debounce | EV #1 ("open → immediate STOP" same tick; reclose re-runs debounce) | Test | **Verified** |
| SR-SYS-07 | F-P-02, F-H/M dispatch | `machine.c` default→STOP | MR[A] I3, HIL10 (fresh needs gesture), HIL30 (boot), REQ 3_04 | Inspection + Test | **Verified** |
| SR-SYS-08 | F-R-06/07, F-P-01/04 | endpoint CRC/echo/clamp | REQ 2_01 (corruption), MR[F] (bad-CRC→STOP). **Residual bit-error/FMEDA NO TEST** | Analysis | **Partially-verified** |

### 2.2 Remote firmware (SR-R)

| SR | Alloc F-xx | Code (file:line) | Verifying test(s) | Method | Status |
|---|---|---|---|---|---|
| SR-R-01 | F-R-01 | `main.c:265-290` (`estop_channel_closed`) | EV #1/#2 (both-phase truth table; every fault→STOP), 100% MC-DC on `estop_verdict.c` | Test + Insp | **Verified** |
| SR-R-02 | F-R-02 | `main.c:304-306,340-348` | EV #2 (arith-image select; no fault manufactures OK), MC-DC | Test + MC/DC | **Verified** |
| SR-R-03 | F-R-02 | `main.c:305/307` | EV #3 proves **equivalence** (core0==core1), **not** the required object-code non-identity / identical-fault-injection. Object-code diff **NOT** build-gated | Fault-inj + obj-diff | **Unverified-gap** |
| SR-R-04 | F-R-02 | `main.c:340-348` (STOP-only override) | EV #2 (health/debounce never lowers to OK) | Test + MC/DC | **Verified** |
| SR-R-05 | F-R-03 | `main.c:323-347` | EV #1 (OPEN→STOP same tick; only reclose debounced) | Test | **Verified** |
| SR-R-06 | F-R-03 | `main.c:359-363,903` (`estop_primed`) | EV #4/#4b (primed only when both cores settle), HIL30 (cold boot no re-arm; held-open→STOP flows) | Test | **Verified** |
| SR-R-07 | F-R-04 | `main.c:887-903` (transmit-on-agreement) | HIL20 (single-channel divergence → comparator silent → machine liveness STOP; no STOP on wire) | Test + Fault-inj | **Verified** |
| SR-R-08 | F-R-04 | (relies on libc `memcmp`) | **NO TEST** — per-offset positive-diff self-test absent (DU-7) | Fault-inj | **Unverified-gap** |
| SR-R-09 | F-R-01 | pad config set once `main.c:229-247`, never re-verified | **NO TEST** — periodic GPIO re-verify not implemented (DU-1, highest priority) | Fault-inj | **Unverified-gap** |
| SR-R-10 | F-R-01 | not implemented | **NO TEST** — common-cause pull-down detection absent (DU-6) | Fault-inj + Analysis | **Unverified-gap** |
| SR-R-11 | F-R-01 | `ESTOP_SETTLE_US` bench constant | **NO TEST** — settle-vs-deployed-cable characterization absent (DU-8) | Analysis + Test | **Unverified-gap** |
| SR-R-12 | F-R-05, F-P-01 | single shared encode image | **NO TEST** — golden-vector encode test absent (DU-5) | Test + Fault-inj | **Unverified-gap** |
| SR-R-13 | F-R-06 | `main.c:741/746/767/779` (echo relayed, not synthesized) | Library anti-replay REQ 2_02/2_03; MR[F] bad-CRC→STOP. **Remote-side spoofed-reply→no-OK integration NO TEST** | Test + Insp | **Partially-verified** |
| SR-R-14 | F-R-06/07 | `main.c:568,572-579` (`sess_ensure_socket` dark on VPN=0) | **NO TEST** — VPN-drop regression absent (implemented, untested) | Test | **Unverified-gap** |
| SR-R-15 | F-R-09 | not implemented (peer slots not CRC/version checked) | **NO TEST** (FMEA R09-1) | Test | **Unverified-gap** |

### 2.3 Host machine (SR-H)

| SR | Alloc F-xx | Code (file:line) | Verifying test(s) | Method | Status |
|---|---|---|---|---|---|
| SR-H-01 | F-H-03, F-P-03 | `machine.c:290-298`; host `machine_app_runner.c:721` | MR[C] I2 (silence→STOP), MR[F] (timeout→STOP), HIL20/30 liveness | Test | **Verified** |
| SR-H-02 | F-H-02 | `machine_app_runner.c:375-379,834-844,666` | MR[A] A2a/A2b/A4 I4 (short cycle refused), HIL10 (short blip defers) | Test | **Verified** |
| SR-H-03 | F-H-02 | `machine_app_runner.c:228-241` (`cfg_load` raw `parse_uint`, no floor) | **NO TEST** — host safety-envelope clamp not implemented (DU-4, cheap/high value) | Test | **Unverified-gap** |
| SR-H-04 | F-H-03, F-P-03 | `machine.c:282` (skips when `now ≤ last`); no independent clock | **NO TEST** — frozen/backward-clock detection absent (DU-2, highest priority) | Fault-inj | **Unverified-gap** |
| SR-H-05 | F-H-05 | `pstop_c` OR property | MR[B/D/E] (OR, ownership, ghost bond). Random-fault ghost-bond masking unquantified | Test | **Partially-verified** |
| SR-H-06 | F-H-01 | `machine_app_runner.c:752-757` (pre-dispatch filter) | MR[F] F3 (out-of-range type: no crash, no spurious arm), MR[E] allowlist | Insp + Test | **Verified** |

### 2.4 ROS 2 machine (SR-M)

| SR | Alloc F-xx | Code (file:line) | Verifying test(s) | Method | Status |
|---|---|---|---|---|---|
| SR-M-01 | F-M-07 | `machine_bridge_node.cpp:72-97,323,354,28-35` | `test_timing_floors` (8/8, every accept/reject boundary) + `test_node_runtime` runtime rejection: `SetParametersRejectsUnsafeMinStop`/`RejectsOversizeHeartbeat`/`AcceptsTighter` (set-param) and `ConfigureServiceApplies`/`RejectsUnsafe` (service) — reject-loosening exercised at unit, param, and service levels (closes M07-1/DU-4) | Test | **Verified** (2026-08-02) |
| SR-M-02 | F-M-04/05 | `hardware_backend.cpp:119` (`bool_at("relay_stop", true)`) | JL exercises `bool_at` default mechanism (present-key), **not** the missing-key→STOP safety default directly | Test | **Partially-verified** |
| SR-M-03 | F-M-04, F-M-08 | `hardware_backend.cpp` / `machine_bridge_node.cpp:286` | `test_hardware_parse` (malformed/empty→blind) + `test_hardware_backend` `ClosedPortUnreachable`/`Non2xxUnreachable` (backend→unreachable) + `test_node_runtime` `HardwareUnreachableDiagnosesError` (state→UNSTABLE **and** diagnostics ERROR) — closes M04-1 | Test | **Verified** (2026-08-02) |
| SR-M-04 | F-M-03 | `software_backend.cpp:199` (`relay.applicable=false`) | Inspection (safety-case scope statement) | Inspection | **Residual-accepted** |
| SR-M-05 | F-M-05 | `json_lite.hpp:87,97` (depth cap 32) | JL `RejectsPathologicalNesting` (500-deep → clean reject, no crash), `RejectsMalformed` | Test | **Verified** |
| SR-M-06 | F-M-05 | `json_lite.hpp:185-198` | JL `NumbersBoolsNull` (tolerant numeric parse) — confirms no safety field is numeric | Inspection + Test | **Residual-accepted** |

### 2.5 `pstop_c` interface / integration (SR-I)

| SR | Alloc F-xx | Code (file:line) | Verifying test(s) | Method | Status |
|---|---|---|---|---|---|
| SR-I-01 | F-P-01, F-R-05 | encode `main.c:422`, decode `:745` (library size/span) | REQ 2_01 (CRC corruption detect); MR drives real 40-byte + CRC16 wire. **Golden-vector pin NO TEST** (= SR-R-12) | Test | **Partially-verified** |
| SR-I-02 | F-P-02, F-H-01/F-M-02 | `machine.c:196-201`; host `:752-757`; ROS `build_backend` | REQ 2_08 (only OK/STOP/BOND/UNBOND), REQ 2_07 (wrong machine-ID reject), REQ 3_04 (stay STOP until bond+OK); MR[F] malformed→STOP; MR[A] I3 | Test + Insp | **Verified** |
| SR-I-03 | F-P-03, F-H-03 | host binds `CLOCK_MONOTONIC` | REQ 3_05/3_01 (heartbeat-rate validity). **Frozen-clock diagnostic NO TEST** (= SR-H-04) | Fault-inj | **Partially-verified** |
| SR-I-04 | F-P-04, F-R-06 | remote relays echo, no synth `main.c:767,779` | REQ 2_02 (out-of-order), REQ 2_03 (lost), REQ 2_01 (corruption). **Shell-level replay integration NO TEST** | Test + Insp | **Partially-verified** |

---

## 3. Requirements coverage summary

### 3.1 Headline numbers

- **(a) SRs with ≥1 passing verifying test: 27 / 39 = 69.2 %** (was 25/39;
  SR-M-01 + SR-M-03 gained verifying ROS 2 tests 2026-08-02).
  (15 Verified + 11 Partially-verified + 1 Residual-with-test [SR-M-06]. The
  remaining Residual [SR-M-04] is inspection-only; the 11 Unverified-gap SRs
  have no test.)
  **Strict, fully-verified only: 15 / 39 = 38.5 %.** This is the honest number
  for "requirement completely discharged by test" — the 11 Partials each leave a
  named leg (end-to-end, quantification, or golden-vector/replay) untested.

- **(b) Safety functions F-xx traced to ≥1 SR: 22 / 27 = 81.5 %.**
  Five functions carry **no** requirement (§5): F-R-08, F-R-10 (both declared
  **non-safety**), and **F-H-04, F-M-01, F-M-06** (undeclared — genuine
  requirements-coverage holes). Excluding the two declared-non-safety functions:
  22 / 25 = 88.0 %.

### 3.2 Breakdown by area

| Area | Count | Verified | Partially-verified | Unverified-gap | Residual-accepted | ≥1-test % | Fully-verified % |
|---|---|---|---|---|---|---|---|
| SR-SYS | 8 | 2 | 5 | 1 | 0 | 87.5 % | 25.0 % |
| SR-R | 15 | 6 | 1 | 8 | 0 | 46.7 % | 40.0 % |
| SR-H | 6 | 3 | 1 | 2 | 0 | 66.7 % | 50.0 % |
| SR-M | 6 | 3 | 1 | 0 | 2 | 83.3 %† | 50.0 % |
| SR-I | 4 | 1 | 3 | 0 | 0 | 100 % | 25.0 % |
| **Total** | **39** | **15** | **11** | **11** | **2** | **69.2 %** | **38.5 %** |

† SR-M ≥1-test counts SR-M-01/03/05 (Verified) + SR-M-02 (Partial) + SR-M-06
(Residual-with-test) = 5/6 = 83.3 % (SR-M-01/03 verified 2026-08-02).

**Reading:** SR-I is fully *touched* by the pre-qualified `pstop_c` suite but
never *completed* (golden-vector + replay integration missing). **SR-R is the
weak area** — 8 of 15 have no test, and it holds five of the six
highest-priority DU gaps. Structural coverage confirms the split: the decision
core (`estop_verdict.c`) is at 100 % MC-DC, the host runner sits at 56.8 %
branch, and the ROS 2 node — now structurally covered (2026-08-02) — is at
~89 % line (`machine_bridge_node.cpp` 98 %, `hardware_backend.cpp` 99 %; see
COVERAGE.md §1).

---

## 4. Test-gap register (input to "drive coverage up")

Ranked. **P0** = highest-priority DU / cheapest-high-value first (OPEN_ITEMS §3b);
**P1** = implemented-but-untested (test only, no code work); **P2** =
Partial→Verified closers; **P3** = requirements-completeness holes (§5).

| Rank | SR | Gap | DU / cross-ref | Work needed |
|---|---|---|---|---|
| **P0-1** | SR-H-03 | Host accepts unsafe timing config (no floor) | **DU-4** (cheapest) | Implement compiled envelope in `cfg_load`; test rejects `min_stop_ms=0`, huge heartbeat |
| **P0-2** | SR-R-09 | GPIO pad config never re-verified → false-OK | **DU-1** (top) | Implement periodic register read-back + drive-low self-test; fault-injection test |
| **P0-3** | SR-H-04 | Frozen/backward clock disables heartbeat | **DU-2** (top), SR-I-03 | Independent clock / HW watchdog; freeze-clock fault-injection → STOP |
| **P0-4** | SR-R-03 | Compiler may erase Option-B diversity | **DU-3** | Object-code diff gate **or** compile-diverse TUs + identical-fault-injection test (EV #3 proves equivalence, not this) |
| **P0-5** | SR-R-08 | `memcmp` mis-returns equal undetected | **DU-7** | Per-offset positive-diff self-test (40 offsets) |
| **P0-6** | SR-R-12 | Common-mode encode fault | **DU-5**, SR-I-01 | Golden-vector encode test + second encoder image |
| P1-1 | SR-R-14 | VPN-drop → dark path (implemented, untested) | FMEA R06-3 | VPN-drop regression test |
| ~~P1-2~~ | SR-M-01 | ~~`validate_timing` reject-loosening~~ **DONE 2026-08-02** (`test_node_runtime` set-param + service rejection, `test_timing_floors`) | DU-4/M07-1 | ✅ ROS 2 set-param rejection test added |
| ~~P1-3~~ | SR-M-03 | ~~Unreachable `/state.json`→UNSTABLE~~ **DONE 2026-08-02** (`test_hardware_backend` + `test_node_runtime` `HardwareUnreachableDiagnosesError`) | M04-1 | ✅ Drop-`/state.json` backend + node test added |
| P1-4 | SR-M-02 | Missing-`relay_stop`→STOP default (partial) | M04-2 | Field-level missing-key→true test |
| P2-1 | SR-I-01 | Golden-vector encode/decode both ends | (= SR-R-12) | Author golden-vector both-end pin |
| P2-2 | SR-I-04 | Shell-level replay integration | HARA §7 item 6 | Replay reply → no-OK integration test |
| P2-3 | SR-R-13 | Remote-side spoofed-reply→no-OK | R06-1 | Replay-reply integration on remote |
| P2-4 | SR-SYS-01 | End-to-end latency ≤1.3 s unmeasured | HARA A-05 | Injected-demand→STOP latency measurement |
| P2-5 | SR-SYS-02 | End-to-end staleness/forgery | H-05/H-11 | Stale/replayed-OK end-to-end test |
| P2-6 | SR-SYS-04 / SR-H-05 | Ghost-bond random-fault masking unquantified | — | FMEDA / random-fault campaign |
| P2-7 | SR-SYS-05 | `machn` hardware final element untested here | H-10/A-07 | `machn` series-relay + feedback bench test |
| P2-8 | SR-SYS-08 | Residual bit-error/masquerade rate | A-08 | FMEDA feed (analysis) |
| P3 | SR-R-10, SR-R-11, SR-R-15 | Unimplemented DU/config items | DU-6/DU-8/R09-1 | Implement, then test |

**Not in the register (deliberate):** SR-M-04 and SR-M-06 are Residual-accepted
(non-safety software backend / no numeric safety field) — no test owed.

---

## 5. Function → SR reverse map

Every F-xx from `SYSTEM_DEFINITION.md` §4 and the SR(s) whose "Allocated to"
touches it. A function with **no SR** is a requirements-coverage hole.

### 5.1 Remote firmware (F-R)

| F-xx | Function | SRs touching it |
|---|---|---|
| F-R-01 | Dual-channel loop sensing | SR-SYS-01, SR-R-01/09/10/11 |
| F-R-02 | Per-core verdict (Option A/B) | SR-SYS-02, SR-R-02/03/04 |
| F-R-03 | Release debounce + boot hold | SR-SYS-03/06, SR-R-05/06 |
| F-R-04 | Lockstep comparator | SR-R-07/08 |
| F-R-05 | Message encode + transmit | SR-SYS-01, SR-R-12, SR-I-01 |
| F-R-06 | Black-channel round-trip | SR-SYS-02/08, SR-R-13/14, SR-I-04 |
| F-R-07 | Network transport mgmt | SR-SYS-08, SR-R-14 |
| **F-R-08** | Status LED ring | **— none (declared non-safety)** |
| F-R-09 | Boot / NVS / identity | SR-R-15 |
| **F-R-10** | Admin/web interface | **— none (declared non-safety; SYSTEM_DEFINITION requires a *non-interference* argument, currently unrequirement'd)** |

### 5.2 Host machine (F-H)

| F-xx | Function | SRs touching it |
|---|---|---|
| F-H-01 | Host `pstop_c` machine role | SR-H-06, SR-I-02 |
| F-H-02 | Arming policy | SR-SYS-03, SR-H-02/03 |
| F-H-03 | Heartbeat/liveness | SR-SYS-01/02, SR-H-01/04, SR-I-03 |
| **F-H-04** | Robot status output + logging | **— none (HOLE: undeclared, no SR)** |
| F-H-05 | Many-to-one aggregation | SR-SYS-04, SR-H-05 |

### 5.3 ROS 2 machine (F-M)

| F-xx | Function | SRs touching it |
|---|---|---|
| **F-M-01** | Lifecycle node mgmt | **— none (HOLE: undeclared, no SR)** |
| F-M-02 | Backend abstraction | SR-I-02 |
| F-M-03 | Software backend | SR-SYS-01/03/05, SR-M-04 |
| F-M-04 | Hardware backend | SR-SYS-01/05, SR-M-02/03 |
| F-M-05 | JSON parsing | SR-M-02/05/06 |
| **F-M-06** | Publish status/relay/bonded on ROS topics | **— none (HOLE: undeclared, no SR)** |
| F-M-07 | Parameter validation + reconfig | SR-M-01 |
| F-M-08 | Diagnostics | SR-M-03 |

### 5.4 Pre-qualified `pstop_c` interface (F-P)

| F-xx | Function | SRs touching it |
|---|---|---|
| F-P-01 | Encode/decode + CRC-16 | SR-SYS-08, SR-R-12, SR-I-01 |
| F-P-02 | Machine dispatch (default→STOP) | SR-SYS-07, SR-I-02 |
| F-P-03 | Heartbeat on monotonic clock | SR-SYS-02, SR-H-01/04, SR-I-03 |
| F-P-04 | Counter/stamp echo + LOST/OOO | SR-SYS-02/04/08, SR-R-14(via 06), SR-I-04 |

### 5.5 Requirements-coverage holes

| F-xx | Nature | Action |
|---|---|---|
| F-R-08 | LED status — declared non-safety | Acceptable; note in safety-case scope. |
| F-R-10 | Admin/web — declared non-safety | **Needs a non-interference requirement** — SYSTEM_DEFINITION §6 demands F-R-10 be *shown non-interfering* with the verdict path; no SR currently captures that obligation. |
| **F-H-04** | Robot status output / operator logging | **Add an SR** or explicitly scope non-safety. It carries the operator-visible STOP/OK signal — arguably needs a "must reflect committed verdict" requirement. |
| **F-M-01** | ROS 2 lifecycle mgmt | **Add an SR** or scope non-safety. Lifecycle transitions gate activation of the safety path (configure/activate/error). |
| **F-M-06** | ROS 2 topic publication | **Add an SR** or scope non-safety. Publishes status/relay/bonded-remotes; if any downstream consumer treats these as safety, an obligation is missing. |

---

*End of traceability matrix. Coverage figures measure verification completeness
against the requirement set only; they do **not** substantiate the SIL 3 / PL e
claim, which remains allocated-not-achieved pending FMEDA, Option-B
fault-injection (SR-R-03), and SC 3 process evidence.*
