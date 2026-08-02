<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Protective-Stop Safety & Coverage — Open Items & Next Steps

**Living tracker.** Updated as work progresses. This is the single place to see
what is decided, what is open, and what is next across the safety-analysis +
code-coverage effort. Newest status at the top of each section.

_Last updated: 2026-08-02._

---

## 1. Decisions log (settled)

| Date | Decision | Notes |
|---|---|---|
| 2026-08-02 | **MC/DC toolchain = GCC 14** (`-fcondition-coverage`) | Installed 14.3.0 via ubuntu-toolchain-r/test PPA; gcov-14 + gcovr condition coverage confirmed. No Bullseye license needed. |
| 2026-08-01 | Safety standard = **IEC 61508 (SIL) + ISO 13849 (PL)** | |
| 2026-08-01 | `pstop_c` = **pre-qualified** (referenced, not re-verified) | Add only interface/integration requirements at its boundary. |
| 2026-08-01 | Integrity target = **SIL 3 / PL e**; coverage bar = statement + branch + **MC/DC** | HARA confirms PL e; SIL 3 allocated (see gates in §3). |
| 2026-08-01 | Firmware structural coverage = **on-target ESP32 gcov** | Hardest path; transport TBD (JTAG vs UART vs console dump). |
| 2026-07-23 | **Option C dropped** (LFSR/PRNG + stamp signature) | A's both-phase covers stuck-at; deterministic challenge reproducible by a same-firmware fault. |

## 2. Open decisions (need input)

| ID | Decision | Status |
|---|---|---|
| OD-1 | Bullseye in CI to *match `pstop_c`* vs standardize on GCC-14 gcov everywhere | Leaning GCC-14 (now installed). `pstop_c` keeps Bullseye; new codebases use gcov. Confirm acceptable to have two tools. |
| OD-2 | Vehicle-level HARA inputs (assumptions A-01..A-10 in HARA.md) — mass, speed, exposure, **demand rate W**, process safety time | Integrator must supply. SIL sits at 2 (W2) vs 3 (W3) pending W. |

## 3. Open safety items (from HARA / FMEA)

### 3a. SIL 3 quantitative-claim gates (from HARA)
- [ ] **G-FMEDA** — no FMEDA yet: SFF / DC / PFH / β unquantified. Blocks any numeric SIL 3 claim.
- [ ] **G-CCF** — common-cause coverage rests on Option B diversity; not quantified. Single toolchain / identical cores → limited HW diversity.
- [ ] **G-SC3** — systematic-capability (SC 3) process evidence beyond MISRA not yet demonstrated (static analysis in CI, unit + fault-injection tests, traceability — this effort).

### 3b. Dangerous-Undetected register (from FMEA — prioritized)
- [ ] **DU-4 (F-H-02) — CHEAP, HIGH VALUE:** host runner does no timing-config validation → `min_stop_ms=0` defeats arming (SF-3); huge heartbeat defeats stop latency (SF-1). ROS2 node validates; host does not. **Fix: port the ROS2 validation to the host runner.**
- [ ] **DU-1 (F-R-01) — SMALL:** lost GPIO pull-down → open loop reads *closed* → false OK, no runtime detection. Pad config set once at `estop_init`, never re-verified. **Fix: periodic pull-down / pin-config re-verification → STOP on mismatch.** (This is the "GPIO config-integrity check" the design doc claims but code never implemented.)
- [ ] **DU-2 (F-H-03 / F-P-03):** frozen `get_time_cb` (CLOCK_MONOTONIC) silently disables the heartbeat watchdog. Mitigation options: independent clock cross-check / sanity on time advance.
- [ ] **DU-3 (F-R-02):** Option B diversity may be compiler-erasable. **Action: verify against object code** whether reads go through `volatile`/`esp_rom` barriers before treating as real.
- [ ] **DU-5/DU-7 (F-R-04/05):** lockstep blind to common-mode faults in shared downstream code (encode / `memcmp` / priming, β≈1).
- [ ] Remaining 3 DU rows — see FMEA.md §3.

### 3c. HARA top risks
- [ ] **H-03 — DERP failover dark window (5–9 s)** likely exceeds process safety time for a short-PST vehicle. Needs PST budget (OD-2) + a mitigation (faster failover / local-STOP latch).
- [ ] **H-10/H-12 — software machine has no rated final element:** host/ROS2 STOP is explicitly non-safety. The ESP32 machine (dual series relays + feedback, HFT=1) is the only rated actuator. State clearly in the safety case; software-machine deployments are non-safety-rated.
- [ ] **`machn` double relay-weld** — proof-test-interval item for PL e (from FMEA).

## 4. Doc reconciliation (DONE 2026-08-02 — task #11)
Applied. Register: **`docs/safety/RECONCILIATION.md`** (R-01…R-09). The docs were
annotated in place — stale text retained with bold status markers + inline
`[Reconciled 2026-08-02]` notes; wrong values corrected directly.
- [x] §5.5/§6/§10#8 claim a runtime **GPIO config-integrity check** — never implemented (root of DU-1). Marked **NOT IMPLEMENTED** (R-02).
- [x] §5–§7/§9/§10 describe **Option C** (LFSR challenge, `S^=(Cn^En)` accumulator, stamp signature) as live — dropped; real controls Option A (both-phase) + B (diverse expressions). §5 marked **DROPPED 2026-07** (R-01).
- [x] Heartbeat-timeout formula/values stale: `hb×(max_missed+1)`, 1000×2 → corrected to `hb×max_missed`, 400×3 ≈ 1.2 s in `PSTOP_SAFETY_DESIGN.md` §5.6, `MULTI_REMOTE_VALIDATION` and the `machine_app_runner.c:721` comment (R-03/R-07/R-08).
- [x] §5.4 "semantic 1oo2D" vs byte `memcmp` — framing reconciled: byte-identical healthy encodings make memcmp sufficient (semantic comparator made unnecessary, not skipped); common-cause still needs FMEDA (R-04).
- [x] Whole doc SIL2-framed → SIL3/PLe banner added; rationale retained (R-05).
- [x] **NEW (R-09):** §2 "rolling drive level" description was itself stale (one-phase-per-tick) — code samples both phases every tick and ignores the counter (`main.c:269`); marked superseded.

## 5. Coverage status & next steps

Full report: **`docs/safety/COVERAGE.md`**.

| Codebase | Tool | Baseline | Status |
|---|---|---|---|
| Host machine (`machine_app_runner.c`) | gcov/gcovr (→ gcc-14 MC/DC) | **70.4% line / 56.8% branch** (fn 90.5%) | Baseline done; re-run under gcc-14 for condition coverage; drive branch up. |
| ROS2 machine | colcon + gcov/gcovr | **`json_lite` 84% line / 53% branch; node+backends 0%** | Baseline done. Only 2/114 TUs execute — node/backends have no unit tests (primary debt). |
| Remote firmware | on-target ESP32 gcov | — | Not started (task #8). Transport decision pending (JTAG on bench?). |
| `pstop_c` | Bullseye (own CI) | referenced | Pre-qualified; not re-measured here. |

- [x] `docs/safety/COVERAGE.md` started (host + ROS2 baselines). CI gate pending (task #10).
- [ ] Requirements-driven coverage (task #9) — requirements now exist; build `TRACEABILITY.md` next.

## 6. Deferred from prior safety work (A/B implementation)
- [ ] Design + FMEA doc — **now being produced** by this effort (supersedes the earlier deferred item).
- [ ] Upstream `pstop_c` far-Hamming status-encoding memo (for maintainers).
- [ ] Automated diversity HIL regression (fault-injection build → assert divergence).
- [ ] **GitHub push + `pstop`→`main` PR still HELD** pending reviewer sign-off. `origin/pstop` = `acf775a` (pre A+B). A+B validated + in Nextcloud (`eed75f3`).

## 6b. Safety requirements (DONE 2026-08-02 — task #5)
**`docs/safety/SAFETY_REQUIREMENTS.md`** — 39 requirements (`SR-<area>-nn`): SYS 8, R 15,
H 6, M 6, I 4. Status: **18 Satisfied · 8 Partial · 11 Gap · 2 Residual-accepted.**
Every DU-1..9 discharged; SG-6 fully discharged, SG-1..5 partial. SIL3/PLe kept
**allocated, not achieved**. Gap requirements = the implementation roadmap:
- [ ] **SR-H-03** host config-floor validation (DU-4) — cheapest/highest value.
- [ ] **SR-R-09** GPIO config re-verify → STOP (DU-1).
- [ ] **SR-H-04** frozen-clock diagnostic (DU-2).
- [ ] **SR-R-03** Option-B object-code-diff gate (DU-3).
- [ ] **SR-R-08/12** comparator/encode common-mode (DU-5/7); **SR-R-10/11** pull-down CCF + settle (DU-6/8); **SR-R-15** NVS peer-slot integrity.
- Full list + trace in SAFETY_REQUIREMENTS.md §7–§9.

## 7. Next steps (working queue)
1. ~~Deep doc reconciliation (task #11)~~ — **DONE**. ~~Derive requirements (task #5)~~ — **DONE**.
2. **Structural coverage baselines:** re-run host under **gcc-14** (line+branch+MC/DC); ROS2 (task #7); firmware on-target (task #8).
3. **Requirements-traceability + coverage** (task #9): map each SR → code → test; report % SRs with a verifying test + % safety lines traced to an SR.
4. **Implement Gap requirements** (SR-H-03, SR-R-09 first) as SR-traced tests land — drives both coverages up.
5. Aggregate report + CI gate (task #10). Curate HARA/FMEA residual caveats (DU-3 object-code check).
