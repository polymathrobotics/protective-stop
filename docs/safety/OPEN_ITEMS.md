<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Protective-Stop Safety & Coverage — Open Items & Next Steps

**Living tracker.** Updated as work progresses. This is the single place to see
what is decided, what is open, and what is next across the safety-analysis +
code-coverage effort. Newest status at the top of each section.

_Last updated: 2026-08-02 (rev 2 — #15/#16/#17 batch + stability battery + battery-tool fix)._

> **2026-08-02 — "do them in order" batch merged + verified:**
> 1. **#1 — SR-H-04b machine clock-freeze guard + DU-1 GPIO re-verify DONE**
>    (`1cfbd9a`): host+machn `get_time_cb` freeze guard (`clock_guard`, 5117-check
>    host test), and remote periodic E-stop GPIO pad-config re-verify → controlled
>    clean reset (`gpio_cfg_fault`, SR-R-09). **DU-1 CLOSED.**
> 2. **#2 — DU-3 object-code diversity gate DONE** (`d96129c`): verified Option-B
>    diverse expressions survive the optimizer (distinct object code) +
>    `scripts/check_estop_diversity.sh` CI guard. **DU-3 CLOSED.**
> 3. **#3 — production networking robustness DONE** (`1f7604e`,
>    `docs/PRODUCTION_NETWORKING.md`): same-LAN direct-path validated; DERP-home +
>    promote-direct-after-DISCO shipped.
> 4. **Stability battery (~30 min) PASSED:** power-cycle 6/6, arm/press/discordance
>    3/3, 4-min soak, `pstop_multi_remote_test` 34/34, real 2-chip E2E.
> 5. **`tools/stop_reset_battery.py` FIXED** (`b802c35`, `7c72e36`): self-contained
>    auto-spawn + strictly-monotonic send stamp; **22/22**. Both defects were
>    test-harness footguns, not firmware. See memory `reference_pstop_battery_tool_footguns`.

> **2026-08-02 — three-subagent batch merged (`347de6f`, on GitHub):**
> 1. **DERP reachability FIXED in firmware** (`ml_wg_mgr.c`): the chip was
>    installing an unvalidated netmap endpoint as the live WG endpoint; now
>    DERP-homed, direct only after bidirectional DISCO validation. Remote DUT
>    reachable + **full HIL smoke (arm / press→STOP / discordance) passed over
>    the fixed safety link** — this also closes the deferred firmware-extraction
>    HIL validation. No infra workaround needed.
> 2. **SR-H-04 / DU-2 CLOSED**: ESP32 TWDT/IWDT/RTC-WDT + dual-core mutual clock
>    cross-check (`docs/WATCHDOG_CLOCK_PROTECTION.md`); fault-injection proved a
>    frozen clock is caught by the cross-check (missed by the lockstep memcmp).
> 3. **Quantitative FMEDA** (`docs/safety/FMEDA.md`): SFF≈93% / PFH≈3.7e-9/h /
>    β=10%, SIL 3 met-but-fragile; closing DU-2 (done) + DU-1 → SFF≥99%.
>    **Escalated to user:** real ESP32-S3 + relay part numbers, and demand rate
>    + proof-test interval, to move from assumed to demonstrated.

---

## 1. Decisions log (settled)

| Date | Decision | Notes |
|---|---|---|
| 2026-08-02 | **Firmware coverage = HYBRID only** (host logic-harness for branch+MC/DC; on-target = functional/HIL smoke) | On-target gcov needs JTAG+OpenOCD (ESP-IDF), but **both JTAG routes are blocked by the design**: the physical JTAG pins GPIO39–42 *are* the E-stop loop pins (`main.c:207`), and USB-Serial-JTAG is displaced by the TinyUSB/NCM tether on the native USB. No no-solder JTAG path exists, and any external JTAG would break the safety wiring under test. (Side note: JTAG being consumed by the safety loops is a mild field-integrity property.) |
| 2026-08-02 | **OD-1 SETTLED — two coverage tools accepted (user-confirmed):** `pstop_c` keeps its own **Bullseye** coverage + CI (pre-qualified, not re-measured); **every other in-scope codebase uses GCC-14 gcov/gcovr** (line + branch + `-fcondition-coverage` MC/DC). | Rationale: `pstop_c` is pre-qualified with its existing Bullseye evidence, so re-tooling it buys nothing; new code standardizes on the free GCC-14 flow already wired into `scripts/coverage.sh` + CI. The two tools never measure the same code, so there is no cross-tool comparability problem. |
| 2026-08-02 | **MC/DC toolchain = GCC 14** (`-fcondition-coverage`) | Installed 14.3.0 via ubuntu-toolchain-r/test PPA; gcov-14 + gcovr condition coverage confirmed. No Bullseye license needed. |
| 2026-08-01 | Safety standard = **IEC 61508 (SIL) + ISO 13849 (PL)** | |
| 2026-08-01 | `pstop_c` = **pre-qualified** (referenced, not re-verified) | Add only interface/integration requirements at its boundary. |
| 2026-08-01 | Integrity target = **SIL 3 / PL e**; coverage bar = statement + branch + **MC/DC** | HARA confirms PL e; SIL 3 allocated (see gates in §3). |
| 2026-08-01 | Firmware structural coverage = **on-target ESP32 gcov** | Hardest path; transport TBD (JTAG vs UART vs console dump). |
| 2026-07-23 | **Option C dropped** (LFSR/PRNG + stamp signature) | A's both-phase covers stuck-at; deterministic challenge reproducible by a same-firmware fault. |

## 2. Open decisions (need input)

| ID | Decision | Status |
|---|---|---|
| ~~OD-1~~ | ~~Bullseye vs GCC-14 gcov~~ | **SETTLED 2026-08-02 (user-confirmed)** — see Decisions log §1: `pstop_c` keeps Bullseye; everything else GCC-14 gcov. |
| OD-2 | Vehicle-level HARA inputs (assumptions A-01..A-10 in HARA.md) — mass, speed, exposure, **demand rate W**, process safety time | Integrator must supply. SIL sits at 2 (W2) vs 3 (W3) pending W. |

## 3. Open safety items (from HARA / FMEA)

### 3a. SIL 3 quantitative-claim gates (from HARA)
- [ ] **G-FMEDA** — no FMEDA yet: SFF / DC / PFH / β unquantified. Blocks any numeric SIL 3 claim.
- [ ] **G-CCF** — common-cause coverage rests on Option B diversity; not quantified. Single toolchain / identical cores → limited HW diversity.
- [ ] **G-SC3** — systematic-capability (SC 3) process evidence beyond MISRA not yet demonstrated (static analysis in CI, unit + fault-injection tests, traceability — this effort).

### 3b. Dangerous-Undetected register (from FMEA — prioritized)
- [x] **DU-4 (F-H-02) / SR-H-03 — DONE 2026-08-02:** host runner now validates timing config at startup (`cfg_validate`, floors mirror the ROS2 node: heartbeat [50,1000], max_missed [1,5], min_stop ≥100) and **refuses to start** on unsafe values. Test `tools/test_config_floor.py` (7/7). Host coverage 70.4→71.4% L / 56.8→57.9% B / 58.3% MC-DC.
- [x] **DU-1 (F-R-01) — DONE 2026-08-02 (SR-R-09, `1cfbd9a`):** remote now periodically re-verifies the E-stop GPIO pad config (`gpio_ll_get_io_config` every ~10 ticks) and forces a controlled clean reset on mismatch (`gpio_cfg_fault`). Closes the "GPIO config-integrity check" the design doc claimed but code never implemented. _Remaining: on-bench fault-injection of a live pad-config corruption (bench-blocked, low value — logic is host-reasoned)._
- [x] **DU-2 (F-H-03 / F-P-03) — DONE 2026-08-02 (SR-H-04):** remote-side dual-core mutual clock cross-check + ESP32 TWDT/IWDT/RTC-WDT (`347de6f`, fault-injection validated). NOTE: this covers the *remote* clock; the *machine*-side `get_time_cb` freeze (host runner / ROS2) still wants an analogous guard — track as SR-H-04b.
- [x] **DU-3 (F-R-02) — DONE 2026-08-02 (SR-R-03, `d96129c`):** verified Option-B diverse expressions produce distinct object code (survive the optimizer, not collapsed to one form) + `scripts/check_estop_diversity.sh` CI guard so a future refactor can't silently erase the diversity. See `docs/safety/DU3_OBJECT_CODE_DIVERSITY.md`.
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
| Remote firmware (decision core `estop_verdict.c`) | host harness gcc-14 | **100% line/branch/MC-DC** (39/39) | Hybrid (on-target JTAG infeasible). Extracted from main.c behavior-preserving; firmware builds clean. **On-bench smoke of this build DEFERRED** — bench DERP-relayed, DUT HTTP unreachable. |
| `pstop_c` | Bullseye (own CI) | referenced | Pre-qualified; not re-measured here. |

**Firmware extraction — real-HW validation (2026-08-02):** OTA'd build `2a4d77c` to a **local** unit pstop-01d7ed38 (`10.43.0.195`, on `usb0`, no loop relay). Confirmed on-target: clean boot, loops settle to 1/1/1/1, verdict **OK**, **mm=0 (no nuisance divergence post-extraction)**, bonds to a local machine streaming OK/crc=ok/black-channel echo. **DONE 2026-08-02:** press→STOP / discordance / arm all validated on the (now-reachable) remote DUT `100.75.70.74` over the fixed WG safety link — arm held 3023 ms, press→STOP, single-channel discordance → STOP via liveness with 0 STOP-msgs on wire. Deferred smoke fully closed.

**DUT reachability diagnosis (2026-08-02).** From this host, PSTOP06 (pstop-01d7f344 @ 100.75.70.74) is reachable only via **DERP(sfo) relay** — `tailscale ping` succeeds (~40-120 ms) but **no direct path forms** (8/8 relayed, "direct connection not established"). Over DERP, disco pings + the TCP 3-way handshake complete, but **HTTP data never returns** ("Empty reply"/timeout), independent of HTTP version, an MSS clamp (1140), or a relay-3 power-cycle. Ruled out: httpd wedge (power-cycle didn't fix), MTU size (path MTU ~1250; clamp didn't fix), HTTP/1.1 keep-alive (HTTP/1.0 also fails). My NAT is friendly (`MappingVariesByDestIP: false`), and the **DUT fw already has the direct-path-healing + send-fail fixes (66df9ae)** — so the blocker is the **remote bench's NAT/firewall preventing direct WireGuard hole-punching**, forcing a DERP path that carries tiny packets but not the DUT's HTTP responses. (Earlier this session the direct path *did* form intermittently — 16 ms — so it's flaky, not permanently blocked.) The DUT's **outbound** path is fine (fleet check-ins work). **Not fixable by reflashing.** Remediation: run the HIL suite **on the bench host** (local USB-NCM, direct) as designed; or fix bench-side direct connectivity (subnet router / open WG UDP / DERP alignment). Fleet-push can still update DUT firmware (DUT pulls outbound) but won't give interactive HTTP for the smoke.

- [x] `docs/safety/COVERAGE.md` (host + ROS2 + firmware-core baselines). CI gate pending (task #10).
- [x] **Requirements-driven coverage (task #9) DONE** — `TRACEABILITY.md`: **25/39 SRs have a test (64.1%), 13/39 strictly verified (33.3%); 22/27 functions traced (81.5%)**. 13 unverified-gap SRs = the task #10 roadmap. `SR-R` firmware is weakest (8/15 unverified).
- **Function→SR holes to resolve:** F-R-10 admin/web needs a *non-interference* SR; F-H-04, F-M-01, F-M-06 need an SR or explicit non-safety scoping.

## 6. Deferred from prior safety work (A/B implementation)
- [ ] Design + FMEA doc — **now being produced** by this effort (supersedes the earlier deferred item).
- [ ] Upstream `pstop_c` far-Hamming status-encoding memo (for maintainers).
- [ ] Automated diversity HIL regression (fault-injection build → assert divergence).
- [x] **GitHub push DONE 2026-08-02** (per user): `origin/pstop` = `e40191d` — A+B safety work, full `docs/safety/` chain, coverage + gap closures. Secrets-audited clean (no fleet IP/creds; internal test IPs consistent with existing repo norm). [ ] `pstop`→`main` PR still open for reviewer sign-off.

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
5. ~~Aggregate report + CI gate (task #10)~~ — **DONE 2026-08-02**: `scripts/coverage.sh` + `.github/workflows/coverage.yml` (ros:humble + gcc-14), CI green (2m27s). Aggregate: firmware core 100% / host 71.4% / ROS2 52.4% line.
6. ~~firmware HIL press/discordance/arm~~ **DONE**; ~~SR-H-04 frozen-clock~~ **DONE (SR-H-04b)**; ~~DU-3 object-code check~~ **DONE**.

## 8. What actually remains (rev 2 — 2026-08-02)

**Needs your input (blocking, I cannot proceed without it):**
- [ ] **FMEDA firm-up (OD-2):** real ESP32-S3 + relay part numbers, demand rate **W**, and proof-test interval — moves SIL 3 from *allocated/assumed* to *demonstrated*, and sets W2(SIL2) vs W3(SIL3). Also unblocks HARA **H-03** (DERP failover 5–9 s dark window vs process-safety-time budget; may then need a local-STOP latch mitigation).
- [ ] **OD-1:** confirm the two-tool coverage split is acceptable (Bullseye stays with `pstop_c`; GCC-14 gcov everywhere else).

**Blocked on hardware / people:**
- [ ] **machn (ESP32 machine) clock-guard HW validation** — needs a live machine node + relay HIL bench (host+machn logic done + host-tested).
- [ ] **`pstop`→`main` PR** reviewer sign-off (your call; not reminding).

**Open engineering, lower priority:**
- [ ] Coverage headroom: ROS2 node/backends (need a spun executor) + host branch paths. Firmware core already 100%.
- [ ] Remaining DU rows: **DU-5/7** comparator/encode common-mode (β≈1 shared downstream), **DU-6/8** pull-down CCF + settle, **SR-R-15** NVS peer-slot integrity.
- [ ] On-bench fault-injection of SR-R-09 pad-config corruption (bench-blocked, low value).
- [ ] Repo-wide pre-commit cosmetic cleanup (deferred maintainer-level debt; ~22 files reformat).
- [ ] Upstream `pstop_c` far-Hamming status-encoding memo (maintainers).
