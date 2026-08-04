<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Protective-Stop — Documentation ↔ Code Reconciliation Register

**Date:** 2026-08-02 · **Basis:** current firmware/host/library tree.
**Method:** every safety claim in the design docs cross-checked against the
code cited in `docs/safety/SYSTEM_DEFINITION.md` (the authoritative backbone).
Where a doc disagreed with the code, **the code wins**; the doc was corrected
in place. Per the certification-track rule, **no design rationale was deleted** —
stale text is retained and carries a bold status marker (**DROPPED 2026-07**,
**NOT IMPLEMENTED**) and/or an inline `> **[Reconciled 2026-08-02]**` note.
Simply-wrong numeric/formula values were corrected directly.

**Seed findings** were the HARA §7 "Doc-vs-code register" (D1–D6) and the FMEA
§6 "Doc-vs-code inconsistencies"; each was re-verified against code and expanded
here. This register supersedes those two lists as the single reconciliation
artifact and adds R-09 (found during this pass).

**Discrepancy classes:** `dropped-feature` (design describes an assessed-and-not-
adopted mechanism as live) · `unimplemented-claim` (design credits a control the
code does not contain) · `stale-value` (a numeric/formula/description that is
simply wrong vs code) · `framing` (the mechanism is present but described with
the wrong rationale) · `target-change` (integrity target moved).

---

## Register

| ID | Doc / location | Stale claim | Actual code (ground truth) | Class | Severity | Change applied | Applied? |
|---|---|---|---|---|---|---|---|
| **R-01** | `PSTOP_SAFETY_DESIGN.md` §5 (+ §5.1/§5.2/§5.3), §6 fault→reaction map, §7, §9, §10 rows 7–8 | The LFSR/PRNG rolling **challenge** `Cn`, the `S ^= (Cn ^ En)` **liveness accumulator**, and the **stamp echo-signature** are presented as the target/live integrity architecture. | **Option C — DROPPED 2026-07.** None of these exist in the code. Real stuck-at/systematic controls: **Option A** fresh both-phase sample per tick (`main.c:265-308`; `(void)counter` at `:269`) + **Option B** diverse verdict expressions (`:305` arithmetic image vs `:307` boolean) compared by byte `memcmp` (`:890`). Anti-replay = `pstop_c` native counter/`MSG_LOST`/`OUT_OF_ORDER` (F-P-04), not a stamp signature. | dropped-feature | **High** | §5 heading marked **DROPPED 2026-07 (Option C)** with a scope note; §6/§7/§9/§10 given inline reconciled notes mapping each claimed mechanism to the real one; text retained as rationale. | yes |
| **R-02** | `PSTOP_SAFETY_DESIGN.md` §5.5, §6 ("config-integrity check"), §8 row E1, §9 (DC bullet), §10 item 8 | A runtime **GPIO config-integrity check** (periodic re-read/verify of pin direction, mux, pulldown) is credited as an existing/target diagnostic. | **NOT IMPLEMENTED.** Pad config is set **once** at `estop_init` (`main.c:229-247`) and never re-verified at runtime. This is the root of **FMEA DU-1** (lost pulldown → open loop reads closed → false OK, no on-line detection). | unimplemented-claim | **High** | §5.5 heading marked **NOT IMPLEMENTED**; the config-integrity bullet flagged **open gap (FMEA DU-1)**; §9/§10 notes state it is not a shipped control. | yes |
| **R-03** | `PSTOP_SAFETY_DESIGN.md` §5.6 | Heartbeat timeout = `heartbeat_ms × (max_missed_heartbeats + 1)`; "today 1000 × 2 ≈ 1–2 s". | Library math is `heartbeat_ms × max_missed_heartbeats` (`machine.c:290-298`: OK while `diff ≤ hb`; STOP when `diff/hb ≥ max_missed`). Current defaults **400 × 5 ≈ 2.0 s** (SYSTEM_DEFINITION §2; `max_missed` raised 3 → 5 on 2026-08-04, was 400 × 3 ≈ 1.2 s). | stale-value | **High** | Formula and values corrected directly in §5.6 with a code cite; original noted in an HTML comment. | yes |
| **R-04** | `PSTOP_SAFETY_DESIGN.md` §5.4, §8 row D1 | Recommends a **"semantic 1oo2D"** comparator to fix common-cause (G1); implies byte `memcmp` is an interim/insufficient stand-in. | Comparator is **byte-identical `memcmp`** of the two 40-byte encodings (`main.c:890`). Option B is *designed* so healthy cores emit **byte-identical** encodings and any caught fault makes them **differ**, so the byte compare is **sufficient** — the semantic comparator was rendered **unnecessary**, not skipped. Common-cause remains **unquantified** (β≈1 on shared post-selection code; single toolchain/ISA) → needs FMEDA + §10 row-9 fault-injection. | framing | Med | §5.4 given a "Framing correction" reconciled note stating byte-memcmp sufficiency-by-design **and** the do-not-overclaim caveat; §8 D1 covered by the §8 note. | yes |
| **R-05** | `PSTOP_SAFETY_DESIGN.md` — whole doc (Status, Standard-frame line, §9) | Framed as **SIL2** throughout. | Program target is **SIL 3 / PL e** (SYSTEM_DEFINITION; HARA §6). | target-change | Med | Top banner + Standard-frame reconciled note + §9 note flag the target change; the SIL2 rationale is retained as design history (not rewritten). | yes |
| **R-06** | `PSTOP_SAFETY_DESIGN.md` §4 gap G3 | Predictable `counter&1` listed as an open gap "lowering DC", implying the (dropped) LFSR challenge is the fix. | Moot/misleading: the adopted compensating measure is **Option A both-phase** sampling, not an unpredictable challenge. `counter` is ignored (`main.c:269`). | framing | Low–Med | Reconciled note after §4 explains G3 is moot-as-written and maps G1→Option B, G2/G4→dropped-chain residuals tracked in HARA/FMEA. | yes |
| **R-07** | `MULTI_REMOTE_VALIDATION_2026-07-22.md` (logic-under-test bullet) | "a remote goes silent past `heartbeat_ms x (max_missed_heartbeats+1)`". | `heartbeat_ms × max_missed_heartbeats` (`machine.c:290-298`). | stale-value | Med | Formula corrected in place + a short reconciliation banner added near the top pointing here. | yes |
| **R-08** | `host/machine_app_runner.c:721` (code comment) | "the timeout is each operator's `heartbeat_ms x (max_missed_heartbeats + 1)`". | `heartbeat_ms × max_missed_heartbeats` (`machine.c:290-298`; e.g. 400 × 5 ≈ 2.0 s at the current `max_missed` = 5). | stale-value | **High** (safety code comment) | Comment corrected to the library math with a code cite and worked example. | yes |
| **R-09** | `PSTOP_SAFETY_DESIGN.md` §2 "Rolling drive level" bullet | "Each tick the owning core drives its OUT pin with the tick counter's LSB … Until both phases have been sampled the channel reads OPEN" — i.e. **one phase per tick, two ticks to a verdict**. | **Superseded.** Code samples **both phases every tick** (drive HIGH→read→drive LOW→read in one call, `main.c:274-279`) and **ignores the counter** (`:269`). This *is* what Option A ("fresh both-phase sample every tick") refers to; the rolling-LSB text is the predecessor design. | stale-value | Med | §2 bullet marked *(superseded)* with a reconciled note describing both-phase-per-tick; original retained as history. | yes |

---

## Precision note on the two "×(max_missed)" thresholds (not a contradiction)

The corrected formula (R-03/R-07/R-08) is the **heartbeat *liveness* timeout** —
silence → STOP the robot — in `machine_check_heartbeats` (`machine.c:290-298`):
`missed = silence/heartbeat_ms`, STOP when `missed ≥ max_missed_heartbeats` ⇒
timeout `= heartbeat_ms × max_missed_heartbeats`. This is what every corrected
passage means by "heartbeat timeout".

A **different** library function, `check_timestamp` (`protocol.c:38-65`),
computes `PSTOP_MSG_LOST` from the *echoed* `received_stamp` ack-lag and uses
`missed ≥ max_missed_heartbeats + 1`. That is the black-channel bounded-loss
window on the counter/stamp round-trip, **not** the liveness timeout — its `+1`
is correct and intentional and was **not** changed (both are inside the
pre-qualified `pstop_c` boundary and out of scope for modification). `pstop_c`
is untouched. `host/README.md` §"Liveness timeout" already documents this
distinction correctly.

## Docs reviewed and found consistent (no edit needed)

| Doc | Verdict |
|---|---|
| `docs/safety/SYSTEM_DEFINITION.md` | Authoritative backbone — reflects Option A+B / C-dropped, `hb×max_missed`, SIL 3 / PL e. **Updated 2026-08-04:** `max_missed` 3 → 5, so timeout is now 400×5≈2.0 s (was 400×3≈1.2 s). |
| `docs/safety/HARA.md`, `docs/safety/FMEA.md` | Seed findings; already code-accurate. Their D1–D6 / §6 lists are subsumed by this register. No change. |
| `docs/MACHINE_ROS2_NODE_DESIGN.md` | Uses `max_missed` correctly; example config still shows defaults 400/3/500 (§4, §12). No stale heartbeat formula, no Option C, no SIL2. **Note (2026-08-04):** the machn (safety-credited, A-07) default was raised to `max_missed` = 5; the ROS 2 node's own compiled default was **not** confirmed changed by that commit (machn-specific), so its example still reads 3. **Open reconciliation item:** confirm whether the ROS 2 node default should track machn (=5) and update the example + code accordingly, or document the two paths intentionally differ. 5 is within the ROS 2 validator envelope [1,5]. |
| `docs/MACHINE_ESP32_DESIGN.md` | Series-relay + feedback actuator description matches design intent; no stale safety claim. |
| `docs/SAFETY_CHAIN.md` | Reboot/rollback recovery only; correctly states machine holds STOP on silence. No verdict/heartbeat/Option-C claims. |
| `docs/FAILOVER_AND_ARMING_DESIGN_2026-07-21.md` | Arming policy + debounce match the code (`min_stop_ms=500`, single-tick STOP, 3-tick reclose). No stale claim. |
| `docs/TESTING.md`, `docs/MISRA_COMPLIANCE_2026-07-21.md` | Procedural; no stale safety formula or dropped-feature claim. |
| `host/README.md` §"Liveness timeout" | **Already correct** — states `heartbeat_ms × max_missed_heartbeats` and explicitly "not `× (max_missed + 1)`", with the `max_missed=1` false-STOP caveat. No change. |
| `docs/archive/*` (ESTOP_SAFETY_DESIGN, ESTOP_SAFETY_DECISIONS, transport/chaos reports) | **Historical / superseded** — intentionally left unedited. They contain the same stale `hb×(max_missed+1)` formula and Option-C language, but archival status is their reconciliation. |
| `docs/safety/coverage/*.html` | **Generated** gcov artifacts; the stale `:721` comment appears only as a rendering of the source that R-08 fixed. Regenerate to refresh; not hand-edited. |

## Items left "needs human decision"

**None.** Every discrepancy above is a clear factual correction verifiable
against the cited code. Two dispositions are recommended (not deferred) and
flagged for the safety lead's awareness:

1. **`PSTOP_SAFETY_DESIGN.md` §10 rows 7–8** test dropped/unimplemented features
   (stamp-signature replay; config-integrity). They were **re-scoped in place**
   (row 7 → `pstop_c` counter/`MSG_LOST` anti-replay; row 8 → kept as a
   *gap-demonstration* of DU-1 until the check exists) rather than removed.
2. **Common-cause sufficiency of Option B** (R-04) is asserted as *sufficient by
   design* for the comparator, **but explicitly not quantified** — it remains a
   SIL 3 evidence gate (FMEDA + §10 row-9 fault-injection) per HARA §6/§7 and
   FMEA DU-3. No claim was strengthened beyond the code.
