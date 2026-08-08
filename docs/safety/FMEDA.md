<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Protective-Stop — FMEDA (quantitative, first draft)

**Status:** Engineering first draft for safety-lead curation. This document
turns the qualitative **FMEA** (`docs/safety/FMEA.md`) and its
**Dangerous-Undetected register (DU-1..DU-9)** into a **quantitative FMEDA**
(λ apportionment → SFF / DC / PFH / β) for the SIL 3 / PL e claim the
**HARA** (`docs/safety/HARA.md` §6) flags as *allocated, not achieved*. It
consumes the architecture, function IDs, and diagnostics defined in
`docs/safety/SYSTEM_DEFINITION.md` (**1oo2D-to-stop / 2oo2-to-run, HFT = 1**,
10 Hz diagnostic tick, Option A both-phase sampling, Option B diverse
expressions, lockstep comparator, machine heartbeat, `machn` dual series relay
+ feedback, CRC-16 black channel).

> **⚠ No manufacturer datasheet failure rates were available.** Every base
> failure rate (λ) below is derived from a **published reliability standard or
> a published FMEDA analogue, cited inline**. Where a value is an engineering
> assumption pending a real part number, it is tagged **`[ASSUMPTION]`** and
> the closest published anchor is cited. §8 collects the sensitivity of the SIL
> result to each. **No number in this document is un-cited.**

**Standard frame:** IEC 61508-2 (random-hardware integrity, SFF, architectural
constraints) + IEC 61508-6 (PFH formulas, β) + ISO 13849-1 (actuation-side PL,
B10d, proof test). Safe state = **STOP = de-energized**. Inherited fail-danger
polarity `OK == 0x00` (SYSTEM_DEFINITION §5).

---

## 1. Method & assumptions

### 1.1 Reliability model

+ **Random-hardware only.** Per IEC 61508 and the TI white paper *SLOA294A*
  (§2, "systematic failures are excluded from the calculation of random hardware
  failure metrics"), this FMEDA quantifies **random** hardware failures only.
  Systematic faults — the dominant residual here (DU-3 compiler-erased diversity,
  DU-4 config, DU-5 encode, DU-7 comparator logic) — are **not** given a λ; they
  are governed by the **systematic-capability (SC 3)** argument in HARA §6.3 and
  are carried in §7 as the reason the numeric PFH is *necessary but not
  sufficient*.
+ **Constant failure rate** (exponential, useful-life region of the bathtub
  curve) — TI *SLOA294A* §3.
+ **Series safety chain.** For the STOP function the remote sensing subsystem
  **and** the machine actuation subsystem are in *functional series* (a
  dangerous-undetected fault in either defeats the stop), so subsystem
  λ_DU and PFH **add** at system level.
+ **Per subsystem: 1oo2D**, evaluated with the IEC 61508-6:2010 Annex B
  formulas (§5.3).

### 1.2 Mission profile (stated + cited)

| Parameter | Value | Basis / citation |
|---|---|---|
| Environment | Sheltered mobile-vehicle enclosure ≈ **IEC 60654-1 class C** | Adopted from the exida R. STAHL FMEDA (*STAHL 04/04-03 R004*, §Management summary), the closest published SN 29500 analogue |
| Reference temperature | **40 °C** average (base rates); **60 °C** as a ×2.5 stress sensitivity case | exida R. STAHL FMEDA: "failure rates valid … 40 °C … for 60 °C multiply by an experience-based factor of **2.5**" |
| Duty | Powered continuously while the vehicle operates; treated as **24/7 continuous** | TI *SLOA294A* §7 recommended industrial usage profile ("always on 24/7") |
| Demand mode | **High-demand / continuous** → metric is **PFH** | IEC 61508-1; HARA A-04 (unattended autonomy) |
| Diagnostic test interval | **0.1 s** (10 Hz tick) | SYSTEM_DEFINITION §2; `machn/main/main.c:76` `TICK_MS` |
| Proof-test interval **T1** | **1 year (8760 h)** `[ASSUMPTION]` — for latent actuation faults (relay double-weld) | ISO 13849-1 proof-test concept; value is an integrator/commissioning parameter (FMEA §5, SAFETY_REQUIREMENTS SR-SYS-05) |
| Mean repair/restore time (MTTR/MRT) | **8 h** `[ASSUMPTION]` | IEC 61508-6 Annex B default order; fleet-serviceable device |

### 1.3 Failure-rate databases chosen (and why)

| Database | Used for | Why | Citation |
|---|---|---|---|
| **Siemens Norm SN 29500** (parts -1/-2/-3/-4/-5) | Primary — ICs, discrete semis, passives, connectors | Widely accepted for functional-safety BFR; look-up reference FIT + stress conversion; the basis of the cited exida analogue | SN 29500-1:2004 … -5:2004 (editions per exida R. STAHL FMEDA §2.3 [N2]); method per TI *SLOA294A* §5 |
| **IEC TR 62380:2004** | Cross-check for the MCU die (adds package/solder-joint terms SN 29500 omits) | SN 29500 is "deficient … in accounting for silicon-package interaction" | TI *SLOA294A* §6, §9 |
| **MIL-HDBK-217F Notice 2** | Passives / switch / connector sanity-check | Public, reproducible base rates | e.g. RM chip-film resistor λ_b = **0.0037 /10⁶ h** (ROHM *R1102A*, MIL-HDBK-217F) |
| **ISO 13849-1 / EN IEC 61810-2-1** | Electromechanical relay (wear-driven) | Relay reliability is cycle-driven (B10d), not purely time-driven | ISO 13849-1 Annex C; Omron G7SA B10d = **4.0×10⁵** ops (Omron *Reliability Data for Safety of Machinery*) |

**Type classification:** every programmable/complex element (ESP32-S3 MCU,
both remote and machine) is **Type B** ("complex", not all failure modes well
defined) per IEC 61508-2 §7.4.4.1.3. This selects the **Type B** column of the
architectural-constraint table (§7). The relays and passives are Type A but are
integrated into a Type-B-led subsystem.

### 1.4 Failure-mode split default

Where no part-specific distribution exists, the **50 % dangerous** default is
used: ISO 13849-1 (and common FMEDA practice) — *"in the absence of detailed
information, consider 50 % of failures as dangerous."* (ISO 13849-1; ReeR /
TE Connectivity safety-relay guidance). Detected/undetected within the
dangerous fraction is then set by the **diagnostic coverage** claimed in §4,
whose ceiling is fixed by the **FMEA DU register**.

---

## 2. Part / function inventory & base failure rates (λ at 40 °C)

All λ in **FIT** (failures / 10⁹ h). Safe/Dangerous split per §3; detected split
per §4. **Every λ cites a source**; `[ASSUMPTION]` marks a value pending a real
part number, with its anchor.

### 2.1 Remote sensing/verdict channel — one core (F-R-01/02/04/05)

The remote is **two** such cores (HFT = 1). Values are **per core/channel**.

| # | Element (function) | λ FIT @40 °C | Source / anchor |
|---|---|---|---|
| RC-1 | ESP32-S3 core logic + local RAM (F-R-02 verdict) | **40** `[ASSUMPTION]` | SN 29500-2 digital IC; anchored to the ISO 26262 device budget (ASIL-B ≤ 100 FIT, ASIL-D ≤ 10 FIT — TI *SLOA294A* Table 2-1) apportioned across 2 cores; cross-checked vs AURIX SN 29500-2 base (Infineon KBA-236335) |
| RC-2 | Safety GPIO drive+sense pads **+ pull-down** (F-R-01) | **4** `[ASSUMPTION]` | SN 29500-2 (on-die pad/config cell + SEU on the config register); this cell carries **DU-1** (lost pull-down) |
| RC-3 | DPST pole + loop wiring + connector (F-R-01) | **8** `[ASSUMPTION]` | MIL-HDBK-217F switch + connector models (order 1–10 FIT); contact-stuck-closed is the dangerous mode |
| RC-4 | Clock / crystal (F-R-02 fresh-sample timing) | **8** `[ASSUMPTION]` | SN 29500-4 quartz crystal (order 5–10 FIT); frozen/off-frequency clock → stale sample (**DU-8**) |
| — | **Per-channel total λ** | **60** | rounded working figure |

### 2.2 Machine actuation channel — `machn`, one of two series relays (F-M / SG-5)

The machine is **two** such channels, relays **wired in series** with per-contact
feedback (HFT = 1, 1oo2D — `machn/main/main.c:19-27,65-74,303,313-357`). Values
**per channel**.

| # | Element (function) | λ FIT @40 °C | Source / anchor |
|---|---|---|---|
| MC-1 | ESP32-S3 machine core (decode + decide, F-M-03) | **40** `[ASSUMPTION]` | as RC-1 |
| MC-2 | Electromechanical relay, per contact (final element) | **30** `[ASSUMPTION]` | Wear/cycle-driven; ISO 13849-1 B10d method, Omron G7SA B10d = 4.0×10⁵ ops; time-FIT bounded by MIL-HDBK-217F relay model. **Dominated by cycling, not calendar time** — see §8. Weld/stuck-closed = dangerous |
| MC-3 | Feedback resistor divider (2×, per contact) | **2** | MIL-HDBK-217F RM chip-film λ_b 0.0037/10⁶ h ×πT(1.2)×πQ(3) ≈ 0.5–2 FIT each (ROHM *R1102A*) |
| MC-4 | Machine time base `g_tick_now_ms` (single latched clock, F-P-03) | **8** `[ASSUMPTION]` | SN 29500-4 crystal; **carries DU-2** (frozen clock). One latched value feeds both cores (`machn/main/main.c:194,426`) → **β ≈ 1** (§6) |
| — | **Per-channel total λ** | **80** | rounded working figure |

### 2.3 Network / black-channel elements (F-R-06/07, F-P-01/04)

Treated as an **untrusted black channel** (HARA A-08; IEC 61508-2 §7.4.11;
IEC 62280 principles). The transport is **not** assigned a λ in the SFF roll-up;
instead its residual undetected-corruption probability is bounded by the
endpoint **CRC-16** (`pstop_c` over bytes [0..37]):

+ Residual undetected-error fraction of a 16-bit CRC ≈ **2⁻¹⁶ ≈ 1.5×10⁻⁵**
  (upper bound, IEC 61508-2 Annex A / IEC 62280 residual-error argument).
+ Combined with per-session src IP/port filtering (`main.c:741`) and the
  `pstop_c` counter/stamp echo + `MSG_LOST`/`OUT_OF_ORDER` (F-P-04), **all**
  identified corruption/replay effects tend to STOP (fail-safe; FMEA R06-x).
+ **Contribution to λ_DU: treated as negligible** vs the endpoint hardware,
  contingent on the black-channel assumption being a stated safety requirement
  (SR-SYS-08). This is a *qualitative* discharge, flagged in §8.

---

## 3. Failure-mode split (Safe / Dangerous) per part

Split rationale cited per row. "S" = tends toward the de-energized safe state;
"D" = tends away (toward spurious RUN or a defeated stop). Polarity anchor:
sense inputs pulled **down**, comparator **transmit-on-agreement**, machine
**default-STOP** (SYSTEM_DEFINITION §5) — these bias most modes **Safe**.

| Element | λ FIT | Safe % | λ_S | Dang % | λ_D | Split source |
|---|---|---|---|---|---|---|
| RC-1 MCU core | 40 | 50 | 20 | 50 | 20 | ISO 13849-1 50 % default |
| RC-2 GPIO+pull-down | 4 | 25 | 1 | 75 | 3 | Pull-down loss reads **false-closed → OK** = dangerous-biased (FMEA R01-3); `[ASSUMPTION]` |
| RC-3 DPST+wiring | 8 | 62 | 5 | 38 | 3 | Open-circuit (the common wiring fault) → STOP = safe; only stuck-closed is dangerous |
| RC-4 clock | 8 | 50 | 4 | 50 | 4 | ISO 13849-1 50 % default |
| **Remote channel** | **60** | | **30** | | **30** | |
| MC-1 machine core | 40 | 50 | 20 | 50 | 20 | 50 % default |
| MC-2 relay (contact) | 30 | 50 | 15 | 50 | 15 | De-energize-to-safe: coil-open/drop-out = safe; **weld/stuck-closed** dangerous. ISO 13849-1 50 % default (conservative; forcibly-guided relays argue < 50 %) |
| MC-3 feedback divider | 2 | 50 | 1 | 50 | 1 | 50 % default |
| MC-4 machine clock | 8 | 62 | 5 | 38 | 3 | Backward/stopped clock disables heartbeat = dangerous (FMEA H03-1); most drift is detectable-safe |
| **Machine channel** | **80** | | **41** | | **39** | |

---

## 4. Diagnostic coverage (DC) — λ_D → λ_DD / λ_DU

DC per dangerous mode, mapped to the **implemented** diagnostic and to the
**FMEA DU register** (which fixes the *undetected* remainder). DC levels use the
IEC 61508-2:2010 **Annex C** bands (Low ≈ 60 %, **Medium ≈ 90 %**, High ≈ 99 %)
and the Annex A technique tables (comparison / dynamic principles).

### 4.1 Remote channel

| Dang mode (λ_D FIT) | Diagnostic (implemented) | DC % (band + rationale) | λ_DD | λ_DU | DU ref |
|---|---|---|---|---|---|
| RC-1 MCU verdict fault (20) | **Option B** diverse expressions → lockstep memcmp diverges (`estop_verdict.c:32-36`, `main.c` comparator); **Option A** live both-phase sample (`estop_verdict.c:8-35`) | **95 %** — High/Medium; comparison of independent channels, IEC 61508-2 Annex A Table A.4 ("comparator"). *Capped below 99 %* by DU-3 (compiler may erase Option B) | 19 | 1 | DU-3 |
| RC-2 pull-down lost → false OK (3) | Single-channel divergence caught by 2oo2-to-run; **BUT common-mode loss undetected** — pad config set once at `estop_init`, never re-verified | **50 %** — Low; the "config-integrity check" claimed in design §6/§10 **does not exist** (FMEA DU-1) | 1.5 | 1.5 | **DU-1**, DU-6 |
| RC-3 DPST/wiring stuck-closed (3) | Both-phase drive/echo detects a stuck level within a tick | **90 %** — Medium; dynamic (both-phase) test, IEC 61508-2 Annex A | 2.7 | 0.3 | — |
| RC-4 clock frozen/off (4) | Option A demands a *fresh matched* sample; a wholly frozen sample path partly evades it | **75 %** — Low/Medium | 3.0 | 1.0 | DU-8 |
| **Remote channel** | | **DC ≈ 89 %** | **26.2** | **3.8** | |

### 4.2 Machine channel

| Dang mode (λ_D FIT) | Diagnostic (implemented) | DC % (band + rationale) | λ_DD | λ_DU | DU ref |
|---|---|---|---|---|---|
| MC-1 machine core (20) | `machn` dual-core RX lockstep: both decode+decide, comparator agrees or withholds reply (`main.c:445-463`) | **95 %** — comparison, Annex A Table A.4 (capped by shared-silicon β, §6) | 19 | 1 | DU-3(analogue) |
| MC-2 relay weld (15) | Per-contact resistor-divider feedback → contradiction ≥ `RELAY_FAULT_STOP_TICKS` (~1 s) forces STOP (`main.c:313-357,486-497`); **series** partner still opens the chain | **90 %** — Medium; output-feedback + 1oo2 series. **Residual = double-weld** (both contacts, commanded-closed) latent to next demand → proof-test governed | 13.5 | 1.5 | FMEA §5 latent weld |
| MC-3 feedback divider (1) | Divider fault shows as a persistent contradiction (self-diagnosing) | **90 %** — Medium | 0.9 | 0.1 | — |
| MC-4 machine clock frozen (3) | ~~**NONE**~~ **[Reconciled 2026-08-07: clock-freeze guard now landed — SR-H-04/SR-H-04b `clock_guard`, DU-2 CLOSED; the 0 %/3.0 below is the stale pre-mitigation value, recompute pending — see §5 banner]** | **0 %** (stale) — was: whole liveness case rests on one clock (FMEA DU-2) | 0 | 3.0 | **DU-2** |
| **Machine channel** | | **DC ≈ 86 %** | **33.4** | **5.6** | |

**Undetected remainder ↔ FMEA DU register.** The λ_DU cells above are exactly
the DU register population: **DU-1** (RC-2, β≈1 as DU-6), **DU-2** (MC-4, β≈1),
**DU-3** (the 95 % cap on both comparators), **DU-8** (RC-4). The systematic
DU-4/5/7 carry no random λ (§1.1) but sit *inside* the 95 %/90 % DC caps as the
reason coverage cannot be claimed at 99 %.

---

## 5. Roll-up — SFF, DC, PFH

> **[Reconciled 2026-08-07 — the numbers below are a CONSERVATIVE pre-mitigation
> snapshot, not yet recomputed.]** This roll-up predates the **DU-1** (SR-R-09
> GPIO pad re-verify) and **DU-2** (SR-H-04 machine clock-freeze guard) code
> mitigations, both now landed and CLOSED (see OPEN_ITEMS / SAFETY_REQUIREMENTS).
> The MC-4 machine-clock row (§4: DC = 0 %, λ_DU = 3.0 — the β≈1 term that
> **dominates the PFH**) and the DU-1 pad-config term are therefore
> stale-pessimistic: with the guards in place those λ_DU move toward λ_DD,
> lifting SFF toward ≥ 99 % and dropping PFH by roughly 5×. **These figures
> UNDERSTATE the achieved integrity.** A re-run crediting the mitigations is
> pending — do it together with the real-part-number firm-up (OPEN_ITEMS §8) so
> it is done once with defensible λ data.

### 5.1 Per-channel and per-subsystem

| Metric | Remote channel | Machine channel |
|---|---|---|
| λ_total | 60 | 80 |
| λ_S | 30 | 41 |
| λ_D | 30 | 39 |
| λ_DD | 26.2 | 33.4 |
| λ_DU | **3.8** | **5.6** |
| **SFF** = (λ_S+λ_DD)/λ_total | **93.7 %** | **93.0 %** |
| **DC** = λ_DD/λ_D | **87.3 %** | **85.6 %** |

**System SFF (λ-weighted across both subsystems):**
SFF = (30+26.2+41+33.4) / (60+80) = 130.6 / 140 = **93.3 %**
**System DC** = (26.2+33.4)/(30+39) = 59.6/69 = **86.4 %**

*Sanity check vs published analogue:* the exida R. STAHL SN 29500 Type-A field
device reports single-channel λ_DD = 150, λ_DU = 58 FIT, **SFF = 72 %** at
40 °C (*STAHL 04/04-03 R004* Table 2). Our per-channel dangerous total (~30–39
FIT) and SFF (~93 %, higher because of the online both-phase/comparison
diagnostics that device lacks) are the **same order of magnitude** — the
inventory is not obviously mis-scaled.

### 5.2 PFH formula (cited)

**IEC 61508-6:2010, Annex B**, high-demand **1oo2** (each subsystem):

```
t_CE  = (λ_DU/λ_D)·(T1/2 + MRT) + (λ_DD/λ_D)·MTTR
PFH_1oo2 ≈ 2·[(1−β)·λ_DU + (1−β_D)·λ_DD]² · t_CE   (residual dual-independent)
           + β·λ_DU  +  β_D·λ_DD                    (common-cause, DOMINANT)
```

For the **1oo2D** variant (IEC 61508-6 Annex B, 1oo2D table) the diagnostic
additionally reconfigures to the healthy channel; the *dominant* term is
unchanged — **β·λ_DU** (common-cause dangerous-undetected). Because the
architecture's diagnostics are strong (DC ≈ 86 %), the independent
dual-failure term is negligible and **PFH is governed by the common-cause
λ_DU**. This is the single most important structural result of the FMEDA.

### 5.3 PFH computation

Constants: T1 = 8760 h, MRT = MTTR = 8 h, β = 0.10, β_D = 0.05 (§6).
λ in /h (FIT ×10⁻⁹).

**Remote subsystem** (λ_DU = 3.8 FIT, λ_DD = 26.2 FIT):
+ Common-cause: β·λ_DU = 0.10 × 3.8e-9 = **3.8×10⁻¹⁰ /h**
+ Residual dual-independent: 2·[(0.9)(3.8e-9)]²·t_CE, t_CE ≈ (3.8/30)(4380+8)+(26.2/30)(8) ≈ 563 h → 2·(3.4e-9)²·563 ≈ **1.3×10⁻¹⁴ /h** (negligible)
+ **PFH_remote ≈ 3.8×10⁻¹⁰ /h**

**Machine subsystem** (λ_DU = 5.6 FIT). Split by β:
+ MC-4 clock (3.0 FIT) is a **single latched time source → β ≈ 1**, so it enters
  at ~full rate: ≈ **3.0×10⁻⁹ /h** (dominant single-point).
+ Remaining λ_DU (2.6 FIT, β = 0.10): β·λ_DU ≈ **2.6×10⁻¹⁰ /h**.
+ Residual dual-independent ≈ 10⁻¹⁴ (negligible).
+ **PFH_machine ≈ 3.3×10⁻⁹ /h**

**System PFH = PFH_remote + PFH_machine ≈ 3.7×10⁻⁹ /h ≈ 4 FIT.**

| Roll-up result | Value |
|---|---|
| **SFF** | **≈ 93 %** |
| **DC** | **≈ 86 %** (Medium) |
| **PFH** | **≈ 3.7×10⁻⁹ /h (≈ 4 FIT)** |
| Dominant contributor | **DU-2** machine clock (β≈1, ≈ 3.0×10⁻⁹/h) |

---

## 6. β — common-cause (IEC 61508-6 Annex D)

β is scored from the Annex D checklist (separation/diversity, complexity,
assessment, procedures, competence, environmental control). IEC 61508-6:2010
Annex D yields β in the range **1 %–10 %** (down to 0.5 % for the best-separated
sensors/final elements; Analog Devices *EngineerZone — How to Quantify Common
Cause Failures*; Ho et al., *The development of a common cause factor score
table on IEC 61508 Part 6 Ed. 2.0*, 2024).

**Scored β = 10 % (worst case) for this architecture**, because the coupling
factors that Annex D scores *against* are all present:

| Annex D factor | This design | Effect on β |
|---|---|---|
| Physical separation | Two cores on **one die**, shared package/power/clock | ↑ (worst) |
| Diversity/redundancy | **Option B** diversifies the *interpretation expression* only (`estop_verdict.c:32-36`) | ↓ **but only for interpretation faults** |
| Complexity/design | Single **shared source tree**, single **toolchain**, identical **Xtensa ISA** | ↑ (worst) — DU-3 |
| Common operational stress | Shared brown-out, ESD, thermal (DU-6 both pull-downs) | ↑ (worst) |

**Interpretation.** Option B genuinely lowers β for a *systematic
misinterpretation of the reads in one expression* (the comparator diverges). It
does **not** lower β for **shared-silicon / shared-toolchain common mode** —
there β stays near the ceiling, and for the specific cases where the shared
element **is** the diagnostic or the clock (comparator memcmp DU-7; machine time
base DU-2) **β ≈ 1** and no HFT credit applies. β_D (detected common cause) is
taken as **5 %**; detected common-cause tends to the announced safe state and is
a minor PFH term.

**Net:** β = **10 %** for the sensing/verdict redundancy; **β ≈ 1** for the
single-point shared functions (machine clock, each comparator). The PFH in §5 is
dominated precisely by the β ≈ 1 terms.

---

## 7. SIL verification

### 7.1 Architectural constraint — IEC 61508-2:2010 Table 3 (Route 1H, Type B)

| SFF band | HFT = 0 | **HFT = 1** | HFT = 2 |
|---|---|---|---|
| < 60 % | — | SIL 1 | SIL 2 |
| 60 – < 90 % | SIL 1 | SIL 2 | SIL 3 |
| **90 – < 99 %** | SIL 2 | **SIL 3** | SIL 4 |
| ≥ 99 % | SIL 3 | SIL 4 | SIL 4 |

With **SFF ≈ 93 %** (90–<99 % band) and **HFT = 1**, Type B →
**SIL 3 is permitted** by the architectural constraint. *Margin is thin:* the
result sits in the middle of the band, and any downward revision of DC (→ SFF
< 90 %) drops the ceiling to **SIL 2**.

### 7.2 PFH vs the SIL 3 band — IEC 61508-1 Table 3

SIL 3 continuous/high-demand band: **10⁻⁸ ≤ PFH < 10⁻⁷ /h** (IEC 61508-1
Table 3; TI *SLOA294A* Table 2-2 restates the SIL 3 target as **SFF ≥ 99 % /
PFH ≤ 10 FIT** at element level). Computed **PFH ≈ 3.7×10⁻⁹ /h** is **below the
10⁻⁷ /h ceiling** — i.e. it **satisfies** the SIL 3 PFH target (and numerically
even exceeds it toward SIL 4).

### 7.3 Verdict

> **On these assumptions, SIL 3 is MET** — SFF ≈ 93 % + HFT = 1 (Type B) permits
> SIL 3 (Table 3), and PFH ≈ 3.7×10⁻⁹ /h clears the SIL 3 band. **But the claim
> is fragile and conditional**, for three reasons that mirror the HARA §6 gate:
>
> 1. **The margin is carried by β ≈ 1 single-point λ_DU** (DU-2 machine clock,
>    DU-7 comparator, DU-6 common-mode pull-down), not by the redundancy. These
>    do **not** benefit from HFT = 1. Fixing them is what earns real margin.
> 2. **Every λ is an assumption** (no datasheet data). §8 shows the result is
>    robust to ~25× error in λ_DU/β before the 10⁻⁷ ceiling is breached — except
>    the common-cause fraction, which for shared silicon + single toolchain is
>    the genuine risk and is only partly quantifiable by β.
> 3. **Systematic capability (SC 3) is a separate gate** (HARA §6.3): DU-3
>    (compiler-erased Option B diversity) is a *systematic* risk with no λ; the
>    numeric PFH does not discharge it. SC 3 process evidence + Option-B
>    fault-injection (SR-R-03) remain mandatory.

### 7.4 Dominant λ_DU contributors → diagnostic roadmap

Ranked by PFH contribution; each maps to an existing **Gap** requirement whose
implementation **moves λ_DU → λ_DD** (raising SFF toward ≥ 99 % and unlocking
SIL 3 with HFT-margin / SIL 4 headroom):

| Rank | λ_DU contributor | ≈ PFH | Fix (moves DU→DD) | Requirement |
|---|---|---|---|---|
| 1 | **DU-2** machine clock frozen (β≈1) | 3.0×10⁻⁹ /h | Independent 2nd time source / external watchdog → DC 0 %→90 % | SR-H-04 / SR-I-03 |
| 2 | **DU-1/DU-6** pull-down lost (common mode) | ~2–3×10⁻¹⁰ /h (amplified by β on the sensing pair) | Periodic pad-config read-back + drive-low-must-read-0 self-test | SR-R-09 / SR-R-10 |
| 3 | **DU-7** comparator memcmp mis-equal (single point) | folded in comparator cap | Per-offset positive-diff self-test | SR-R-08 |
| 4 | **DU-3** Option B erased (systematic, no λ) | — (SC 3 gate) | Object-code diff build-gate / compile-diverse TUs | SR-R-03 |
| 5 | **DU-8** stale/short-settle sample | 1.0×10⁻⁹ contribution to λ_DD/λ_DU | Characterize `ESTOP_SETTLE_US` vs cable RC | SR-R-11 |

**Closing rank 1 alone** (SR-H-04) removes the dominant ~3×10⁻⁹/h term, drops
system PFH to ≈ 7×10⁻¹⁰/h and lifts machine-channel SFF from 93 % toward 97 %.
**Closing ranks 1+2** lifts system SFF ≥ 99 %, giving unambiguous SIL 3 (Route
1H) with HFT-margin.

---

## 8. Sensitivity & caveats — every assumed value flagged

| Assumed value | Used as | Sensitivity of the SIL result |
|---|---|---|
| MCU die λ = 40 FIT/core `[ASSUMPTION]` | RC-1 / MC-1 | Linear on λ_D; even ×3 keeps SFF > 90 % (DC-limited, not λ-limited). **Low** sensitivity to the absolute value; **high** to its DC. |
| Relay λ = 30 FIT + 50 % dangerous `[ASSUMPTION]` | MC-2 | Cycle-driven, not time-driven — **the real driver is actuation frequency vs B10d** (Omron G7SA B10d = 4.0×10⁵). A high-demand vehicle can exhaust B10d well inside the mission; **relay proof-test / replacement interval is the true control**, not the FIT. **High** sensitivity — needs the real part + demand rate. |
| Clock λ = 8 FIT, β ≈ 1 (DU-2) | MC-4 | **Dominant PFH term.** Doubling it doubles system PFH (→ ~7×10⁻⁹, still < 10⁻⁷). Breaches SIL 3 only above ~25×. But it is a **single point with DC = 0 %** — the qualitative risk exceeds the quantitative one. **Highest** priority regardless of the number. |
| β = 10 % (Annex D worst) | sensing CC | Linear on the dominant β·λ_DU term. If shared-silicon common mode is truly worse than 10 % (plausible — one die, one toolchain), PFH rises proportionally. **High** — this is the least defensible input. |
| DC 95 % (comparators), 90 % (both-phase/feedback) | §4 | Sets SFF directly. Dropping the comparator DC to 60 % (if Option B is object-code-identical, DU-3) pushes SFF < 90 % → **architectural ceiling falls to SIL 2**. **Highest structural sensitivity.** |
| Proof-test T1 = 1 yr `[ASSUMPTION]` | relay double-weld latent | Governs the latent-weld term; a longer interval linearly worsens it. Integrator/commissioning parameter (SR-SYS-05). |
| MTTR/MRT = 8 h `[ASSUMPTION]` | t_CE | Negligible (dual-independent term is already negligible). **Very low.** |
| Black channel λ_DU ≈ 0 (CRC-bounded) | §2.3 | Qualitative discharge; valid only if the black-channel integrity assumption is a signed safety requirement (SR-SYS-08). If the residual 2⁻¹⁶ argument is rejected, a λ_DU must be added. **Medium.** |
| 50 % Safe/Dangerous default | §3 | Conservative for de-energize-to-safe parts (real safe fraction is higher). Relaxing it *improves* SFF. **Low (conservative).** |

**Overarching caveat.** This FMEDA is **quantitatively "SIL 3 met" but
qualitatively gated**: the numbers rest on assumed λ and on β = 10 %, and the
margin is held up by β ≈ 1 single-point λ_DU. The result is therefore
**consistent with HARA §6 — "SIL 3 / PL e allocated, not achieved"**: it is
achievable, and the FMEDA shows the *cheapest path* to a robust claim is the DU
roadmap (§7.4), led by the frozen-clock diagnostic (DU-2) and the GPIO pad
re-verify (DU-1). Real part failure rates + the Annex-D β re-score + the
SC 3 / Option-B fault-injection evidence must replace the assumptions before any
external SIL 3 claim.

---

## Sources cited

+ **IEC 61508-1:2010** Table 3 — SIL vs PFH bands (continuous mode).
+ **IEC 61508-2:2010** §7.4.4 (Type A/B), Table 3 (architectural constraints, Route 1H), Annex A/C (diagnostic-coverage technique tables & bands).
+ **IEC 61508-6:2010** Annex B (PFH formulas for 1oo2 / 1oo2D), Annex D (β common-cause scoring).
+ **IEC TR 62380:2004** — reliability data handbook (MCU die/package cross-check).
+ **Siemens Norm SN 29500** (-1:2004, -2:2004, -3:2004, -4:2004, -5:2004) — component reference failure rates (editions per the exida analogue below).
+ **MIL-HDBK-217F Notice 2** — passive/switch/connector base rates; RM chip-film resistor λ_b = 0.0037/10⁶ h (ROHM *R1102A*, <https://fscdn.rohm.com/en/products/databook/operation/passive/resistor/common/r_failure_rate.pdf>).
+ **ISO 13849-1** — 50 % dangerous default; B10d method; proof-test concept.
+ **exida FMEDA — R. STAHL Isolating Repeater 9165** (*STAHL 04/04-03 R004*, V5R0, 2015): SN 29500 @ 40 °C, IEC 60654-1 class C, ×2.5 at 60 °C; single-channel Type A λ_DD 150 / λ_DU 58 FIT, SFF 72 %, PFH 5.8×10⁻⁸/h — <https://r-stahl.com/fileadmin/tx_aimeos/Files/8_/00/STAHL040403R004_008_00/FMEDA_report_STAHL040403R004_008_00.pdf>.
+ **TI SLOA294A** — *Understanding Functional Safety FIT Base Failure Rate Estimates per IEC 62380 and SN 29500* (Rev. A, 2024): SN 29500 method; SIL 3 SFF ≥ 99 % / PFH ≤ 10 FIT (Table 2-2); op-amp λ_ref 12 FIT @ 55 °C (SN 29500-2 Table 4) — <https://www.ti.com/lit/SLOA294A>.
+ **Omron** — *Reliability Data for Safety of Machinery Safety Components*: G7SA safety relay B10d = 4.0×10⁵ ops (forcibly-guided, IEC 61810-3) — <https://www.ia.omron.com/support/sistemalibrary/data/omron_safety_components_en_20240826.pdf>.
+ **Infineon KBA-236335** — AURIX MCU base failure rate SN 29500-2 vs IEC TR 62380 — <https://community.infineon.com/t5/Knowledge-Base-Articles/AURIX-MCU-Base-failure-rate-SN29500-2-vs-IEC-TR-62380/ta-p/373852>.
+ **Analog Devices EngineerZone** — *How to Quantify Common Cause Failures* (IEC 61508-6 Annex D β 1–10 %) — <https://ez.analog.com/b/engineerzone-spotlight/posts/how-to-quantify-common-cause-failures>.
+ **Ho et al. (2024)** — *The development of a common cause factor score table on IEC 61508 Part 6 Ed. 2.0* — <https://www.sciencedirect.com/science/article/abs/pii/S0950423024000287>.

---

*End of first draft. Every λ is an assumption pending real part data (§8); the
SIL 3 result is conditional on those assumptions, on β = 10 %, and on the
separate SC 3 / Option-B systematic case (HARA §6). Curate with the safety lead
and re-run against manufacturer FIT data and the integrator's demand rate before
any external SIL 3 / PL e claim.*
