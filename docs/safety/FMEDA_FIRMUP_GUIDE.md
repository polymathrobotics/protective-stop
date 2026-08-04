<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Firming Up the pstop FMEDA Without Manufacturer Failure-Rate Data

> **How to read this document.** It is dense with functional-safety acronyms and
> symbols. Every acronym is spelled out at first use, and there is a full reference at
> the end: **Appendix A** (acronym glossary with explanations), **Appendix B** (symbols
> and notation), and **Appendix C** (the harder concepts in plain language). If a term
> is unfamiliar, jump to the appendix — each entry is a few sentences, not just a
> definition.

**Purpose.** A **Failure Modes, Effects and Diagnostic Analysis (FMEDA)** is the
component-by-component spreadsheet that predicts how often a safety device fails, and in
which direction (safe vs dangerous). Our current one (`docs/safety/FMEDA.md`) reports a
**Safe Failure Fraction (SFF)** of ≈ 93 %, a dangerous-failure frequency
(**Probability of dangerous Failure per Hour, PFH**) of ≈ 3.7 × 10⁻⁹ per hour, and a
**common-cause failure fraction (β, "beta")** of 10 % — enough to claim **Safety
Integrity Level 3 (SIL 3)** but "met but fragile," and built on **assumed** component
numbers. This guide is the playbook for turning those assumptions into an
**assessor-defensible** FMEDA when you **cannot get component makers to supply
failure-rate (λ, "lambda") data** — Espressif publishes no **Failures-In-Time (FIT,
failures per 10⁹ hours)** or **Mean Time Between Failures (MTBF)** for the ESP32-S3
microcontroller, and most relay and passive-component vendors won't give a
dangerous-failure breakdown.

It was produced from two **IEC 61508** advisor passes (data-sourcing +
method/defensibility). *IEC 61508* is the International Electrotechnical Commission's
umbrella standard for the functional safety of electrical/electronic safety systems.
The guide is tailored to the pstop architecture: **Remote** = a dual-core ESP32-S3
reading a 2-channel **emergency-stop (E-stop)** loop through its **General-Purpose
Input/Output (GPIO)** pins; **Machine** = an ESP32-S3 microcontroller (**MCU**) driving
two **series** safety relays with feedback. The voting architecture is **one-out-of-two
with diagnostics (1oo2D)** and the **Hardware Fault Tolerance (HFT)** is 1 (it survives
one hardware fault), in **high/continuous demand** (the safety function is in constant
use — see Appendix C).

> **Honesty up front.** Everything below is *generic / handbook-derived*. None of it
> substitutes for vendor qualification data. The defensible position is **not** "we
> found the real numbers" — it is "we used a *named, dated, condition-matched,
> conservatively-adjusted, sensitivity-tested* number and can audit every one." A
> certification body / assessor (for example **TÜV**, the German technical-inspection
> association; **exida** or **Sira**, functional-safety certifiers) accepts handbook
> rates on exactly those terms.

---

## 0. The two things an assessor actually checks

1. **Random hardware failure ≠ systematic failure.** Generic rates and this FMEDA
   address only *random hardware* failure — parts wearing out or failing by chance
   (the architectural-constraint and PFH gates below). They do **nothing** for
   **Systematic Capability level 3 (SC 3)** — freedom from design/software mistakes,
   which is proved separately (the "Route 1S/2S/3S" systematic routes of IEC 61508-2,
   and IEC 61508-3 for software; see Appendix C). Keep them apart; a good FMEDA must not
   be allowed to *imply* systematic compliance.
2. **Two independent hardware gates, both must pass.** (a) **Architectural
   constraints** — the Safe Failure Fraction and Hardware Fault Tolerance together cap
   the achievable SIL (IEC 61508-2 §7.4.4, Tables 2 & 3). (b) **Quantified target** —
   the PFH must fall in the SIL-3 band (IEC 61508-2 §7.4.5). For pstop these are
   *different* tests: **our PFH is strong; our exposure is the SFF architectural gate**
   (see §2).

---

## 1. The firm-up playbook (do these in order)

1. **Freeze the FMEDA component list** — every part in the safety path (both ESP32-S3
   dies, the GPIO loop network, both series relays + feedback, power conditioning,
   oscillators).
2. **Pick one primary rate source and cite its edition** (recommendation in §3). Use a
   *consistent family* of sources; justify any per-part deviation.
3. **For each component**: base failure rate λ → **failure-mode split** (what fraction
   of failures are safe vs dangerous) → apply **π-factors** (pi-factors — the
   stress-adjustment multipliers that scale a rate to your real operating conditions,
   §4) → **Diagnostic Coverage (DC)** decides how much of the dangerous rate is detected
   (DD) vs undetected (DU) (§5).
4. **Relays**: run the **ISO 13849 B10d / MTTFd** route (§6) — the machinery-standard
   wear-out calculation — as an independent cross-check on the constant-rate relay
   numbers; set the replacement/proof-test interval from it.
5. **Common-cause fraction β via IEC 61508-6 Annex D** — a completed scoring
   questionnaire, with no phantom diversity credit (§7).
6. **Build the Assumptions Register** (§8) — the central defensibility artifact; one
   row per rate.
7. **Sensitivity analysis** (§9) — show the SFF stays ≥ 90 % (for the complex "Type B"
   channel) and the PFH stays < 10⁻⁷ per hour across the handbook uncertainty band
   (×3 to ×10).
8. **Add field evidence** where you can (§10) — "prior-use / proven-in-use" data to move
   from *assumed* toward *demonstrated*.
9. **Self-audit against the assessor checklist** (§11) before submission.

---

## 2. Architecture & metric — where pstop is strong vs fragile

Every element is classified **Type A** (simple — all failure modes well understood, with
field data) or **Type B** (complex — contains something whose failure behaviour isn't
fully known; any microcontroller is Type B). The SFF and HFT then cap the SIL via
IEC 61508-2 Tables 2 & 3 ("Route 1H", the hardware route). See Appendix C for Type A/B.

| Element | Type | HFT | SFF band needed to license SIL 3 |
|---|---|---|---|
| ESP32-S3 (Remote cores, Machine MCU) | **B** (complex) | 1 | **SFF 90–<99 %** (Table 3) |
| Series relays + feedback | **A** (well-defined modes, field data) | 1 | **SFF 60–<90 %** (Table 2) |

**Consequence.** The ESP32-S3 is **Type B**, so at HFT = 1 you need **SFF ≥ 90 %** to
license SIL 3 — and the current 93 % sits *just* inside that band. That boundary is the
whole "fragile": if an assessor reclassifies even one failure mode you called
Dangerous-Detected (DD) as Dangerous-Undetected (DU), the SFF can drop below 90 % and
demote the Type-B channel to SIL 2. **Protecting the 90 % boundary is priority #1.**
Argue the **relays as Type A explicitly** (well-known modes, field data, defined fault
behaviour) — they then only need SFF 60–<90 %, a far more comfortable margin.

**Demand mode = high/continuous → the metric is PFH.** An operator can hit the E-stop at
any moment, so demand is not below once per year and not tied to the proof-test schedule
(IEC 61508-4 §3.5.16 defines the modes). Therefore:

- Target band: **SIL 3 continuous = PFH between 10⁻⁸ and 10⁻⁷ per hour** (IEC 61508-1
  Table 3). Our 3.7 × 10⁻⁹ per hour is comfortably inside — **the quantitative gate is
  not the problem.**
- **The proof-test interval T1 barely helps in high demand.** In the low-demand metric
  (average Probability of Failure on Demand, **PFDavg**), T1 scales the dominant
  ½·λ_DU·T1 term; in the high-demand metric (PFH) the residual is **β·λ_DU** — a *rate*
  that is almost independent of T1. So don't lean on "proof-test more often" to rescue
  PFH — lean on **β** and on converting **DU → DD** with diagnostics.

**The 1oo2D high-demand approximation you must internalise (IEC 61508-6 Annex B.3.3):**

```
PFH_1oo2D  ≈  2·(1−β)²·λ_DU²·t_CE   +   β·λ_DU
                (negligible)              (dominant)
```

Here **λ_DU** is the dangerous-undetected failure rate per channel, **β** the
common-cause fraction, and **t_CE** the "channel-equivalent down-time" (a small
mean-outage term — see Appendix B). For realistic λ_DU the squared term vanishes and
**PFH ≈ β·λ_DU**. Everything that decides "SIL 3 by a comfortable margin vs by a
whisker" is (a) how small **λ_DU** is (→ diagnostics) and (b) how small and defensible
**β** is. This is the strategic heart of the whole exercise.

---

## 3. Where to get failure rates without a vendor (data sources)

These are recognized reliability-prediction handbooks and databases you can cite in
place of vendor data. Each entry names what it is best used for and its known bias.

| Source | Edition | Use it for | Bias / pedigree |
|---|---|---|---|
| **Siemens Norm SN 29500** (used with **IEC 61709:2017**) | 29500-1/-2/-4/-5; 61709 Ed. 3 | **Primary base rates** for the ESP32-S3 die + passive parts; the π-factor conversion engine | Tool-supported, well-structured; **optimistic** (rates read low). Reference conditions: 40 °C, sheltered (environment class C of IEC 60654-1). |
| **exida Safety Equipment Reliability Handbook (SERH), 4th ed.** + the SILStat field-data program | 4th | **Relays + the safe/dangerous split** — its numbers are *already* broken into the four safety buckets (λ_SD/λ_SU/λ_DD/λ_DU) in FMEDA form | Functional-safety pedigree (built for de-energize-to-trip logic). The same device can vary ~10× depending on data method — record which data set and duty you used. |
| **Quanterion Failure Mode/Mechanism Distributions (FMD-2016)** | 2016 | **Failure-mode distributions** — the safe-vs-dangerous apportionment (e.g. what fraction of relay failures are contact weld) | Empirical field data; the go-to source when a handbook gives a total rate but no mode split. |
| **FIDES Guide 2009/2022** (published in France as **UTE C 80-811**) | 2009/2022 | **Mobile-robot / rugged-environment** penalty (mission profile, thermal cycling, a process-quality factor Π_Process of 0.5–4) | Physics-of-failure + lifecycle model; more representative than SN 29500's "sheltered" reference for a moving vehicle. |
| **US Military Handbook 217F, Notice 2 (MIL-HDBK-217F N2)** | 1995 | **Environment π-factors** — the clean, citable Ground-Benign→Ground-Mobile ("G_B → G_M") mobility penalty; connector/relay/PCB models | Frozen since 1995, **pessimistic** for silicon (fine for a conservative safety argument); officially discouraged for new integrated-circuit work. |
| **Quanterion Nonelectronic Parts Reliability Data (NPRD-2016)** | 2016 | Field rates for **electromechanical/mechanical** parts (relays, connectors, switches) | Field data across ground/airborne/naval environments. |
| **Telcordia SR-332 Issue 4** | 2016 | Alternative electronic base rates; can blend lab + field data | Telecom-lineage, optimistic for harsh environments. |

**Recommended sourcing strategy for pstop:** use SN 29500 (with IEC 61709) for the
ESP32-S3 electronics; overlay FIDES / MIL-HDBK-217F Ground-Mobile factors to justify the
mobile penalty; use exida SERH + FMD-2016 for the relays and the dangerous-mode split;
cross-check the relays with ISO 13849 B10d (§6). Cite editions everywhere; never write
"industry typical."

---

## 4. Applying π-factors — the worked conversion (do this for every dominant line)

A **π-factor** (pi-factor) is a multiplier that adjusts a handbook's base failure rate
(quoted at some standard "reference condition") to your real operating condition —
temperature, electrical stress, environment, part quality, etc.

```
λ_operating = λ_ref × π_T × π_U × π_E × π_Q × π_(other)
   λ_ref = base rate at the handbook's reference condition
   π_T = temperature factor,  π_U = voltage,  π_E = environment,  π_Q = quality

Temperature factor uses the Arrhenius law (failure rate rises exponentially with heat):
  π_T = exp[ (E_a / k) · (1/T_ref − 1/T_op) ]
   E_a = activation energy (eV), material/mechanism-specific
   k   = Boltzmann's constant = 8.617×10⁻⁵ eV/K
   T_ref, T_op = reference and operating temperature, in kelvin
   Typical E_a: a complex integrated circuit ≈ 0.3–0.7 eV; passive parts ≈ 0.1–0.15 eV
```

**Worked example — the ESP32-S3 die (this is why "SIL 3 met" looks fragile):**

| Factor | Value | Basis |
|---|---|---|
| λ_ref | 60 FIT @ 40 °C | SN 29500-2 die model (assumed) |
| π_T (temperature) | **≈ 6.45** | E_a = 0.4 eV, operating 85 °C vs reference 40 °C |
| π_E (environment) | ≈ 3 | mobile (MIL-HDBK-217F Ground-Benign→Ground-Mobile) — not sheltered |
| π_Q (quality) | ≈ 2 | commercial-grade part, not automotive-qualified (AEC-Q100) |
| **λ applied** | **≈ 2320 FIT** | 60 × 6.45 × 3 × 2 |

The 40× jump from 60 to ~2320 FIT is exactly the trap: an FMEDA that quietly uses
*reference-condition* rates reads "SIL 3" but collapses under honest operating
conditions. Two legitimate levers pull it back down: **run the die cooler** (every
10–15 °C roughly halves π_T) and **diagnostics** (move that rate into the
dangerous-*detected* bucket λ_DD, where it barely touches PFH). **Show this conversion
explicitly in the FMEDA for every dominant line** — undocumented reference-condition
rates are the single most common generic-rate finding by assessors.

---

## 5. Diagnostic Coverage without vendor data — claim it from IEC 61508

**Diagnostic Coverage (DC)** is the fraction of a component's *dangerous* failures that
an on-line self-test detects (and reacts to) before they can cause harm. A "diagnostic"
here is any automatic check the system runs on itself (see Appendix C). You do **not**
need vendor DC numbers — the standard lets you claim DC from its own catalogue of
techniques, then justify it with your FMEDA reasoning.

- **Three tiers:** Low = 60 %, Medium = 90 %, High = 99 % (IEC 61508-2).
- **Where the maximum claimable DC lives:** **IEC 61508-2 Annex A** tables list, per
  element type (processing unit, program/data memory, input/output, program-sequence
  /watchdog monitoring, clock, sensors, final elements), the highest DC you may claim
  for each technique. **IEC 61508-7 Annex A** describes each technique and its
  effectiveness. **IEC 61508-2 Annex C** is the normative calculation:
  **DC = Σλ_DD / Σλ_D** and **SFF = (Σλ_S + Σλ_DD) / (Σλ_S + Σλ_D)** (Σ = "sum of"; λ_S
  = safe rate, λ_D = total dangerous rate). A worked example is in IEC 61508-6 Annex C.
  *(Verify the exact Annex A table numbering against your licensed 2010 edition before
  citing a specific "Table A.n" — the element-type→table mapping is what you cite.)*

**pstop diagnostics → defensible DC claims:**

| Diagnostic (already in the design) | Technique | DC claim | Justification / cap |
|---|---|---|---|
| Both-phase 2-channel E-stop sampling | redundant-input cross-monitor | **90 %**; 99 % only if dynamically pulse-tested | catches single-channel stuck, cross-short, short-to-supply |
| Dual-core compare (Remote) | reciprocal comparison by software | **90 %** | identical cores → **no** common-cause / shared-clock credit; do not claim 99 % |
| **Relay feedback readback** (Machine) | output / final-element monitoring | **99 % if force-guided (mechanically-linked) contacts, else 90 %** | this is what converts the relay-weld dangerous rate λ_D into detected λ_DD; weld detection needs the mirror contact mechanically forced (see Appendix C) |
| Heartbeat / windowed watchdog | program-sequence, temporal + logical | simple = **60 %**; **windowed + logical + independent timebase = 90 %** | a bare "kick the watchdog" only catches gross hangs |
| GPIO pad re-verify (requirement SR-R-09) | input/output readback / plausibility | **60–90 %** | catches a stuck driver; no common-cause credit for drive + sense sharing one die |
| Clock cross-check (requirement SR-H-04b) | clock monitoring | **90 %**; High only with a *truly independent* 2nd oscillator | checking against the same crystal is weak; against the independent tick / external watchdog timebase it is Medium-High |

Rules assessors apply — bake them in: **identical redundancy caps DC at Medium (90 %)
and pushes the residual into β**; a diagnostic only counts if it acts **within the
process safety time (PST)** and forces the safe state (document the diagnostic test
interval and the fault reaction). Then split **λ_DD = DC·λ_D** and
**λ_DU = (1−DC)·λ_D** — the sum of λ_DU across the design drives PFH.

---

## 6. Relays — the ISO 13849 B10d cross-check (relays wear out; a constant rate hides it)

ISO 13849 is the machinery functional-safety standard. Because relay contacts wear with
use, a constant-failure-rate model under-represents them; ISO 13849 models wear-out from
a cycle count instead.

```
n_op   = (d_op · h_op · 3600) / t_cycle      operations per year
   d_op    = operating days per year
   h_op    = operating hours per day
   t_cycle = seconds between demands
MTTFd  = B10d / (0.1 · n_op)                 Mean Time To dangerous Failure, in years (cap 100)
   B10d = cycles at which 10 % of samples have failed DANGEROUSLY
If only B10 is known (10 % failed by ANY mode), ISO 13849-1 permits B10d = 2·B10
   (i.e. assume half of failures are dangerous)
```

**Mean Time To dangerous Failure (MTTFd)** bands: Low 3–<10 years · Medium 10–<30 years
· High 30–100 years. Typical **B10d**: a power relay/contactor ≈ 1.3–2 × 10⁶ cycles; a
signal relay up to ~2 × 10⁷ (de-rate hard near the contact's rated load).
**Worked:** switch every 60 s, 16 h/day, 300 days/yr, B10d = 2 × 10⁶ →
n_op = 288 000/yr → MTTFd = 2 × 10⁶ / (0.1 × 288 000) = **69 years (High).**

**But an E-stop relay is usually *idle*, not cycling** — so **weld/stiction/corrosion
wear-out**, not cycle count, governs. A relay that never operates can weld silently;
the **feedback readback + a periodic forced test** is what keeps λ_DU bounded. ISO
13849-1 Annex E gives "cross-monitoring + feedback of relays" a DC of **99 %**, which
corroborates the §5 relay-feedback claim. **Reconcile the two routes:** if the B10d
calculation yields a much worse number than the constant-rate route, the constant-rate
number is hiding wear-out — trust B10d and set the **replacement interval** from it.

---

## 7. Common-cause failure fraction β via IEC 61508-6 Annex D — no phantom diversity

A **common-cause failure (CCF)** is one root cause that takes out both redundant
channels at once (a shared power glitch, a shared firmware bug, a shared thermal event).
**β ("beta")** is the fraction of a channel's failures that are common-cause — and, per
§2, in a 1oo2D high-demand system it dominates PFH.

IEC 61508-6 Annex D is a **scored questionnaire** (~37 items grouped as: separation,
diversity, complexity, assessment, procedures, competence, environmental control +
testing). Each "yes" adds an **X** score (the part whose effectiveness is *improved by
diagnostics*, amplified by a "Z" factor tied to DC and diagnostic interval) and a **Y**
score (diagnostic-independent). The total **S** is looked up in **Table D.4 → β**; a
separate score **S_D → β_D** gives the smaller common-cause fraction for *detected*
failures. *(The Table D.4 score→β cut-offs are proprietary — reproduce them from a
licensed copy of the standard.)*

**Defending pstop's β:**

- **Two identical ESP32-S3 cores running identical firmware = the textbook
  low-diversity, high-CCF case.** You earn **little or no diversity credit** — identical
  cores fail together under the same transient, clock glitch, thermal excursion, or
  firmware defect. **Do not claim diversity you don't have**; an assessor strikes it and
  then distrusts the whole register. **Hold β = 10 % on the cores** — that is the
  method's conservative default ceiling and is essentially unimpeachable.
- **Where you *can* lower β legitimately** (these load the X term, and they are real on
  our design): separation (independent power rails; physical/thermal separation of the
  two relays and their drive), strong on-line diagnostics (lockstep compare, watchdogs,
  relay readback), and documented electromagnetic-compatibility (EMC) and thermal
  qualification.
- **Highest-leverage engineering change available:** because **PFH ≈ β·λ_DU**, the
  single biggest win is **relay diversity** — if the two series relays are *different
  types / manufacturers*, you can argue **β = 5 %** (or lower) on the dominant
  final-element channel with a completed Annex D worksheet, roughly halving PFH. Two
  identical relays → hold β = 10 % there too. Always compute **β_D separately** (it feeds
  the detected-failure terms in the 1oo2D formula).

**Deliverable:** the completed Annex D questionnaire — item-by-item evidence, the X/Y
sums, the Z input, and the Table D.4 lookup — attached to the FMEDA. β must never appear
as a bare "10 %" with no worksheet behind it.

---

## 8. The Assumptions Register — the central defensibility artifact

One row per generic rate. This is the document that turns "we used a database number"
into an auditable strength:

| # | Field | Why the assessor needs it |
|---|---|---|
| 1 | Component / failure mode | trace to the FMEDA line |
| 2 | λ used | the number entering the roll-up |
| 3 | **Source + edition/version + table/page** | e.g. "SN 29500-2:2010 Tbl x"; "exida SERH 4th"; "FMD-2016" — named & dated, never "typical" |
| 4 | Source reference conditions | the temperature (θ_ref), duty, stress and quality the base rate assumes |
| 5 | **π-factors applied** | π_T / π_E / π_Q / … each with its value + the clause/equation defining it |
| 6 | Applied conditions | your actual operating temperature (θ_op), environment class (e.g. Ground-Mobile), duty — matched to (4) |
| 7 | Failure-mode-split source | where the safe/dangerous ratio came from (FMD-2016 / the source's own split) |
| 8 | Rationale | one line: why this source applies to this part |
| 9 | **Conservatism note** | *which direction is it pessimistic, and by how much* (e.g. "θ 85 °C assumed vs 60 °C measured → λ overstated ≈ 1.9×") |
| 10 | Sensitivity link | cross-reference to the §9 case that varies this λ |
| 11 | Owner / date / status | change control |

Rules that make it audit-ready: **every rate condition-matched** (a mismatched
reference condition is the #1 finding); **every rate conservative on the record**
(column 9 is your currency when you lack vendor data); **one consistent source family**
or a per-part justification; and **state the handbook uncertainty yourself** (IEC 61508-7
notes handbook rates carry a factor of ×3 to ×10) to pre-empt the assessor.

---

## 9. Uncertainty & conservatism — the load-bearing exhibit

Because handbook rates carry ×3-to-×10 uncertainty, no assessor accepts a single point
estimate that clears SIL 3 by only 5 %. **You must prove robustness:**

- **Sensitivity analysis.** Identify the dominant contributors (for a 1oo2D high-demand
  system: **β and the λ_DU of the relay channel**, per §2). Vary each **×3 and ×10**;
  move a claimed-DD mode to DU (DC 90 → 80 %); sweep β across its Annex D range; take
  the worst-case temperature. Present a table: for each perturbation recompute the
  **SFF (does it stay ≥ 90 % for the Type-B channel? ≥ 60 % for the Type-A relay?)** and
  the **PFH (does it stay < 10⁻⁷ per hour?)**. The claim you are proving is **"SIL 3
  holds across the credible uncertainty envelope,"** not "the nominal is 3.7 × 10⁻⁹." If
  SIL 3 breaks under a ×3 rate or a DC→80 % move, fix it *before* submission (relay
  diversity, better diagnostics, a cooler die).
- **Conservative rounding:** round λ_DU **up**, DC and SFF **down**, β **up**; take the
  upper end of any range; use the worst-case temperature (max ambient + self-heat).
  Record every rounding in register column 9.
- **Bound the DU terms explicitly:** since PFH ≈ β·λ_DU, put an upper bound on the sum of
  λ_DU and show that β · λ_DU(upper) is still < 10⁻⁷ per hour. A bounded worst case that
  passes beats a precise nominal.

---

## 10. From *assumed* toward *demonstrated* — prior-use / proven-in-use

Generic rates are an assumption; the strongest FMEDA adds field evidence. "Proven-in-use"
(IEC 61508-2 §7.4.10) is the standard's route for crediting an accumulated operating
history in place of, or alongside, predicted rates.

- **Requirements:** a documented, stable operating history; an **unchanged
  design/firmware** over the window (any change resets the clock); an operating
  environment that matches the target; and **field failures systematically recorded and
  classified safe/dangerous** — a real fault-reporting system, not anecdote.
- **How much:** the assessor expects the accumulated device-hour count to statistically
  support the target at a stated confidence — a common benchmark is **> 10⁷ device-hours
  (~1000 device-years)** across a sufficient installed base (**≥ ~100 units**) with a
  dangerous-failure record consistent with the claimed λ.
- **How hours become a rate (a chi-squared, χ², upper confidence bound — *not* a naïve
  failures ÷ hours):**
  `λ_a = χ²(a; 2k+2) / (2T)`, where **k** = number of dangerous failures observed,
  **T** = accumulated device-hours, and **a** = the confidence level. With **k = 0**
  failures this reduces to `λ = −ln(1−a) / T`. Assessors expect a = 70–90 %.
  **Zero failures does not mean λ = 0** — it is an upper bound *only if T is large
  enough*; low/zero counts on thin exposure are worthless.
- **Combine:** carry the **more conservative** of {handbook value, χ²-field bound} into
  the FMEDA. Showing that the field bound is *no worse* than the generic assumption is
  powerful corroboration.
- **pstop opportunity:** the fleet already logs check-ins. Instrument it to accumulate
  **per-unit operating hours + a classified fault log** now — that is how, in a year, you
  convert the relay/MCU assumptions into a proven-in-use argument, and (via **Route 2H**,
  IEC 61508-2 §7.4.4.3 — the field-data route to Hardware Fault Tolerance) potentially
  sidestep the fragile Type-B SFF gate with field-reliability evidence.

---

## 11. Pre-submission self-audit (what a TÜV / exida / kenexis reviewer asks for)

(*kenexis* is another functional-safety engineering consultancy.)

**Architecture:** Type A/B decided per block and justified (ESP32-S3 = B, relays = A) ·
HFT + voting + the compliance Route (1H/2H) declared, with the exact Table 2/3 row that
licenses SIL 3 · demand mode determined (→ metric = PFH). **FMEDA:** a complete
component × failure-mode table, with mode splits sourced · every λ_DD backed by a named
diagnostic + its coverage + a test interval ≤ the process safety time · SFF, average
Diagnostic Coverage (DCavg) and PFH reproducible from the raw sheet · **no SFF padding
with "no-effect" failures**. **Generic-rate defensibility:** an Assumptions Register with
source + edition + reference conditions + π-factors shown for **every** rate · a single
source family or a per-part justification · reference conditions matched, worst-case
temperature used · handbook uncertainty acknowledged. **Common cause:** a completed Annex
D worksheet → β and β_D · no unearned diversity credit. **Robustness:** a sensitivity
table showing SFF ≥ 90 % and PFH < 10⁻⁷ per hour across λ×3/×10, DC decreasing, β
increasing · the dominant term (β·λ_DU) explicitly bounded. **Evidence & lifecycle:** any
prior-use data with the §7.4.10 conditions met and a χ² confidence stated · **Systematic
Capability demonstrated separately** (not conflated with the FMEDA) · a safety manual
stating the proof-test procedure, the Proof-Test Coverage (PTC), the proof-test interval
T1, the Mean Time To Restoration (MTTR), the assumed environment, and every condition of
safe use.

---

## 12. The pstop action list (in priority order)

1. **Protect the SFF-90 % boundary.** Re-examine every mode you called Dangerous-Detected
   on the ESP32-S3 channel; make sure each has a real diagnostic (§5) with a coverage
   you would defend under challenge. This is the fragile gate.
2. **Introduce relay diversity** (two *different* relay types/manufacturers) — the single
   highest-leverage change; it roughly halves PFH by licensing β < 10 % on the dominant
   channel (§7). If the relays are already diverse, document it and claim the credit.
3. **Build the Assumptions Register** (§8) with SN 29500 + exida SERH + FMD-2016 +
   FIDES/MIL-HDBK-217F sources, editions, π-factor conversions, and conservatism notes.
4. **Run and table the sensitivity analysis** (§9) — this is the exhibit that converts
   "fragile" into "robust across uncertainty."
5. **Complete the Annex D β worksheet** (§7) — kill any phantom diversity; hold β = 10 %
   on identical elements.
6. **Stand up the field-hours + classified-fault log** on the fleet now (§10) so
   proven-in-use becomes available in ~a year (and opens Route 2H as an escape from the
   Type-B SFF gate).
7. **Keep Systematic-Capability (SC 3) evidence separate** — the MISRA C coding standard,
   static analysis in Continuous Integration (CI), the unit + fault-injection tests, and
   the traceability this effort produced.

---

## Caveats carried from the advisor research (do not publish these unverified)

- The exact **IEC 61508-2 Annex A table titles/numbering (A.1–A.15)** — cite the
  element-type→table mapping, but confirm the specific "Table A.n" against a licensed
  2010 edition before quoting it.
- The **IEC 61508-6 Annex D Table D.4** score→β cut-offs and any device-hour-vs-SIL
  confidence tables are proprietary tabular content — populate from a licensed copy, not
  from secondary paraphrase.
- The 1oo2 / 1oo2D **Annex B formulas** are the widely-published forms; verify them
  term-by-term against your licensed Annex B (edition-specific t_CE / t_GE conventions
  vary) before quoting in a certification submission.
- The **FIT figures in §3–§4 are order-of-magnitude, generic**, from secondary sources
  (exida blog, automotive targets, FMD mode logic, standard reference conditions) —
  starting points to be replaced by sourced register entries, not final values.

---

# Appendix A — Acronym glossary

Grouped by theme; within each group, roughly in order of first appearance.

## Analysis methods & documents
- **FMEDA — Failure Modes, Effects and Diagnostic Analysis.** The core spreadsheet of
  this whole effort: list every component, every way it can fail, how often, whether the
  failure is safe or dangerous, and whether a diagnostic catches it. Everything else here
  feeds or checks the FMEDA.
- **FMEA — Failure Modes and Effects Analysis.** The qualitative predecessor of FMEDA
  (no diagnostic/quantitative columns); FMEDA = FMEA + failure rates + diagnostic
  coverage.
- **HARA — Hazard Analysis and Risk Assessment.** (Referenced elsewhere in the safety
  set.) Determines *what* SIL/PL a function must reach; the FMEDA then shows whether the
  hardware *achieves* it.

### Standards & bodies
- **IEC — International Electrotechnical Commission.** Publisher of IEC 61508, the base
  functional-safety standard. Its parts referenced here: -1 (general/SIL bands), -2
  (hardware, architectural constraints, proven-in-use), -3 (software), -4 (definitions),
  -6 (calculation formulas + Annex D β), -7 (technique catalogue + uncertainty).
- **ISO — International Organization for Standardization.** Publisher of ISO 13849, the
  machinery functional-safety standard, used here for the relay B10d/MTTFd cross-check.
- **SN — Siemens Norm.** A Siemens corporate standard; SN 29500 is the de-facto default
  source of component base failure rates.
- **UTE — Union Technique de l'Électricité.** French electrotechnical standards body; UTE
  C 80-811 is the French-standard designation of the FIDES Guide.
- **TR — Technical Report.** IEC/TR 62380 was a reliability-data report, now folded into
  IEC 61709.
- **TÜV — Technischer Überwachungsverein.** German technical-inspection organisation; a
  common functional-safety certification body ("an assessor").
- **exida / Sira / kenexis.** Functional-safety certification / consulting firms (proper
  names, not acronyms) — "the assessor" in this document.
- **SINTEF.** A Norwegian research institute; author of the PDS method and β-factor
  report cited for common-cause data.
- **PDS.** SINTEF's reliability-analysis method for computer-based safety systems (from
  the Norwegian *Pålitelighet av Datamaskinbaserte Sikkerhetssystemer*).
- **CALCE — Center for Advanced Life Cycle Engineering** (University of Maryland); cited
  for its assessment of the FIDES method.

### Integrity levels & architecture
- **SIL — Safety Integrity Level** (1–4). The required reliability of a safety function;
  SIL 3 is our target. Higher = more reliable = harder to prove.
- **PL — Performance Level** (a–e). The ISO 13849 machinery-standard equivalent of SIL;
  PL e ≈ SIL 3.
- **SC — Systematic Capability** (1–4). How well design/software mistakes were prevented
  — proved by *process* (reviews, testing, coding standards), separately from the
  FMEDA's random-hardware numbers. SC 3 is our target.
- **HFT — Hardware Fault Tolerance.** How many hardware faults the function survives and
  still performs. HFT = 1 means one fault is tolerated (we have two channels).
- **1oo2D — "one-out-of-two with diagnostics."** A voting architecture: either of two
  channels can command the safe state (so it fails safe easily), and on-line diagnostics
  detect a broken channel and remove it. The "D" is what makes it *1oo2D* rather than
  plain 1oo2.
- **Type A / Type B element.** A hardware classification: Type A = simple, all failure
  modes understood, field data exists (our relays); Type B = complex, contains something
  whose failure behaviour isn't fully known (any microcontroller). Type B needs a higher
  SFF for the same SIL — the root of our "fragile" gate. (See Appendix C.)
- **Route 1H / 2H.** The two hardware routes to satisfying architectural constraints in
  IEC 61508-2: 1H uses SFF+HFT tables; 2H uses field-reliability data with confidence
  bounds.
- **Route 1S / 2S / 3S.** The three routes to Systematic Capability (prior-use,
  compliant development, or proven-in-use of the element).

### Failure-rate & reliability metrics
- **λ (lambda) — failure rate.** How often a part fails; usually in FIT.
- **FIT — Failures In Time.** 1 FIT = 1 failure per 10⁹ device-hours. The working unit
  for λ.
- **MTBF — Mean Time Between Failures.** The reciprocal-style summary of λ; ~1/λ for
  repairable items. Espressif publishes neither FIT nor MTBF for the ESP32-S3.
- **λ_S, λ_DD, λ_DU, λ_D — the four safety buckets.** Safe; Dangerous-Detected;
  Dangerous-Undetected; and total Dangerous (λ_D = λ_DD + λ_DU). Also seen as **λ_SD /
  λ_SU** (safe-detected / safe-undetected). λ_DU is the killer term.
- **SFF — Safe Failure Fraction** = (λ_S + λ_DD) / (λ_S + λ_D). The fraction of failures
  that are either safe or caught by a diagnostic. Our fragile number (~93 %, needs
  ≥ 90 %).
- **DC — Diagnostic Coverage** = λ_DD / λ_D. The fraction of *dangerous* failures a
  diagnostic detects. **DCavg** is the λ-weighted average across an element.
- **PFH — Probability (average frequency) of dangerous Failure per Hour.** The
  high-demand target metric; SIL 3 = 10⁻⁸ to 10⁻⁷ per hour. Ours ≈ 3.7 × 10⁻⁹ (good).
- **PFDavg — average Probability of Failure on Demand.** The *low-demand* metric (not
  ours); scales with the proof-test interval T1.
- **β (beta) — common-cause failure fraction.** Fraction of failures that hit both
  channels from one root cause; **β_D** is the smaller value for *detected* failures.
  Dominates PFH here (PFH ≈ β·λ_DU).
- **CCF — Common-Cause Failure.** The failure β measures — one cause, both channels.
- **B10 / B10d.** Cycle counts at which 10 % of samples have failed (B10 = any mode;
  B10d = *dangerously*). The wear-out input for relays.
- **MTTFd — Mean Time To dangerous Failure** (years). Derived from B10d and the operating
  cycle count; banded Low/Medium/High in ISO 13849.
- **MTTR — Mean Time To Restoration** (a.k.a. repair time); **MRT — Mean Repair Time.**
  How long a detected fault stays present; feeds the small t_CE/t_GE outage terms.
- **PST — Process Safety Time.** The maximum time the system may take to reach the safe
  state after a fault/demand; a diagnostic only counts if it reacts within the PST.
- **PTC — Proof-Test Coverage.** The fraction of dangerous-undetected failures a manual
  proof test actually reveals (rarely 100 %).
- **T1 — proof-test interval.** How often the manual proof test runs.

### Data sources
- **SERH — Safety Equipment Reliability Handbook** (exida). Failure rates already split
  into the four safety buckets.
- **FMD — Failure Mode/Mechanism Distributions** (Quanterion FMD-2016). The safe/dangerous
  *mode split* source.
- **NPRD / EPRD — Nonelectronic / Electronic Parts Reliability Data** (Quanterion). Field
  failure rates for mechanical/electromechanical and electronic parts respectively.
- **FIDES.** A French aerospace/defence reliability-prediction guide (physics-of-failure +
  lifecycle); strong for mobile/rugged use. (Name evokes Latin *fides*, "trust"; not a
  strict acronym.)
- **MIL-HDBK-217F — US Military Handbook 217F.** The venerable, pessimistic prediction
  handbook; used here mainly for environment factors.
- **G_B / G_M — Ground-Benign / Ground-Mobile.** MIL-HDBK-217 environment classes; the
  G_B→G_M ratio is a citable "mobile vehicle" penalty.

### Components & engineering
- **ESP32-S3.** The Espressif dual-core microcontroller used for both the Remote and the
  Machine.
- **MCU — Microcontroller Unit.**
- **IC — Integrated Circuit.**
- **PCB — Printed Circuit Board.**
- **GPIO — General-Purpose Input/Output.** The microcontroller pins that read the E-stop
  loop.
- **E-stop — Emergency stop / protective stop.** The safety function itself.
- **EMC — Electromagnetic Compatibility.** Immunity to electrical interference; a
  documented EMC qualification earns common-cause (β) credit.
- **AEC-Q100.** The Automotive Electronics Council's stress-qualification standard for
  ICs; a "commercial-grade" (non-AEC-Q100) part carries a higher quality π-factor.
- **ASIL — Automotive Safety Integrity Level.** The automotive (ISO 26262) analogue of
  SIL; cited only for reference FIT targets.
- **MISRA — Motor Industry Software Reliability Association.** Its C coding standard is
  part of our Systematic-Capability evidence.
- **CI — Continuous Integration.** Automated build/test on every change; hosts the static
  analysis that supports SC 3.
- **SR-R-09 / SR-H-04b.** Internal pstop safety-requirement IDs (Remote GPIO re-verify;
  machine-side clock guard).

---

# Appendix B — Symbols & notation

| Symbol | Read as | Meaning |
|---|---|---|
| **λ** | "lambda" | Failure rate (failures per hour, usually in FIT). |
| **λ_S, λ_DD, λ_DU, λ_D** | lambda-S / -D-D / -D-U / -D | Safe / Dangerous-Detected / Dangerous-Undetected / total-Dangerous failure rate. |
| **β** | "beta" | Common-cause failure fraction (0–1); **β_D** = the value for detected failures. |
| **π** | "pi" | A stress-adjustment multiplier. **π_T** temperature, **π_U** voltage, **π_E** environment, **π_Q** quality, **π_S** switching, **π_L** load; **Π_Process** = FIDES development-quality factor. |
| **Σ** | "sigma" / "sum of" | Summation over all components (e.g. Σλ_DU = total dangerous-undetected rate). |
| **θ** | "theta" | Temperature. **θ_ref** = handbook reference temp; **θ_op** = your operating temp. |
| **E_a** | "E-sub-a" | Activation energy (electron-volts, eV) in the Arrhenius temperature model. |
| **k** | — | Boltzmann's constant, 8.617 × 10⁻⁵ eV/K (in the Arrhenius term). *Separately*, **k** also denotes the *number of observed failures* in the §10 chi-squared formula — context distinguishes them. |
| **T** | — | In §10, accumulated device-hours. *Separately*, **T1** = the proof-test interval. |
| **t_CE, t_GE** | — | "Channel-equivalent" and "group-equivalent" down-time: small mean-outage terms (from λ_DU, λ_DD, MTTR/MRT and T1) in the IEC 61508-6 Annex B formulas. |
| **n_op** | — | Relay operations per year (ISO 13849 B10d route). |
| **χ²** | "chi-squared" | A statistical distribution; used to turn observed failures + device-hours into an upper-confidence-bound failure rate. |
| **≈ / × / →** | approximately / times / leads-to | e.g. "PFH ≈ β·λ_DU"; "×3" = a factor of three; "DU → DD" = converting undetected to detected. |
| **10⁻⁷ /h** | ten-to-the-minus-seven per hour | A frequency; the SIL-3 PFH ceiling in high demand is 10⁻⁷ per hour. |

---

# Appendix C — Key concepts in plain language

- **Random vs systematic failure.** *Random* = physical parts failing by chance or
  wear (what the FMEDA and failure rates cover). *Systematic* = a design or software
  mistake that is baked in and would recur (covered by process, not by rates). The
  standard demands both be handled, by *separate* arguments; a strong FMEDA says nothing
  about systematic quality.

- **The two hardware gates.** To claim SIL 3 the hardware must pass **two** independent
  tests: (1) **architectural constraints** — a table lookup on SFF and HFT that *caps*
  the SIL your architecture is *allowed* to claim; and (2) the **quantified target** —
  the computed PFH must land in the SIL-3 band. You can ace one and fail the other. For
  pstop, the PFH is comfortably good, but the architectural gate (needing SFF ≥ 90 % on
  the complex Type-B microcontroller channel) is what sits on a knife-edge.

- **Type A vs Type B.** A "simple" element (Type A) has fully understood failure modes
  and field history — you're allowed to reach a given SIL at a *lower* SFF. A "complex"
  element (Type B) — anything with a microcontroller — could fail in ways nobody has
  characterised, so the standard demands a *higher* SFF for the same SIL. That extra
  demand is the whole reason our SFF margin is "fragile."

- **Demand mode (low vs high/continuous).** *Low demand* = the safety function is called
  on rarely (< once/year), so what matters is the chance it is broken *when finally
  asked* (PFDavg). *High/continuous demand* = it is effectively always working, so what
  matters is the *rate* at which it fails dangerously (PFH). An E-stop can be pressed any
  time → high/continuous → PFH, and the proof-test interval barely moves PFH.

- **Safe vs dangerous failure; detected vs undetected.** A *safe* failure drives the
  system to its safe state (relays open, robot stops) — a nuisance, not a hazard. A
  *dangerous* failure could leave the system running when it should stop. A dangerous
  failure is *detected* if an automatic diagnostic notices it and forces the safe state
  in time; otherwise it is *undetected* — and undetected dangerous failures (λ_DU) are
  what actually cause accidents. Nearly every lever in this guide is about shrinking
  λ_DU.

- **Diagnostic.** Any automatic self-check the system runs while operating — comparing
  the two cores, reading the relay feedback contact, a watchdog timer, re-reading a pin's
  configuration. Diagnostic *Coverage* is how much of the dangerous failure rate those
  checks catch. A diagnostic only counts if it also *reacts* (forces the safe state)
  within the process safety time.

- **Proof test.** A *manual*, periodic test (unlike a diagnostic, which is automatic and
  continuous) that reveals dangerous-undetected failures that have accumulated — e.g.
  physically exercising a relay to confirm it still opens. Its "coverage" (PTC) is rarely
  100 %.

- **Common cause (β).** Redundancy only helps if the two channels fail *independently*.
  A common cause — one shared power rail, one firmware bug, one hot enclosure — defeats
  redundancy by taking out both at once. β is the fraction of failures that do this, and
  in a redundant high-demand system it becomes the dominant term of PFH. **Diversity**
  (making the two channels genuinely different) is the main way to lower β; claiming
  diversity you don't actually have (two identical chips) is the classic assessor
  red flag.

- **Reference conditions & π-factors.** A handbook quotes a base failure rate at some
  standard "reference" temperature/stress/environment. Your part runs hotter, in a
  moving vehicle, at commercial grade — all of which raise the true rate. π-factors are
  the multipliers that convert the reference rate to your condition (the worked example
  in §4 shows a 60 FIT reference becoming ~2320 FIT applied). Using reference numbers
  without this conversion is the single most common way an FMEDA silently overstates
  SIL.

- **Force-guided (mechanically-linked) relay contacts.** A relay whose monitor contact
  is *mechanically tied* to the main contact, so if the main contact welds shut, the
  monitor contact is physically prevented from indicating "open." This is what lets the
  feedback readback *reliably* detect a welded relay (a dangerous failure) and claim
  99 % diagnostic coverage; without mechanical linkage the monitor can lie and the claim
  drops to 90 %.

- **Proven-in-use / prior-use.** Crediting a large accumulated field history (many units
  × long time, with a real failure-logging system) as evidence the failure rate is at
  least as good as claimed — a way to back up, or replace, a predicted number with
  measured reality. Only valid while the design is unchanged and the environment matches.

- **Sensitivity analysis.** Deliberately re-running the numbers with each dominant input
  made worse (rate ×3 or ×10, coverage lower, β higher, hotter) to prove the SIL claim
  survives the uncertainty rather than depending on optimistic point values. It is the
  exhibit that turns "SIL 3, barely" into "SIL 3, robustly."

- **"Assessor" and "defensible."** The assessor is the third-party reviewer (TÜV/exida/
  Sira/kenexis) who decides whether to certify. "Defensible" means every number is
  traceable to a named, dated source, condition-matched, deliberately conservative, and
  shown to survive uncertainty — the standard this whole guide is written to meet.
