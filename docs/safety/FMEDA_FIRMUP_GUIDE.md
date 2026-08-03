<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Firming Up the pstop FMEDA Without Manufacturer Failure-Rate Data

**Purpose.** `docs/safety/FMEDA.md` currently reports SFF ≈ 93 %, PFH ≈ 3.7 × 10⁻⁹ /h,
β = 10 % — "SIL 3 met but fragile," on **assumed** component rates. This guide is the
playbook for turning those assumptions into an **assessor-defensible** FMEDA when you
**cannot get component makers to supply λ data** (Espressif publishes no FIT/MTBF for
the ESP32-S3; most relay/passive vendors won't give a dangerous-mode split).

It was produced from two IEC 61508 advisor passes (data-sourcing + method/
defensibility) and is tailored to the pstop architecture: **Remote** = dual-core
ESP32-S3 reading a 2-channel E-stop GPIO loop; **Machine** = ESP32-S3 driving two
**series** safety relays with feedback; **1oo2D, HFT = 1**, high/continuous demand.

> **Honesty up front.** Everything below is *generic / handbook-derived*. None of it
> substitutes for vendor qualification data. The defensible position is **not** "we
> found the real numbers" — it is "we used a *named, dated, condition-matched,
> conservatively-adjusted, sensitivity-tested* number and can audit every one." An
> assessor (TÜV / exida / Sira) accepts handbook rates on exactly those terms.

---

## 0. The two things an assessor actually checks

1. **Random hardware ≠ systematic.** Generic rates + this FMEDA address only *random
   hardware* failure (the architectural-constraint and PFH gates). They do **nothing**
   for **systematic capability (SC 3)** — proved separately (Route 1S/2S/3S, and
   IEC 61508-3 for software). Keep them apart; a good FMEDA must not be allowed to
   *imply* systematic compliance.
2. **Two independent hardware gates, both must pass.** (a) **Architectural
   constraints** — SFF/HFT → SIL (IEC 61508-2 §7.4.4, Tables 2 & 3). (b) **Quantified
   target** — PFH (IEC 61508-2 §7.4.5). For pstop these are *different* tests:
   **our PFH is strong; our exposure is the SFF architectural gate** (see §2).

---

## 1. The firm-up playbook (do these in order)

1. **Freeze the FMEDA component list** — every part in the safety path (both ESP32-S3
   dies, the GPIO loop network, both series relays + feedback, power conditioning,
   oscillators).
2. **Pick one primary rate source and cite its edition** (recommendation in §3). Use a
   *consistent family*; justify any per-part deviation.
3. **For each component**: base λ → **failure-mode split** (safe vs dangerous) → apply
   **π-factors** to your operating conditions (§4) → **DC** decides DD vs DU (§5).
4. **Relays**: run the **ISO 13849 B10d / MTTFd** route (§6) as an independent
   cross-check on the constant-λ relay numbers; set the replacement/proof-test interval
   from it.
5. **β via IEC 61508-6 Annex D** — completed questionnaire, no phantom diversity (§7).
6. **Build the Assumptions Register** (§8) — the central defensibility artifact; one
   row per rate.
7. **Sensitivity analysis** (§9) — show SFF ≥ 90 % (Type-B channel) and PFH < 10⁻⁷ /h
   hold across the handbook uncertainty band (×3–×10).
8. **Add field evidence** where you can (§10) — prior-use / proven-in-use to move from
   *assumed* toward *demonstrated*.
9. **Self-audit against the assessor checklist** (§11) before submission.

---

## 2. Architecture & metric — where pstop is strong vs fragile

**Type A vs B (IEC 61508-2 Tables 2 & 3, Route 1H):**

| Element | Type | HFT | SFF band to license SIL 3 |
|---|---|---|---|
| ESP32-S3 (Remote cores, Machine MCU) | **B** (complex) | 1 | **SFF 90–<99 %** (Table 3) |
| Series relays + feedback | **A** (well-defined modes, field data) | 1 | **SFF 60–<90 %** (Table 2) |

**Consequence.** The ESP32 is **Type B**, so at HFT = 1 you need **SFF ≥ 90 %** to
license SIL 3 — and the current 93 % sits *just* inside that band. That boundary is the
whole "fragile": if an assessor reclassifies even one claimed-DD mode as DU, SFF can
drop below 90 % and demote the Type-B channel to SIL 2. **Protecting the 90 % boundary
is priority #1.** Argue the **relays as Type A explicitly** (well-known modes, field
data, defined fault behaviour) — they then only need SFF 60–<90 %, a far comfier margin.

**Demand mode = high/continuous → metric is PFH.** An operator can hit the E-stop any
time; demand is not < 1/yr and not tied to proof-test frequency (IEC 61508-4 §3.5.16).
So:
- Target band: **SIL 3 continuous = PFH ∈ [10⁻⁸, 10⁻⁷)/h** (IEC 61508-1 Table 3). Our
  3.7 × 10⁻⁹ /h is comfortably inside — **the quantitative gate is not the problem.**
- **Proof-test interval T1 barely helps in high demand.** In PFDavg, T1 scales the
  dominant ½·λ_DU·T1 term; in PFH the residual is **β·λ_DU** (a rate, ~T1-independent).
  Don't lean on "proof-test more often" to rescue PFH — lean on **β** and on converting
  **DU→DD** with diagnostics.

**The 1oo2D high-demand approximation you must internalise (IEC 61508-6 Annex B.3.3):**

```
PFH_1oo2D  ≈  2·(1−β)²·λ_DU²·t_CE   +   β·λ_DU
                (negligible)              (dominant)
```

For realistic λ_DU the quadratic term vanishes and **PFH ≈ β·λ_DU**. Everything that
decides "SIL 3 by margin vs by a whisker" is (a) how small **λ_DU** is (→ diagnostics)
and (b) how small and defensible **β** is. This is the strategic heart of the whole
exercise.

---

## 3. Where to get failure rates without a vendor (data sources)

| Source | Edition | Use it for | Bias / pedigree |
|---|---|---|---|
| **Siemens SN 29500** (via **IEC 61709:2017**) | 29500-1/-2/-4/-5; 61709 Ed.3 | **Primary base rates** for the ESP32 die + passives; the π-factor engine | Tool-supported, well-structured; **optimistic** (rates low). Ref: 40 °C, sheltered (IEC 60654-1 class C). |
| **exida SERH, 4th ed.** + SILStat | 4th | **Relays + the safe/dangerous split** — numbers are *already* λ_SD/λ_SU/λ_DD/λ_DU in FMEDA form | FS pedigree (built for de-energize-to-trip). Same device can vary ~10× by data method — record which set/duty. |
| **Quanterion FMD-2016** | 2016 | **Failure-mode / mechanism distributions** — the safe/dangerous apportionment (e.g. relay weld fraction) | Empirical field data; the go-to when a handbook has no mode split. |
| **FIDES Guide 2009/2022** (= UTE C 80-811) | 2009/2022 | **Mobile-robot / rugged environment** penalty (mission profile, thermal cycling, Π_Process 0.5–4) | Physics-of-failure + lifecycle; more representative than SN 29500's "sheltered" ref for a moving vehicle. |
| **MIL-HDBK-217F Notice 2** | 1995 | **Environment π-factors** (the clean, citable G_B→G_M "mobile" penalty); connectors/relays/PCB | Frozen 1995, **pessimistic** for silicon (fine for a safety argument); officially discouraged for new IC work. |
| **Quanterion NPRD-2016** | 2016 | Field rates for **electromechanical/mechanical** parts (relays, connectors, switches) | Field data across ground/airborne/naval. |
| **Telcordia SR-332 Issue 4** | 2016 | Alternative electronic base rates; can blend lab+field data | Telecom-lineage, optimistic for harsh env. |

**Recommended sourcing strategy for pstop:** SN 29500 (via IEC 61709) for the ESP32
electronics; overlay **FIDES / MIL-217 G_M** to justify the mobile penalty; **exida
SERH + FMD-2016** for the relays and the dangerous-mode split; cross-check relays with
**ISO 13849 B10d** (§6). Cite editions everywhere; never write "industry typical."

---

## 4. Applying π-factors — the worked conversion (do this for every dominant line)

```
λ_operating = λ_ref × π_T × π_U × π_E × π_Q × π_(other)

Arrhenius temperature term:
  π_T = exp[ (E_a / k) · (1/T_ref − 1/T_op) ],  k = 8.617×10⁻⁵ eV/K,  T in kelvin
  E_a from the source table: complex IC ≈ 0.3–0.7 eV; passives ≈ 0.1–0.15 eV
```

**Worked — the ESP32-S3 die (this is why "SIL 3 met" looks fragile):**

| Factor | Value | Basis |
|---|---|---|
| λ_ref | 60 FIT @ 40 °C | SN 29500-2 die model (assumed) |
| π_T | **≈ 6.45** | E_a = 0.4 eV, T_op = 85 °C vs T_ref = 40 °C |
| π_E | ≈ 3 | mobile (MIL-217 G_B→G_M) — not sheltered class-C |
| π_Q | ≈ 2 | commercial-grade, not AEC-Q100 |
| **λ applied** | **≈ 2320 FIT** | 60 × 6.45 × 3 × 2 |

The 40× jump from 60 → ~2320 FIT is exactly the trap: an FMEDA that quietly uses
*reference-condition* rates reads "SIL 3" but collapses under honest conditions. Two
legitimate levers pull it back: **run the die cooler** (every 10–15 °C ≈ halves π_T)
and **diagnostics** (move that λ into λ_DD where it barely touches PFH). **Show this
conversion explicitly in the FMEDA for every dominant line** — undocumented
reference-condition rates are the #1 generic-rate finding.

---

## 5. Diagnostic Coverage without vendor data — claim it from IEC 61508

You do **not** need vendor DC numbers. The standard lets you claim DC from its own
technique catalog, then justify with your FMEDA reasoning.

- **Three tiers:** Low = 60 %, Medium = 90 %, High = 99 % (IEC 61508-2).
- **Where the max claimable DC lives:** **IEC 61508-2 Annex A** tables (by element type
  — processing unit, program/data memory, I/O, program-sequence/watchdog, clock,
  sensors, final elements). **IEC 61508-7 Annex A** describes each technique + its
  effectiveness. **IEC 61508-2 Annex C** is the normative calc: **DC = Σλ_DD / Σλ_D**,
  **SFF = (Σλ_S + Σλ_DD)/(Σλ_S + Σλ_D)**; a worked example is in **IEC 61508-6 Annex C**.
  *(Verify the exact Annex A table numbering against your licensed 2010 edition before
  citing a specific "Table A.n" — the element-type→table mapping is what you cite.)*

**pstop diagnostics → defensible DC claims:**

| Diagnostic (already in the design) | Technique | DC claim | Justification / cap |
|---|---|---|---|
| Both-phase 2-channel E-stop sampling | redundant-input cross-monitor | **90 %**, 99 % only if dynamically pulse-tested | catches single-channel stuck, cross-short, short-to-supply |
| Dual-core compare (Remote) | reciprocal comparison by software | **90 %** | identical cores → **no** common-cause/shared-clock credit; do not claim 99 % |
| **Relay feedback readback** (Machine) | output/final-element monitoring | **99 % if force-guided (mechanically-linked) contacts, else 90 %** | this is what converts relay-weld λ_D → λ_DD; weld detection needs the mirror contact mechanically forced |
| Heartbeat / windowed watchdog | program-sequence, temporal+logical | simple = **60 %**; **windowed + logical + independent timebase = 90 %** | bare "kick" only catches gross hangs |
| GPIO pad re-verify (SR-R-09) | I/O readback / plausibility | **60–90 %** | catches driver stuck-at; no drive/sense common-cause credit on one die |
| Clock cross-check (SR-H-04b) | clock monitoring | **90 %**; High only with a *truly independent* 2nd oscillator | vs the same crystal it's weak; vs the independent tick / external WDT timebase it's Medium-High |

Rules assessors apply — bake them in: **identical redundancy caps DC at Medium and
pushes the residual into β**; a diagnostic counts only if it acts **within the process
safety time** and forces the safe state (document the diagnostic test interval + fault
reaction). Then split **λ_DD = DC·λ_D, λ_DU = (1−DC)·λ_D** — the λ_DU sum drives PFH.

---

## 6. Relays — the ISO 13849 B10d cross-check (relays wear out; constant-λ hides it)

```
n_op   = (d_op · h_op · 3600) / t_cycle          operations per year
MTTF_d = B10d / (0.1 · n_op)                      years   (cap 100)
If only B10 is known, ISO 13849-1 permits  B10d = 2·B10  (assume 50 % dangerous)
```

MTTFd bands: Low 3–<10 y · Medium 10–<30 y · High 30–100 y. Typical **B10d**: power
relay/contactor ≈ 1.3–2 × 10⁶ cycles; signal relay up to ~2 × 10⁷ (de-rate hard near
rated load). **Worked:** switch every 60 s, 16 h/day, 300 d/yr, B10d = 2 × 10⁶ →
n_op = 288 000/yr → MTTFd = 2 × 10⁶/(0.1 × 288 000) = **69 y (High).**

**But an E-stop relay is usually *idle*, not cycling** — so **weld/stiction/corrosion
wear-out**, not cycle count, governs. A relay that never operates can weld silently;
the **feedback readback + a periodic forced test** is what keeps λ_DU bounded. ISO
13849-1 Annex E gives "cross-monitoring + feedback of relays" **DC = 99 %**, which
corroborates the §5 relay-feedback claim. **Reconcile the two routes:** if B10d yields
a much worse number than the constant-λ route, the constant-λ number is hiding wear-out
— trust B10d and set the **replacement interval** from it.

---

## 7. Common-cause β via IEC 61508-6 Annex D — no phantom diversity

Annex D is a **scored questionnaire** (~37 items: separation, diversity, complexity,
assessment, procedures, competence, environmental control + testing). Each "yes" adds
an **X** score (portion *improved by diagnostics*, amplified by a Z factor tied to DC +
diagnostic interval) and a **Y** score (diagnostic-independent). Total **S → Table D.4
→ β**; a separate **S_D → β_D** (< β) for detected CCF. *(Reproduce the Table D.4
score→β cut-offs from a licensed copy — they are proprietary.)*

**Defending pstop's β:**
- **Two identical ESP32-S3 cores, identical firmware = the textbook low-diversity,
  high-CCF case.** You earn **little/no diversity credit** — identical cores fail
  together under the same transient, clock glitch, thermal excursion, or firmware
  defect. **Do not claim diversity you don't have**; an assessor strikes it and then
  distrusts the whole register. **Hold β = 10 % on the cores** — that's the method's
  conservative default ceiling and is essentially unimpeachable.
- **Where you *can* lower β legitimately (loads the X term, and it's real on our
  design):** separation (independent power rails, physical/thermal relay separation),
  strong on-line diagnostics (lockstep, watchdogs, relay readback), and documented
  EMC/thermal qualification.
- **Highest-leverage engineering change available:** because **PFH ≈ β·λ_DU**, the
  single biggest win is **relay diversity** — if the two series relays are *different
  types/manufacturers*, you can argue **β = 5 %** (or lower) on the dominant final-
  element channel with a completed Annex D worksheet, ~halving PFH. Two identical
  relays → hold β = 10 % there too. Always compute **β_D separately** (feeds the DD
  terms in 1oo2D).

**Deliverable:** the completed Annex D questionnaire, item-by-item evidence, X/Y sums,
Z input, Table D.4 lookup — attached. β must never appear as a bare "10 %."

---

## 8. The Assumptions Register — the central defensibility artifact

One row per generic rate. This is what converts "we used a database number" into an
auditable strength:

| # | Field | Why the assessor needs it |
|---|---|---|
| 1 | Component / failure mode | trace to the FMEDA line |
| 2 | λ used | the number entering the roll-up |
| 3 | **Source + edition/version + table/page** | e.g. "SN 29500-2:2010 Tbl x"; "exida SERH 4th"; "FMD-2016" — named & dated, never "typical" |
| 4 | Source reference conditions | θ_ref, duty, stress, quality the base rate assumes |
| 5 | **π-factors applied** | π_T/π_E/π_Q/… each with value + the clause/equation defining it |
| 6 | Applied conditions | your actual θ_op, environment class (GM), duty — matched to (4) |
| 7 | Failure-mode-split source | where the safe/dangerous ratio came from (FMD-2016 / source split) |
| 8 | Rationale | one line: why this source applies to this part |
| 9 | **Conservatism note** | *which direction is it pessimistic, by how much* (e.g. "θ 85 °C assumed vs 60 °C measured → λ overstated ≈1.9×") |
| 10 | Sensitivity link | cross-ref to the §9 case varying this λ |
| 11 | Owner / date / status | change control |

Rules that make it audit-ready: **every rate condition-matched** (mismatched ref
conditions = #1 finding); **every rate conservative on the record** (column 9 is your
currency when you lack vendor data); **one consistent source family** or per-part
justification; and **state the handbook uncertainty yourself** (IEC 61508-7 notes
×3–×10) to pre-empt the assessor.

---

## 9. Uncertainty & conservatism — the load-bearing exhibit

Because handbook rates carry ×3–×10 uncertainty, no assessor accepts a point estimate
that clears SIL 3 by 5 %. **Prove robustness:**

- **Sensitivity analysis.** Identify the dominant contributors (for 1oo2D high-demand:
  **β and λ_DU of the relay channel**, per §2). Vary each **×3 and ×10**; move
  claimed-DD → DU (DC 90 → 80 %); sweep β over its Annex D range; take worst-case θ.
  Present a table: for each perturbation recompute **SFF (stays ≥ 90 % Type-B? ≥ 60 %
  Type-A relay?)** and **PFH (stays < 10⁻⁷ /h?)**. The claim you're proving is
  **"SIL 3 holds across the credible uncertainty envelope,"** not "the nominal is
  3.7 × 10⁻⁹." If SIL 3 breaks under a ×3 λ or a DC → 80 % move, fix it *before*
  submission (relay diversity, better diagnostics, cooler die).
- **Conservative rounding:** round λ_DU **up**, DC/SFF **down**, β **up**; take the
  upper end of any range; worst-case θ (max ambient + self-heat). Every rounding in
  register column 9.
- **Bound the DU terms explicitly:** since PFH ≈ β·λ_DU, put an upper bound on Σλ_DU
  and show β·λ_DU_upper still < 10⁻⁷ /h. A bounded worst-case that passes beats a
  precise nominal.

---

## 10. From *assumed* toward *demonstrated* — prior-use / proven-in-use

Generic rates are an assumption; the strongest FMEDA adds field evidence
(IEC 61508-2 §7.4.10):

- **Requirements:** a documented, stable operating history; **unchanged design/
  firmware** over the window (any change resets the clock); operating environment
  matches the target; and **field failures systematically recorded and classified
  safe/dangerous** — a real fault-reporting system, not anecdote.
- **How much:** the assessor expects the device-hour count to statistically support
  the target at a stated confidence — a common benchmark is **> 10⁷ device-hours
  (~1000 device-years)** across a sufficient installed base (**≥ ~100 units**) with a
  dangerous-failure record consistent with the claimed λ.
- **How hours → rate (χ² upper confidence bound, *not* naïve k/T):**
  `λ_a = χ²(a; 2k+2) / (2T)`; with **k = 0** failures, `λ = −ln(1−a)/T`. Assessors
  expect a = 70–90 %. **Zero failures ≠ λ = 0** — it's an upper bound *only if T is
  large enough*; low/zero counts on thin exposure are worthless.
- **Combine:** carry the **more conservative** of {handbook value, χ²-field bound} into
  the FMEDA. Showing the field bound is *no worse* than the generic assumption is
  powerful corroboration.
- **pstop opportunity:** the fleet already logs check-ins. Instrument it to accumulate
  **per-unit operating hours + a classified fault log** now — that's how, in a year,
  you convert the relay/MCU assumptions into a proven-in-use argument, and (via
  **Route 2H**, IEC 61508-2 §7.4.4.3) potentially sidestep the fragile Type-B SFF gate
  with field-reliability HFT evidence.

---

## 11. Pre-submission self-audit (what a TÜV/exida/kenexis reviewer asks for)

**Architecture:** Type A/B per block justified (ESP32 = B, relays = A) · HFT + voting +
Route (1H/2H) declared with the exact Table 2/3 row licensing SIL 3 · demand mode →
PFH. **FMEDA:** complete component × mode table; mode splits sourced · every λ_DD backed
by a named diagnostic + coverage + test interval ≤ PST · SFF/DCavg/PFH reproducible from
the raw sheet · **no SFF padding with "no-effect" failures.** **Generic-rate
defensibility:** Assumptions Register with source + edition + ref conditions + π-factors
shown for **every** rate · single source family or per-part justification · ref
conditions matched, worst-case θ · handbook uncertainty acknowledged. **Common cause:**
completed Annex D worksheet → β and β_D · no unearned diversity credit. **Robustness:**
sensitivity table showing SFF ≥ 90 % and PFH < 10⁻⁷ /h across λ×3/×10, DC↓, β↑ · dominant
term (β·λ_DU) bounded. **Evidence & lifecycle:** any prior-use with §7.4.10 conditions +
χ² confidence · **systematic capability demonstrated separately** (not conflated) ·
safety manual with proof-test procedure, PTC, T1, MTTR, assumed environment, and all
conditions of safe use.

---

## 12. The pstop action list (in priority order)

1. **Protect the SFF-90 % boundary.** Re-examine every claimed-DD mode on the ESP32
   channel; make sure each has a real diagnostic (§5) with a coverage you'd defend under
   challenge. This is the fragile gate.
2. **Introduce relay diversity** (two *different* relay types/manufacturers) — the
   single highest-leverage change, ~halves PFH by licensing β < 10 % on the dominant
   channel (§7). If already diverse, document it and claim the credit.
3. **Build the Assumptions Register** (§8) with SN 29500 + exida SERH + FMD-2016 +
   FIDES/MIL-217 sources, editions, π-conversions, and conservatism notes.
4. **Run and table the sensitivity analysis** (§9) — this is the exhibit that converts
   "fragile" into "robust across uncertainty."
5. **Complete the Annex D β worksheet** (§7) — kill any phantom diversity; hold β = 10 %
   on identical elements.
6. **Stand up the field-hours + classified-fault log** on the fleet now (§10) so
   proven-in-use becomes available in ~a year (and opens Route 2H as an escape from the
   Type-B SFF gate).
7. **Keep systematic-capability (SC 3) evidence separate** — MISRA, static analysis in
   CI, the unit + fault-injection tests, and the traceability this effort produced.

---

## Caveats carried from the advisor research (do not publish these unverified)

- The exact **IEC 61508-2 Annex A table titles/numbering (A.1–A.15)** — cite the
  element-type→table mapping, but confirm the specific "Table A.n" against a licensed
  2010 edition before quoting it.
- The **IEC 61508-6 Annex D Table D.4** score→β cut-offs and any device-hour-vs-SIL
  confidence tables are proprietary tabular content — populate from a licensed copy,
  not from secondary paraphrase.
- The 1oo2 / 1oo2D **Annex B formulas** are the widely-published forms; verify
  term-by-term against your licensed Annex B (edition-specific t_CE/t_GE conventions
  vary) before quoting in a certification submission.
- The **FIT figures in §3–§4 are order-of-magnitude, generic**, from secondary sources
  (exida blog, ASIL targets, FMD mode logic, standard reference conditions) — starting
  points to be replaced by sourced register entries, not final values.

## Key sources

**Primary standards:** IEC 61508-1/-2/-4/-6/-7:2010 (SIL bands; architectural
constraints Tbl 2/3; λ classification; Annex B PFH/PFDavg + Annex D β; technique
catalog; §7.4.10 proven-in-use). ISO 13849-1:2023 (B10d/MTTFd/DC). IEC 61709:2017 +
IEC/TR 62380 (reference conditions & stress models). MIL-HDBK-217F N2. FIDES 2009/2022
(UTE C 80-811). Telcordia SR-332 Iss.4.
**Databases:** Siemens SN 29500; Quanterion NPRD-2016 / EPRD / **FMD-2016** / 217Plus;
**exida SERH 4th ed.** / SILStat.
**Secondary (figures & worked reasoning):** exida (relay 600/60 FIT, demand modes,
proven-in-use math), Intertek (proven-in-use, demand modes), Analog Devices EngineerZone
(CCF quantification, PFH/PFD, proven-in-use math), SINTEF PDS (β/Annex D), PR electronics
& gt-engineering (Type A/B, Route 1H/2H), TI SLOA294A (FIT/ASIL targets), CALCE (FIDES
assessment), Omron/ReeR/machinerysafety101/ABB (B10d/MTTFd/DC).
