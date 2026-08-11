<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Protective-Stop — Hazard Analysis & Risk Assessment (HARA)

**Status:** first draft for curation by the safety lead. Machine-generated from
the authoritative backbone `docs/safety/SYSTEM_DEFINITION.md` and the design
records under `docs/`. Every vehicle-level number is an **explicit assumption**
(§2) the integrator must confirm — no severity, exposure, or demand-rate figure
has been silently chosen. Function IDs (SF-1..SF-3, F-R-xx, F-H-xx, F-M-xx,
F-P-xx) are used verbatim from the System Definition.

**Standard frame:** IEC 61508 (SIL / systematic capability) for the electronic
stop function **and** ISO 13849 (Performance Level) for the actuation/machinery
side. Design target under evaluation: **SIL 3 / PL e** for the on-demand stop.
This document *derives* the required level from risk parameters and then states
whether that target is justified, higher, or lower (§6).

**Analysis basis (code, not stale design):** per the reconciliation block in
SYSTEM_DEFINITION §, the implemented stuck-at / systematic controls are
**Option A** (fresh both-phase physical sample every tick) and **Option B**
(two cores form the verdict by diverse expressions — arithmetic-image vs
boolean). **Option C** (LFSR/PRNG challenge, liveness-XOR accumulator, stamp
echo-signature) was **dropped 2026-07** and is analysed here as a *residual*,
not a control. `PSTOP_SAFETY_DESIGN.md` §5–§7 still describe Option C as the
target architecture and are stale on that point (see §7 Doc-vs-code register).

---

## 1. Scope & method

This HARA covers the **protective-stop (pstop) function** as bounded by
SYSTEM_DEFINITION §1: a wireless-linked handheld/fixed **remote** (ESP32-S3,
dual-core lockstep, DPST normally-closed stop button on two loops) driving a
**machine controller** (host software runner `host/`, ROS 2 node
`ros2/protective_stop_machine/`, or ESP32 `machn` unit) to a **de-energized
STOP** safe state over a Tailscale/WireGuard **black channel**. The
pre-qualified protocol library `pstop_c` is referenced at its interface
(F-P-01..04) and is **not** re-assessed; only its assumptions-of-use are carried
as integration requirements. Hazards are expressed as *failures of the pstop
function* (SF-1..SF-3 violated), following IEC 61508-1 clause 7.4. Risk is rated
twice — with the **IEC 61508-5 Annex D risk graph** (C/F/P/W → SIL) and the
**ISO 13849-1 Annex A risk graph** (S/F/P → PLr) — and the higher demand is
carried. The safe state throughout is **de-energize-to-safe**: absence of a
valid, fresh, agreed OK message ⇒ machine heartbeat-times-out ⇒ STOP.

> **Classification note.** The pstop is a **protective stop** (a normal
> safeguarding function), *not* the machine's emergency-stop (E-stop) function
> under IEC 60204-1 / ISO 13850. The "E-stop switch" is only the hardware part
> name of the DPST button. The vehicle still requires its own compliant
> emergency-stop; this HARA does not discharge that obligation.

---

## 2. Assumption register (vehicle-level — integrator to confirm)

These parameters are **not** derivable from the pstop firmware/design and drive
every risk rating below. Each must be confirmed by the **vehicle integrator's
system HARA** before any SIL/PL claim is final.

| ID | Assumption (worst-case default used here) | Rationale | Confirmed by |
|---|---|---|---|
| **A-01** | Worst-case consequence of a pstop failure is **contact between a moving mobile machine and a person → death or irreversible injury** (IEC C2+, ISO S2). | Off-highway / industrial mobile vehicle (SYSTEM_DEFINITION §2) has kinetic energy sufficient to crush. | Integrator (mass, speed, kinetic energy, guarding) |
| **A-02** | Operator / bystander exposure to the hazard zone is **frequent-to-continuous** (IEC F2, ISO F2). | Autonomous vehicle operating in shared/occupied space; a remote implies humans nearby. | Integrator (site, traffic, exclusion zones) |
| **A-03** | Possibility of **avoiding** the hazard is **scarcely possible / almost impossible** (IEC P2, ISO P2). | Autonomous motion, limited human perception of intent, potential blind approach. May reduce to P1 with low speed + audible/visual warning + clear sightlines. | Integrator (speed, warning devices, sightlines) |
| **A-04** | **Demand rate** on the stop function is **unknown**; assumed **W2–W3** (moderate-to-high). | Cannot be inferred from firmware. This single parameter moves the IEC result by up to two SIL steps. | Integrator (operational demand analysis) |
| **A-05** | **Process Safety Time (PST)** is a vehicle-level number and is **unknown**. Placeholder **PST_min**. The pstop's own worst-case signal-latency budget is **≈ 2.1 s** on the silence/link-loss path (heartbeat timeout `heartbeat_ms × max_missed` = 400 ms × 5 ≈ 2.0 s + one 100 ms tick). **`max_missed` is a tunable, raised 3 → 5 on 2026-08-04** to eliminate control-plane-re-sync nuisance stops; this lengthened the budget from ≈ 1.3 s to ≈ 2.1 s. | FTTI must satisfy `pstop latency + actuation/braking time ≤ PST`. | Integrator (must confirm `PST ≥ 2.1 s + braking`; **any vehicle previously sized for the 1.3 s budget must be re-checked** — else tighten `heartbeat_ms`/`max_missed`, at the cost of nuisance-stop margin) |
| **A-06** | **De-energize = STOP is genuinely safe** for this vehicle (no rollaway on grade, no hazardous load release, no loss of steering/brake authority when de-energized). | The entire architecture rests on de-energize-to-safe (SYSTEM_DEFINITION §5). A vehicle that rolls when de-energized breaks this and needs a held/spring-applied brake. | Integrator (grade, brake type, load dynamics) |
| **A-07** | The **final actuation element** is either the in-scope ESP32 `machn` **dual relay in series + feedback** (MACHINE_ESP32_DESIGN), or an **integrator-supplied element of ≥ the allocated PL/SIL**. The host/ROS 2 **software** machine emits a **non-safety** signal only (MACHINE_ROS2_NODE_DESIGN §1). | The safety function is only as good as its final element; the software machine has none in scope. | Integrator (actuator selection & rating) |
| **A-08** | The transport (Tailscale/WireGuard/DERP over WiFi/Eth/USB-NCM) is an **untrusted black channel** per IEC 61508 / IEC 62280 §; all safety detection lives in the endpoints (CRC-16, counter/stamp echo, heartbeat). | The design explicitly treats `pstop_c` as a black-channel protocol (PSTOP_SAFETY_DESIGN §3). | Assessor (residual bit-error / insertion / masquerade rate argued in FMEDA) |
| **A-09** | `pstop_c` is a **validated SEooC** with systematic capability ≥ the allocated target, and its interface obligations F-P-01..04 are met by the shell. | SYSTEM_DEFINITION §1 declares it pre-qualified and out of scope. | Assessor (SEooC assumptions-of-use vs this integration) |
| **A-10** | As-configured timing on the safety-credited machine (machn, A-07) is **10 Hz tick, `heartbeat_ms` = 400, `max_missed` = 5 (raised from 3 on 2026-08-04), `min_stop_ms` = 500** and any field reconfiguration stays inside the validated safe envelope `max_missed ∈ [1,5]` (ROS 2 validator refuses looser values). `max_missed` = 5 now sits at the **envelope ceiling** — there is no headroom to loosen it further without widening the validated envelope. | SYSTEM_DEFINITION §2; MACHINE_ROS2_NODE_DESIGN §4/§12 (envelope). | Integrator (commissioned config) |

---

## 3. Hazard identification

Hazards are failures of the pstop function. **Cause classes:** RHW = random
hardware; SW = systematic software; CCF = common-cause (dual-core / channel);
BC = black-channel / comms; H/O = human / operational.

| Hazard ID | Hazardous event (pstop function failing) | Operating situation | SF violated | Cause classes |
|---|---|---|---|---|
| **H-01** | **Fail-to-stop on demand** — button pressed or a fault is present, yet the machine holds a RUN/OK state. | Person enters path; operator presses stop. | SF-1 | RHW (masked stuck sense pin), SW, CCF, BC (forged/held OK) |
| **H-02** | **Spurious / nuisance stop** — machine stops with no demand and no fault. | Vehicle moving in traffic / carrying load / on grade. | (counter-risk to SF-1; secondary hazard) | RHW (EMC blip), SW, tight `max_missed` margin |
| **H-03** | **Delayed stop beyond PST** — stop is commanded but arrives after the process safety time. | High-speed approach; short PST. | SF-1 (timeliness) | BC (DERP dark window 5–9 s; degraded-mode ladder), heartbeat timeout, debounce |
| **H-04** | **False / spontaneous arming** — machine transitions STOP→OK without a deliberate operator gesture. | Unattended vehicle re-energizes itself. | SF-3 | RHW (EMC both-channel blip), CCF, BC (replayed OK), H/O |
| **H-05** | **Undetected loss of remote link** — remote is gone/silent but the machine keeps believing a stale OK. | Link drops mid-operation. | SF-2 | BC, SW (frozen `get_time_cb` — F-P-03 breach), RHW |
| **H-06** | **Stale / replayed OK accepted** — an old valid OK is replayed or held and treated as fresh. | Adversarial or fault-induced packet replay. | SF-2 | BC, SW |
| **H-07** | **One-of-many-remotes masking** — in many→one, a misbehaving/ghost remote's OK or a lost bond masks another remote's STOP, or arming ownership is confused. | Multiple bonded remotes on one machine. | SF-1 / SF-3 | SW, BC, H/O |
| **H-08** | **Boot-time transient OK** — power-on loop glitch puts a STOP→OK episode on the wire, self-arming the machine. | Every remote power-up / OTA reboot. | SF-3 | RHW (boot GPIO transient), SW |
| **H-09** | **Common-cause dual-core systematic SW fault** — an identical bug on both cores yields identical wrong OK; the byte comparator agrees. | Any; latent until triggered. | SF-1 / SF-2 | CCF, SW |
| **H-10** | **Machine-side relay weld / stuck-on** — the final element cannot de-energize; commanded STOP does not open the stop circuit. | Contact welds after many cycles / driver short. | SF-1 | RHW |
| **H-11** | **Forged / fail-danger OK on the black channel** — partial corruption or the all-zeros/`OK==0x00` polarity produces an OK the machine accepts. | Bit errors, buffer zeroing, malicious injection. | SF-1 / SF-2 | BC, RHW |
| **H-12** | **Final actuation element not safety-rated** — software/ROS 2 machine's STOP is a non-safety signal the robot control stack may ignore or delay. | Software or hardware-proxy machine deployment. | SF-1 | (architectural / scope) |
| **H-13** | **Non-operator remote re-arms the machine** — an accepted-but-not-authorized (stop-only) remote, or any remote absent from the machine's operator allowlist, performs a STOP→OK re-arm and resumes the machine. | Machine STOPPED; a remote that was admitted only to *stop* drives (or completes) the arming gesture. | SF-3 | SW, H/O (mis-configured operator allowlist) |

---

## 4. Risk assessment

Ratings use the worst-case assumptions of §2 (A-01 severity, A-02 exposure,
A-03 avoidance, A-04 demand). **The result is therefore an upper-bound
envelope; the integrator's confirmed parameters may lower it.**

**IEC 61508-5 Annex D key:** C = consequence (C1 minor … C2 death of one …
C3/C4 multiple); F = exposure (F1 rare … F2 frequent/continuous); P = avoidance
(P1 possible … P2 almost impossible); W = demand rate (W1 low … W3 high).
**ISO 13849-1 Annex A key:** S = severity (S1 slight/reversible, S2
serious/irreversible); F = exposure (F1 seldom, F2 frequent); P = avoidance
(P1 possible, P2 scarcely possible).

| Hazard | C | F | P | W | → **SIL** | S | F | P | → **PLr** | Reasoning |
|---|---|---|---|---|---|---|---|---|---|---|
| **H-01** | C2+ | F2 | P2 | W2–W3 | **SIL 3** | S2 | F2 | P2 | **e** | The defining demand of the function. Person in path + unavoidable + frequent exposure → ISO lands squarely at PLr e (S2·F2·P2). IEC gives SIL 2 at W2, **SIL 3 at W3**; carry SIL 3 as the conservative envelope for unattended autonomy. Chief risk driver. |
| **H-02** | C2 | F2 | P1 | W1–W2 | **SIL 1–2** | S2 | F2 | P1 | **c–d** | A nuisance stop is not itself the primary hazard, but a sudden de-energize in traffic / on grade / under load (A-06) can cause secondary collision or load shift → not negligible. Avoidance is easier (P1: predictable stop). Managed by fail-safe-direction-only debounce; never rated above the primary function. |
| **H-03** | C2+ | F2 | P2 | W2–W3 | **SIL 3** | S2 | F2 | P2 | **e** | Same outcome as H-01 for the collision — a stop after PST is a fail-to-stop. Amplified by the measured **5–9 s DERP failover dark window** and the degraded-mode ladder pausing Tailscale. Timeliness is a first-class safety property here, not just availability. |
| **H-04** | C2+ | F2 | P2 | W2 | **SIL 3** | S2 | F2 | P2 | **e** | Spontaneous re-energize of an unattended machine puts a person at risk with no human in the loop. The EMC both-channel-blip self-arming failure is *documented as having occurred* (FAILOVER_AND_ARMING §2), so likelihood is credible, not theoretical. |
| **H-05** | C2+ | F2 | P2 | W2 | **SIL 3** | S2 | F2 | P2 | **e** | Undetected staleness defeats the whole freshness premise (SF-2). Heartbeat on the machine's own monotonic clock is the control; a frozen `get_time_cb` (F-P-03 breach) is the dangerous-undetected residual. |
| **H-06** | C2+ | F2 | P2 | W2 | **SIL 3** | S2 | F2 | P2 | **e** | Anti-replay now rests **solely** on `pstop_c` native counter monotonicity + `MSG_LOST`/`OUT_OF_ORDER` bounded-loss (F-P-04). The extra stamp echo-signature (Option C) was dropped, so the residual is larger than PSTOP_SAFETY_DESIGN §5.3 implies. |
| **H-07** | C2+ | F2 | P2 | W2 | **SIL 3** | S2 | F2 | P2 | **e** | Fail-safe OR (any STOP or any silence wins) is the primary control (F-H-05); the residual is a *ghost bond* that still counts as present-and-OK, and arming-ownership confusion. Validated 32/32 in `pstop_multi_remote_test.py` but unquantified for random-fault masking. |
| **H-08** | C2+ | F1 | P2 | W1 | **SIL 2** | S2 | F1 | P2 | **d** | Exposure is per-boot (F1), not continuous, so lower W/F — but a boot that self-arms is severe. Boot warm-up hold (`LOOP_BOOT_OPEN_CONFIRM_TICKS = 5`) is the control; residual only if the hold is insufficient on some board. |
| **H-09** | C2+ | F2 | P2 | W2–W3 | **SIL 3** | S2 | F2 | P2 | **e** | The **dominant systematic risk** (PSTOP_SAFETY_DESIGN G1, β≈1 for a shared bug through a byte comparator). Only lever implemented is **Option B diverse expressions**; the "semantic 1oo2D" comparator (§5.4) was **not** built. Diversity sufficiency is **unquantified — no FMEDA**. |
| **H-10** | C2+ | F2 | P2 | W2 | **SIL 3** | S2 | F2 | P2 | **e** | Classic dangerous-undetected. ESP32 `machn`: two relays **in series** (either breaks) + feedback contradiction → stop-on-contradiction ≥ ~1 s gives HFT=1 + diagnosis. Residual: a **single-channel or software** machine has **no** in-scope rated relay (A-07, H-12). **[DESCOPED 2026-08 — the feedback/diagnosis half is OFF by default pending new machine HW, REVERSIBLE (`CONFIG_MACHN_RELAY_FEEDBACK`); see docs/RELAY_FEEDBACK_DESCOPE.md. HFT=1 from the series relays is UNCHANGED (stop-on-demand safe), but the stuck-on diagnosis + stop-on-contradiction are removed, so a single weld is latent-to-next-demand. Compensate with a periodic commanded-open proof test.]** |
| **H-11** | C2+ | F2 | P2 | W2 | **SIL 3** | S2 | F2 | P2 | **e** | `OK==0x00` is an **inherited fail-danger polarity** (all-zeros decodes toward OK, SYSTEM_DEFINITION §5). Compensated by CRC-16 over bytes[0..37] (a zeroed CRC won't match), transmit-on-agreement, and the live-sample derivation. Standing FMEA anchor. |
| **H-12** | C2+ | F2 | P2 | W2 | **SIL 3** | S2 | F2 | P2 | **e** | For the software/ROS 2 machine the "STOP" is a ROS topic explicitly carrying **no safety responsibility**; enforcement reduces to remotes fail-safe-stopping on their own heartbeat timeout. The **final element integrity is unallocated** unless the integrator supplies it. Architectural gap, not a coding defect. |
| **H-13** | C2+ | F2 | P2 | W2 | **SIL 3** | S2 | F2 | P2 | **e** | Same collision outcome as H-04: an unattended machine re-energized by a remote never authorized to *resume* it (only to *stop* it). The control is `pstop_c` gating the STOP→OK ownership on `is_stop_only == false` (`machine.c:135-142`); a stop-only remote's STOP still forces STOP (`:131,155-156`) and it is heartbeat-monitored (`machine_check_heartbeats:266-320`). The **default is maximally safe** — the operator allowlist is empty out of the box, so every bonded remote is stop-only until explicitly promoted. Residual is a **mis-configured allowlist** that mints a non-operator as operator (FMEA H02-4). |

---

## 5. Safety goals

Positive requirements that, if met to the allocated integrity, close the
hazards. **FTTI / PST** entries reference the A-05 placeholder because the
absolute number is a vehicle-level input.

| SG | Safety goal (positive requirement) | Safe state | FTTI / PST | Allocated target | Closes |
|---|---|---|---|---|---|
| **SG-1** | On any stop demand **or** any detected fault, command the machine to **de-energized STOP** within the fault-tolerant time. | De-energized (no valid OK) | ≤ PST_min (A-05); pstop budget ≈ 2.1 s worst case (`max_missed` = 5 since 2026-08-04) | **SIL 3 / PL e** | H-01, H-03, H-09, H-11 |
| **SG-2** | Sustain OK **only** while it is *freshly and causally* derived from a live both-channel read **and** a valid black-channel round-trip; stale/replayed/forged OK must fail to STOP. | De-energized on any staleness | ≤ heartbeat timeout ≈ 2.0 s | **SIL 3 / PL e** | H-05, H-06, H-11 |
| **SG-3** | Permit STOP→OK **only** via a deliberate arming gesture (STOP held ≥ `min_stop_ms` = 500 ms, then release) **and only from a remote on the machine's operator allowlist** (`is_stop_only == false`, default empty); never spontaneously, on a transient, at boot, or from a stop-only / non-operator remote. | Remain STOPPED until a valid gesture from an authorized operator | n/a (state guard) | **SIL 3 / PL e** | H-04, H-08, H-13 |
| **SG-4** | In many→one, **any** bonded remote's STOP or silence forces STOP regardless of other remotes (fail-safe OR); arming requires a single owning gesture, and a lost/ghost bond must not read as present-and-OK. | De-energized if any remote demands/absent | ≤ heartbeat timeout | **SIL 3 / PL e** | H-07 |
| **SG-5** | The **final actuation element** shall de-energize-to-safe with **HFT = 1** and diagnosable stuck-on (relay-output feedback), **or** the integrator shall provide an element of ≥ the allocated integrity. Software/ROS 2 machine outputs are non-safety and must not be the sole enforcement path. **[DESCOPED 2026-08: the "diagnosable stuck-on (relay-output feedback)" limb is OFF by default on `machn` pending new machine HW, REVERSIBLE (`CONFIG_MACHN_RELAY_FEEDBACK`) — docs/RELAY_FEEDBACK_DESCOPE.md. The HFT=1 de-energize-to-safe limb still holds; the requirement now rests on the integrator-supplied element OR restoration of feedback on the new HW. Stuck-on diagnosis is a documented gap meanwhile.]** | Stop circuit open | ≤ `RELAY_FEEDBACK_MS` + ~1 s contradiction hold | **SIL 3 / PL e** (allocated to integrator for software machine) | H-10, H-12 |
| **SG-6** | Bound the spurious-STOP rate so it does not introduce a secondary hazard (traffic/grade/load), **without ever filtering or delaying the stop path** (asymmetric, fail-safe-direction-only debounce). | STOP is always permitted; only re-arm is gated | open→STOP single-tick (~100 ms) | **No de-energize SIL** (availability goal, fail-safe by construction); manage per A-06 | H-02 |

---

## 6. Integrity-target conclusion — is SIL 3 / PL e correct?

**ISO 13849 (PLr): confirmed at PL e.** Under the worst-case assumptions the
severity/exposure/avoidance triple for the primary hazards (H-01/03/04/05/06/07/
09/10/11/12/13) is **S2 · F2 · P2**, which lands directly on **PLr = e** with no
ambiguity. PL e is the correct machinery-side requirement for an unattended
mobile autonomous vehicle whose pstop failure can kill an unavoidable bystander.
It is not higher (there is no PL above e); it drops to **PL d** only if the
integrator confirms **P1** (avoidable — low speed, warnings, sightlines) or
**F1** (rare exposure), which A-02/A-03 do not currently support.

**IEC 61508 (SIL): SIL 3 is the correct conservative target, but it is
demand-rate-sensitive.** The risk graph gives **SIL 2 at W2 and SIL 3 at W3**
for the C2·F2·P2 hazards. Because the **demand rate W is unknown (A-04)** and the
vehicle is unattended, **SIL 3 is the right design envelope** and is consistent
with the ISO PL e result (ISO 13849-1 Table 4 maps PL e ↔ SIL 3). If the
integrator's demand analysis firmly establishes **low demand (W1–W2)** *and*
avoidability (P1), a **SIL 2** allocation could be justified for the electronic
function — but that is a deliberate downgrade the safety lead must sign, not a
default.

**Architecture supports the SIL 3 *claim* only conditionally.** The design
provides the structural preconditions:

- **HFT = 1**, remote **1oo2D to stop / 2oo2 to run**, machine relay
  **de-energize-to-safe** (SYSTEM_DEFINITION §2; PSTOP_SAFETY_DESIGN §9).
- ESP32 `machn` final element is **1oo2D in series** with output feedback →
  HFT = 1 on actuation with stuck-on diagnosis (MACHINE_ESP32_DESIGN).
- Continuous per-tick online proof test (Option A both-phase), fail-safe
  default STOP, asymmetric debounce, arming policy, boot warm-up hold.

**However, the quantitative SIL 3 claim is not yet supported**, and three items
gate it:

1. **No FMEDA.** SFF, DC, PFH/PFD_avg, and β are unquantified. SIL 3 route 1H
   demands (per IEC 61508-2) SFF ≥ 99 % at HFT 1, PFH 10⁻⁸–10⁻⁷/h. None is
   demonstrated. **This is a hard blocker on the numerical claim.**
2. **Common-cause coverage rests on Option B diversity through a *byte*
   comparator.** The design's own G1 says a byte-identical `memcmp` gives ≈0
   systematic coverage; the "semantic 1oo2D" comparator that was meant to fix
   this (§5.4 / D1) **was not implemented**. The SIL 3 systematic case therefore
   hangs entirely on Option B's arithmetic-image-vs-boolean diversity being
   sufficient to diverge the two byte streams under a shared bug — **credible
   but unproven** (fault-injection row 9 of PSTOP_SAFETY_DESIGN §10 must pass).
3. **Systematic capability (SC 3).** SIL 3 requires SC 3 process rigour on the
   new sensing/comparator/arming code. MISRA C is done
   (`MISRA_COMPLIANCE_2026-07-21.md`); static analysis in CI, requirements
   traceability, and structured fault-injection are claimed but not yet evidenced
   to the SC 3 bar (the coverage regime in SYSTEM_DEFINITION targets it).

**Verdict:** **SIL 3 / PL e is the correct target and the architecture is
plausibly capable of it, but the claim is currently unsubstantiated.** Treat
SIL 3 / PL e as *allocated, not achieved*, pending FMEDA + fault-injection
evidence + integrator confirmation of A-01..A-05.

---

## 7. Open items — inputs the integrator / vehicle HARA must supply

**Vehicle-level inputs (block the risk ratings):**

1. Confirm or replace A-01..A-05: worst-case severity, exposure, avoidability,
   **demand rate W**, and **Process Safety Time**. W and PST are the two numbers
   that can move the SIL result and the FTTI budget.
2. Confirm A-06: that de-energize is truly safe (no rollaway on grade, no
   hazardous stored energy). If not, add a spring-applied / held brake and
   re-rate — de-energize-to-safe is otherwise invalid.
3. Confirm A-07/SG-5: **who owns the final actuation element** for the
   software/ROS 2 machine. As drawn, that machine has **no rated final element**
   in scope (H-12) — enforcement is only the remotes' fail-safe timeout.

**Evidence blocking the SIL 3 / PL e claim:**

1. **FMEDA** for the remote sensing chain and the machine relay chain: SFF, DC,
   β, PFH. Without it there is no quantitative SIL/PL claim (§6.1).
2. **Fault-injection proof of Option B diversity** (PSTOP_SAFETY_DESIGN §10
   row 9): inject an identical logic fault into both cores' sensing and show the
   byte encodings still diverge. If they do not, common-cause (H-09) is
   uncontrolled and SIL 3 fails.
3. **Anti-replay residual** (H-06): argue the `pstop_c` native counter /
   `MSG_LOST` bounded-loss window is sufficient without the dropped stamp
   signature, quantified against A-08.
4. **Frozen-clock diagnostic** (H-05): demonstrate the machine's `get_time_cb`
   cannot silently stall (F-P-03), or add a monotonicity check.
5. **DERP dark-window vs PST** (H-03): the measured 5–9 s failover gap and the
   degraded-mode ladder must be shown ≤ PST, or STOP must be forced during the
   gap. **This is likely to fail for any short-PST vehicle** and needs explicit
   treatment. **Note (2026-08-04):** the silent-remote detection latency itself
   was lengthened from ≈ 1.2 s to ≈ 2.0 s (`max_missed` 3 → 5), which further
   tightens the H-03 PST margin for short-PST vehicles even on the non-failover
   path — re-check A-05 against the vehicle PST.
6. Confirm the pstop is **not** credited as the machine's emergency-stop; verify
   an independent IEC 60204-1 / ISO 13850 E-stop exists.

**Doc-vs-code register (found during this analysis — fix before sign-off):**

| # | Location | Inconsistency |
|---|---|---|
| D1 | PSTOP_SAFETY_DESIGN §5.6 vs SYSTEM_DEFINITION §2 (+ MACHINE_ROS2 §12, memory) | Heartbeat timeout formula **and** values are stale: design doc says `heartbeat_ms × (max_missed + 1)`, "1000 × 2 ≈ 1–2 s"; the code/backbone is `heartbeat_ms × max_missed`, **400 × 5 ≈ 2.0 s** (`max_missed` raised 3 → 5 on 2026-08-04; was 400 × 3 ≈ 1.2 s). |
| D2 | PSTOP_SAFETY_DESIGN (whole doc) vs SYSTEM_DEFINITION | Design doc is **SIL 2**-framed throughout; the backbone target is **SIL 3 / PL e**. |
| D3 | PSTOP_SAFETY_DESIGN §5, §6 (fault→reaction map), §7 vs SYSTEM_DEFINITION reconciliation | §5–§7 describe **Option C** (LFSR challenge `Cn`, liveness accumulator `S ^= (Cn^En)`, stamp echo-signature) as the *implemented* target architecture. **Option C was dropped 2026-07.** The actual stuck-at/systematic controls are **Option A** (both-phase fresh sample) + **Option B** (diverse expressions). The §6 fault→reaction map cites mechanisms **that do not exist in the code**. |
| D4 | PSTOP_SAFETY_DESIGN §4 gap G3 | Lists predictable `counter&1` as an open gap "lowering DC". Moot/misleading: Option A both-phase is the adopted compensating measure, not the dropped LFSR. |
| D5 | PSTOP_SAFETY_DESIGN §5.4 / §8 D1 vs SYSTEM_DEFINITION F-R-04 | Recommends a **semantic 1oo2D** comparator; the implemented comparator is still **byte-identical `memcmp`** of the 40-byte encodings. Common-cause coverage therefore depends on Option B diversity, not the semantic comparator the doc credits. |
| D6 | MACHINE_ROS2_NODE_DESIGN §1 vs whole-system expectation | For the software machine the STOP output is explicitly **non-safety**; the final-element integrity for that form factor is undefined in scope (H-12). Not a contradiction, but a scope gap the safety case must close. |

---

*End of draft. Every SIL/PL figure above is conditional on §2 and the §7
evidence. Curate against the vehicle HARA before any external safety claim.*
