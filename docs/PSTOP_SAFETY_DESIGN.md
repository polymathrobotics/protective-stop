# Protective-Stop (pstop) Integrity Chain — Safety Design

> **Reconciliation status — updated 2026-08-02.** This document has been
> reconciled against the current firmware and **predates the shipped code**. It
> is **SIL2-framed**; the program target is now **SIL 3 / PL e**
> (`docs/safety/SYSTEM_DEFINITION.md`). Its **§5–§10 describe the DROPPED
> "Option C" integrity chain** (LFSR/PRNG challenge, `S ^= (Cn ^ En)` liveness
> accumulator, stamp echo-signature) and an **unimplemented GPIO
> config-integrity check** as if live. The **implemented** stuck-at/systematic
> controls are **Option A** (fresh both-phase physical sample every tick) +
> **Option B** (two cores form the verdict by diverse expressions; a byte
> `memcmp` of the two 40-byte encodings detects divergence). Inline
> `> **[Reconciled 2026-08-02]**` notes and bold status markers
> (**DROPPED 2026-07** / **NOT IMPLEMENTED**) below correct each claim **without
> deleting the original design rationale**. Full register:
> `docs/safety/RECONCILIATION.md`. Ground truth: `SYSTEM_DEFINITION.md`.

**Status:** design record. §2 describes what is implemented today
(verified against `firmware/main/main.c`); §4–§8 are the SIL2 hardening
roadmap, still awaiting decisions. Merged from the former
ESTOP_SAFETY_DESIGN.md + ESTOP_SAFETY_DECISIONS.md.
**Standard frame:** IEC 61508 (SIL2 target, HFT=1, de-energize-to-safe).
> **[Reconciled 2026-08-02]** Program target is now **SIL 3 / PL e**
> (SYSTEM_DEFINITION; HARA §6). The SIL2 rationale throughout this doc is
> retained as design history; read the integrity level as **SIL 3 / PL e**.
**Scope:** the remote's stop-switch sensing → verdict → pstop message
path. `pstop_c` (machine *and* remote protocol library,
`components/pstop/upstream`, certification track) stays byte-for-byte
unchanged; everything here lives in the application shell
(`firmware/main/main.c`, the comparator, the GPIO sensing) around it.

Terminology: the product function is a **protective stop (pstop)** — the
physical DPST normally-closed button on the loops is referred to as the
"E-stop switch" only as the hardware part name.

---

## 1. The property to guarantee

> There is **no viable execution path** in which the pstop message says
> `OK` while the physical switch loop is open, stuck, or unread.

Equivalently: a sustained `OK` reaching the machine must be *causally and
freshly* derived from a correct, live read of the switch loop on **both**
independent channels. Any fault — open loop, stuck pin, frozen task,
bypassed branch, lost packet, dead machine — must drive the system to
STOP within the process safety time.

## 2. Implemented today (`firmware/main/main.c`)

- **Dual-channel loopback.** Core 0 owns channel A (IO39 drive →
  DPST pole 1 → IO40 sense); core 1 owns channel B (IO41 →
  pole 2 → IO42). Sense inputs are pulled DOWN, so an open loop
  reads 0 (STOP) — fail-safe idle.
- **Rolling drive level.** *(superseded — see reconciled note.)* Each tick
  the owning core drives its OUT pin
  with the tick counter's LSB (10 µs settle, `ESTOP_SETTLE_US`) and
  verifies the echo. The loop counts as closed only when the most recent
  drive-high echoed high (continuity) AND the most recent drive-low
  echoed low (no stuck-high short). Until both phases have been sampled
  the channel reads OPEN.
  > **[Reconciled 2026-08-02]** **Superseded by Option A both-phase.** The
  > current code (`main.c:265-308`) samples **both phases every tick** — drive
  > HIGH → read → drive LOW → read, in one call — and **ignores the tick
  > counter** (`(void)counter;` "both phases are sampled every tick; no rolling
  > phase", `main.c:269`). The verdict is thus fresh each tick with no two-tick
  > latency. The "rolling counter-LSB, one phase per tick" text above is the
  > *predecessor* design and is what SYSTEM_DEFINITION means by "not the old
  > rolling `counter&1`".
- **Asymmetric release debounce** (`LOOP_RECLOSE_DEBOUNCE_TICKS = 3`).
  The open→STOP edge is single-tick — the stop path is never filtered.
  The closed→OK edge requires 3 consecutive healthy reads, so
  EMC-induced blips produce a ≥300 ms STOP episode instead of chatter.
- **Boot warm-up hold** (`LOOP_BOOT_OPEN_CONFIRM_TICKS = 5`). The
  comparator sends NOTHING until each channel has settled (one full
  closed-debounce cycle, or 5 consecutive open reads = switch genuinely
  held at boot). This keeps the boot-time loop glitch from putting a
  STOP→OK episode (the arming gesture) on the wire at every power-on.
- **Lockstep comparator.** Both cores independently compute a verdict and
  encode the full 40-byte `pstop_msg_t`; the comparator memcmps the two
  encodings — including the CRC — and transmits only on agreement.
  2oo2-to-run, 1oo2-to-stop: a single-channel fault makes the cores
  disagree → nothing is sent → the machine heartbeat-times out to STOP.
- **Machine-side arming policy** (`host/machine_app_runner.c`,
  `min_stop_ms = 500` in `host/machine.toml`): a STOP episode must be
  held ≥500 ms for its OK release to count as the arming gesture; shorter
  STOP→OK cycles are vetoed via the library's public
  `machine_stop_robot()`. See `FAILOVER_AND_ARMING_DESIGN_2026-07-21.md`.

## 3. The black channel is end-to-end

pstop_c is explicitly a black-channel protocol:

- The machine echoes the remote's own data back in every reply
  (`protocol.c`): `resp->received_counter = req->counter`,
  `resp->received_stamp = req->stamp`.
- The machine also *validates* the reverse direction — it checks that the
  remote correctly echoed the machine's counter/stamp, with bounded loss
  and monotonic time (`MSG_LOST`, `OUT_OF_ORDER`).

Both ends can therefore confirm their own data made the full round trip,
and the machine's *native* checks already trip to STOP if the remote's
counter/stamp sequence stalls or regresses. That gives a closed loop —
remote → machine → remote — that the GPIO can be **spliced into**, so a
GPIO fault breaks a chain the *machine* polices, not just the remote.

## 4. Gaps in today's design (the SIL2 case)

| # | Gap | Why it matters at SIL2 |
|---|-----|------------------------|
| G1 | **Common-cause / systematic SW faults.** Two cores run identical code → an identical bug yields identical wrong `OK` → comparator agrees. | Byte-compare gives ~0 systematic-fault coverage (β≈1). Usually the dominant SIL2 risk. |
| G2 | **Bypassable verdict.** `closed ? OK : STOP` is a branch; a wild write / optimizer / logic bug can reach `OK` without a read. | Defeats "OK ⟺ correct read." |
| G3 | **Predictable challenge.** `counter&1` is a known toggle; a fault that happens to track it isn't forced to reveal itself. | Lowers diagnostic coverage (DC). |
| G4 | **Loose coupling to the message.** The verdict is computed *beside* the message, not woven into the data that round-trips. | A stale/forged value can ride the protocol. |

> **[Reconciled 2026-08-02]** How these gaps were actually dispositioned:
> **G3** is **moot as written** — the predictable `counter&1` was *not* fixed by
> the dropped LFSR challenge; the adopted compensating measure is **Option A**
> both-phase sampling (a stuck pin fails the high *or* low phase without any
> unpredictable challenge; `counter` is ignored, `main.c:269`). **G1**
> (common-cause) is addressed by **Option B** diverse expressions, **not** the
> §5.4 semantic comparator (see the §5.4 note). **G2/G4** were targets of the
> dropped Option-C chain and remain **residuals** tracked in HARA (H-06/H-09)
> and FMEA (DU-3/DU-5), not closed controls.

## 5. Proposed integrity-chain architecture — **DROPPED 2026-07 (Option C)**

> **[Reconciled 2026-08-02]** **The entire §5 integrity chain below was assessed
> and NOT adopted (Option C, dropped 2026-07).** The LFSR/PRNG challenge (§5.1),
> the `S ^= (Cn ^ En)` liveness accumulator (§5.2), the stamp echo-signature
> (§5.3), the semantic-1oo2D comparator (§5.4), and the runtime GPIO
> config-integrity check (§5.5) are **not in the code**. The implemented controls
> are **Option A** (fresh both-phase physical sample per tick, `main.c:265-308`)
> - **Option B** (two cores form the verdict by diverse expressions — arithmetic
> image `:305` vs boolean `:307` — with a byte `memcmp` of the two 40-byte
> encodings, `:890`, detecting divergence). The text is kept as design rationale;
> see per-subsection notes and `docs/safety/RECONCILIATION.md` (R-01/R-02/R-04).

One token must circulate the **entire** physical+logical loop every
cycle, and must come back fresh and correct to keep `OK` alive:

```
 per-core LFSR challenge Cn
        │  drive
        ▼
   GPIO OUT ─▶ DPST pole ─▶ GPIO IN   (echo En; En==Cn iff loop closed+healthy)
        │  read
        ▼
   fold En into: (a) counter advance   (b) stamp signature
        │  pstop msg
        ▼
      MACHINE  ──(native checks: counter +1, stamp monotonic, bounded loss)──┐
        │  resp.received_counter = req.counter                               │
        │  resp.received_stamp   = req.stamp   (black-channel echo)          │
        ▼                                                                    │
   REMOTE verifies: my counter/stamp (and the echo signature) returned ◀────┘
        │  only then →
        ▼
   advance token / emit next OK     ── else STOP (chain stalled anywhere)
```

### 5.1 Dynamic challenge–response loopback (fixes G3)
Replace `counter&1` with a per-core rolling pseudo-random challenge
(LFSR seeded per core), optionally multi-bit per tick — a genuine short
serial loopback — and verify the full echo each tick.

### 5.2 Liveness accumulator — no bypass (fixes G2)
Maintain a per-core accumulator that *is* the safety value:
`S ^= (Cn ^ En)` — zero contribution iff the echo matched. `OK` is gated
on `S == seed` AND a freshness counter that must advance every tick.
Emitting `OK` provably requires a history of correct live reads; there is
no constant `OK` to fall through to, and the data dependency stops the
compiler from hoisting/caching the read.

### 5.3 Splice the GPIO into the black-channel round-trip (fixes G4)
Using only existing wire fields (pstop_c unchanged):
- **Counter backbone (machine-enforced).** Gate the counter advance on a
  correct echo: a GPIO fault stops counter progress → the **machine's
  own** `MSG_LOST`/heartbeat logic trips to STOP, with zero machine
  changes.
- **Stamp signature (freshness/anti-replay).** Carry a rolling signature
  of the measured echoes in the low bits of `stamp` (kept strictly
  increasing). The machine echoes it verbatim in `received_stamp`; the
  remote checks the returned signature against the challenge sequence it
  drove — proving the machine acted on the exact, current GPIO-derived
  message.

### 5.4 Dual-core diversity + comparator contract (fixes G1 — the crux)
Make the two channels diverse by construction: core 0 active-high sense,
challenge seed A, GPIO data-register read; core 1 active-low, seed B,
driver-API read. A systematic bug in one encoding does not reproduce
identically in the other. This forces the comparator from byte-identical
`memcmp` to **semantic 1oo2D**: each core emits {verdict, liveness
proof, fresh sequence}; the comparator checks they agree on meaning AND
each proof is valid+fresh, then emits the single canonical pstop message.
Any disagreement, stale proof, or missing proof → send nothing → STOP.

> **[Reconciled 2026-08-02] — Framing correction.** The implemented comparator
> is a **byte-identical `memcmp`** of the two 40-byte encodings (`main.c:890`),
> **not** a semantic 1oo2D. This is **sufficient by design, not a skipped
> control**: **Option B** makes the two cores emit **byte-identical** encodings
> when healthy (core 0 arithmetic image `k_estop_msg[((rb_hi^1)|rb_lo)&1]` `:305`
> vs core 1 boolean `(rb_hi==1)&&(rb_lo==0)` `:307` — the *same* OK/STOP codeword
> when correct), and any caught fault makes one core's bytes **differ**, which
> the memcmp catches. The semantic comparator was therefore rendered
> **unnecessary**, not omitted. **Do not overclaim:** this rests on the two
> source expressions staying non-convergent through one toolchain/ISA, and the
> shared code *after* codeword selection (override/debounce, encode) is β≈1.
> Common-cause coverage is **not quantified** — it requires FMEDA **and** the
> §10 row-9 fault-injection proof (HARA §6; FMEA DU-3/DU-5).

### 5.5 Supporting hardware diagnostics (raise DC) — **PARTIALLY IMPLEMENTED**
> **[Reconciled 2026-08-07]** The **GPIO config-integrity check is now
> IMPLEMENTED** (SR-R-09 / FMEA **DU-1 CLOSED**, commit `1cfbd9a`): the remote
> periodically re-reads each channel's pad direction/mux/pulldown
> (`gpio_ll_get_io_config`) plus a drive-low self-test and forces a controlled
> reset on mismatch (`gpio_cfg_fault`) — closing the dropped-pulldown → false-OK
> path. See `docs/safety/CLOCK_GUARD_AND_GPIO_REVERIFY.md`. Residual: on-bench
> corruption of a *live* pad config is bench-blocked (logic host-reasoned).
>
> **[Reconciled 2026-08-02, superseded above for GPIO config-integrity]** The
> other §5.5 diagnostics below (output read-back, cross-channel read) are still
> **NOT IMPLEMENTED** — do not cite them as existing controls.
- GPIO config integrity: periodically re-read/verify pin direction, mux,
  and the input pulldown (a dropped pulldown is a classic
  dangerous-undetected fault: an open loop would float, not read 0).
- Output read-back: confirm the OUT pin drives what was set.
- Cross-channel read: each core also reads the other's IN (read-only)
  and flags inconsistency.
- Watchdog coupled to the proof: kick a dedicated WDT only on a
  completed, fresh, correct loop+round-trip cycle.
- Physical independence: separate pins/poles (done); ensure the channels
  cannot short together to fake both closed; prefer separate IO banks.

### 5.6 Fail-safe and timing
- Safe state = STOP = de-energized (no message → machine
  heartbeat-timeout).
- Diagnostic test interval = one 10 Hz tick (100 ms). Detection→STOP is
  bounded by the machine heartbeat timeout =
  `heartbeat_ms × max_missed_heartbeats` — today 400 × 5 ≈ **2.0 s**
  (`machine.c:290-298`; defaults per SYSTEM_DEFINITION §2).
  <!-- [Updated 2026-08-04] max_missed_heartbeats raised 3 → 5 (was 400 × 3 ≈
  1.2 s) to ride through ~90-min Tailscale control-plane re-syncs (transient
  ~1.4 s RTT spikes) that were tripping false STOPs; trades detection latency
  (1.2 s → 2.0 s) for far fewer nuisance stops. FTTI/PST budget must now cover
  ~2.0 s + one tick ≈ 2.1 s (HARA A-05, SR-SYS-01). -->
  <!-- [Reconciled 2026-08-02] Was `heartbeat_ms × (max_missed_heartbeats + 1)`,
  "1000 × 2 ≈ 1–2 s" — stale formula AND values; corrected to the library math
  (OK while diff ≤ hb; STOP when diff/hb ≥ max_missed). See RECONCILIATION R-03. -->
- Machine-side liveness is the library's own `check_heartbeats` on the
  machine's own `CLOCK_MONOTONIC` (`get_time_cb`). There is no wrapper
  watchdog and no "follow the remote's clock" option — the library never
  compares the remote's stamp (it only echoes it), and following it
  would freeze the machine clock during silence, disabling the watchdog.

## 6. Fault → reaction map (target state) — **Option-C mechanisms DROPPED**

> **[Reconciled 2026-08-02]** The "Detected by" column cites **Option C
> mechanisms that are not in the code**: the `S` liveness accumulator, the
> low-/high-phase *challenge* mismatch, the *config-integrity check*, and the
> *stamp signature* were **dropped 2026-07** or **never implemented**. The STOP
> *reactions* are still correct; read the *detection* as: open loop / stuck pin →
> **Option A both-phase mismatch** → cores diverge → `memcmp` → send nothing →
> machine heartbeat-timeout STOP; systematic sensing bug → **Option B diverse
> expressions** diverge; stale/replayed packet → **not** a stamp signature but
> `pstop_c` native counter-monotonic + `MSG_LOST`/`OUT_OF_ORDER` (F-P-04);
> pulldown lost → **currently UNDETECTED (FMEA DU-1)**, *not* caught by any
> config-integrity check.

| Fault | Detected by | Reaction |
|-------|-------------|----------|
| One loop opens (press / broken wire) | echo≠challenge → `S` corrupts; cores diverge | STOP (purple ring) |
| Pin stuck high | low-phase challenge mismatches | STOP |
| Pin stuck low / pulldown lost | high-phase mismatch; config-integrity check | STOP |
| Sensing task frozen | freshness counter stalls; comparator publish-timeout | STOP |
| Verdict branch bypassed | no branch — `OK` is a function of `S` | cannot produce OK |
| Stale/replayed packet | stamp signature won't match current round-trip | STOP |
| Packet loss / dead machine | counter stalls → machine MSG_LOST/heartbeat | STOP (machine-side) |
| Random RAM/ALU corruption in one core | semantic cross-check diverges | STOP |
| **Systematic SW bug in sensing** | **diverse cores don't fail identically** | STOP |

## 7. Residual limitations (state in the safety case)

> **[Reconciled 2026-08-02]** This section is written around the **dropped
> Option C** (`En==Cn` challenge/echo, stamp echo-signature); those specific
> mechanisms are **not implemented**. The *structural* residual it names is real
> and now larger: the machine runs **unchanged** `pstop_c` and cannot recompute
> the GPIO verdict; remote-side correctness rests on **Option A/B**, and
> anti-replay rests **solely** on `pstop_c` native counter/`MSG_LOST` (the stamp
> signature that would have tightened it was dropped — HARA H-06, FMEA §5).

- The machine runs **unchanged** pstop_c, so it cannot itself compute
  whether `En==Cn` (it doesn't know the challenge). The GPIO-correctness
  verdict is computed on the remote; the black channel guarantees that
  verdict's freshness and round-trip integrity end-to-end, and the
  machine's native counter/heartbeat checks enforce STOP when the
  GPIO-gated chain stalls. Full machine-side recomputation would require
  a protocol/shared-secret extension — out of scope while pstop_c is
  frozen. A faulty *remote* is covered by §5.4 diversity, not by the
  machine.
- Embedding the echo signature in the remote's `stamp` perturbs only its
  low bits and must be proven to keep the stamp strictly increasing (the
  remote stamps with esp_timer uptime; the machine merely echoes it).

## 8. Open decisions (condensed worksheet)

Recommended defaults marked ⭐. None of these change `pstop_c`.

> **[Reconciled 2026-08-02]** **These decisions are resolved and the ⭐ marks are
> stale.** The A1/A2/A3 LFSR-challenge family, B1/B2 liveness accumulator,
> C1/C2/C3 stamp-signature family, D1 semantic-1oo2D comparator, and E1 GPIO
> config-integrity were **NOT adopted (Option C, dropped 2026-07)**. What *was*
> adopted: **Option A** both-phase sampling (supersedes A1/A2/A3), **Option B**
> source-diverse verdict expressions with **byte `memcmp`** (supersedes D1), and
> machine-side `min_stop_ms` arming (adjacent to B3). **E1 config-integrity
> remains an open gap (FMEA DU-1)**, not a shipped default. Rows kept for
> decision traceability.

| # | Decision | Options (⭐ = recommended) |
|---|---|---|
| A1 | Challenge generator | keep `counter&1` / ⭐ per-core 16-bit Galois LFSR (esp_random seed) / per-tick esp_random |
| A2 | Challenge width per tick | 1 bit / ⭐ 4-bit serial burst (<100 µs of the tick) / 8-bit+ |
| A3 | Settle time per bit | ⭐ 10 µs (current, bench wiring) / 25–50 µs long harness / measure |
| B1 | Liveness accumulator | ⭐ 32-bit sticky XOR accumulator + fault flag / per-tick compare only |
| B2 | Freshness | ⭐ monotonic per-core read sequence checked by comparator / publish-timeout only |
| B3 | Fault latch | ⭐ latch STOP until operator re-arm (press→release) / until reboot / auto-clear (not advised) |
| C1 | On bad echo | stop sending (timeout) / ⭐ send explicit STOP + hold counter discipline / freeze counter (MSG_LOST path) |
| C2 | Stamp echo-signature | ⭐ yes, N=8 low bits, verified on `received_stamp` / counter-gating only |
| C3 | Signature content | ⭐ running CRC/hash of echo bursts / latest tick's echo only |
| D1 | Comparator contract | byte-identical memcmp (status quo) / ⭐ semantic 1oo2D |
| D2 | Diversity set | ⭐ inverted sense + different read path + different LFSR seed & polynomial; optionally different accumulator algorithm, drive polarity |
| D3 | Cross-channel read | ⭐ include / skip |
| E1 | GPIO config integrity | ⭐ re-verify every ~1 s (mismatch → STOP) / blind re-assert / skip |
| E2 | Output read-back | ⭐ include in accumulator / skip |
| E3 | Watchdog coupling | ⭐ dedicated safety WDT kicked only on a valid cycle (~750 ms) / reuse TWDT / none |
| F1 | Reaction-time budget | **needs the robot's numbers**: process safety time, required switch-press→machine-STOP latency, tick rate, heartbeat timeout |
| F2 | SC 2 process scope | MISRA C (done — see `MISRA_COMPLIANCE_2026-07-21.md`), static analysis in CI, unit + fault-injection tests, traceability, FMEDA |
| G1 | Electrical independence | ⭐ separate GPIO banks; ⭐ routing that cannot short channels together; separate pull domains; series protection |
| G2 | Sense idle level | pulldown today (open reads 0 = safe); with inverted logic (D2) channel B may need a pull-up so its open state is also safe |

## 9. IEC 61508 mapping (for the assessor)

> **[Reconciled 2026-08-02]** Integrity target is now **SIL 3 / PL e** (was
> SIL2). The DC bullet's "challenge–response loopback + config-integrity +
> cross-read" describes **dropped/unimplemented** Option C mechanisms; the actual
> online diagnostic is the **Option A both-phase loopback per tick** plus the
> round-trip counter/heartbeat checks. **Config-integrity and cross-channel read
> are NOT implemented.** The β bullet's core-diversity lever is **Option B
> (source-level only)**, not the §5.4 semantic comparator. **No FMEDA exists**, so
> DC/SFF/β/PFH are unquantified — no numeric SIL claim is yet supported (HARA §6).

- **Architecture:** 1oo2D for the stop function (either channel trips),
  2oo2 to enable. HFT = 1.
- **Diagnostic coverage:** continuous online challenge–response loopback
  (every tick) + config-integrity + cross-read + round-trip verification
  → target medium/high DC. Quantify per FMEDA.
- **Common-cause (β):** core diversity (§5.4) + physical channel
  independence (§5.5) — the single biggest SIL2 lever here.
- **Safe failure fraction / safe state:** de-energize to STOP on every
  detected fault; STOP is the protocol default (no message).
- **Proof test:** the per-tick loopback is an online proof test; document
  the interval vs PST.
- **Systematic capability (SC 2):** a lifecycle obligation on the *new*
  sensing/comparator code. `pstop_c` is leveraged as a
  validated/unchanged component and must not be modified (which is also
  why the integrity chain is built *around* it).

## 10. Verification / fault-injection plan

> **[Reconciled 2026-08-02]** Two rows test **dropped/unimplemented** features.
> **Row 7** (replay → *stamp-signature* mismatch): the stamp signature was
> **dropped**; re-scope to the real anti-replay = `pstop_c` counter/`MSG_LOST`.
> **Row 8** (corrupt GPIO pulldown config → *integrity check* → STOP): that check
> is **NOT IMPLEMENTED (FMEA DU-1)** — today this fault is **undetected**; keep
> the row as a *gap-demonstration* until the check exists. Rows 1–6 and **row 9
> (inject an identical logic fault into both cores' sensing — the Option B
> diversity proof)** remain the live plan; **row 9 is a gating SIL 3 evidence
> item** (HARA §7).

Each row is a bench test (observable via the LED ring + `/state.json` +
`machine_app_runner -v`):

1. Open channel A only (pull IO40) → cores diverge → **purple**, machine STOP.
2. Open channel B only (pull IO42) → same, opposite channel.
3. Short an IN pin high → stuck-high caught by low phase → STOP.
4. Freeze one core (debug stub) → publish-timeout/freshness → STOP.
5. Drop/garble packets (netem) → counter stall → machine MSG_LOST → STOP.
6. Kill the machine → heartbeat timeout → STOP (yellow ring).
7. Replay an old packet → stamp-signature mismatch → STOP.
8. Corrupt the GPIO pulldown config at runtime → integrity check → STOP.
9. Inject an identical logic fault into both cores' sensing → diversity
   must still diverge; if it doesn't, the diversity is insufficient.

Each must reach STOP within the reaction-time budget (§5.6 / F1).
