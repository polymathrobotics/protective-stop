# Protective-Stop (pstop) Integrity Chain — Safety Design

**Status:** design record. §2 describes what is implemented today
(verified against `firmware/main/main.c`); §4–§8 are the SIL2 hardening
roadmap, still awaiting decisions. Merged from the former
ESTOP_SAFETY_DESIGN.md + ESTOP_SAFETY_DECISIONS.md.
**Standard frame:** IEC 61508 (SIL2 target, HFT=1, de-energize-to-safe).
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

- **Dual-channel loopback.** Core 0 owns channel A (GPIO39 drive →
  DPST pole 1 → GPIO40 sense); core 1 owns channel B (GPIO41 →
  pole 2 → GPIO42). Sense inputs are pulled DOWN, so an open loop
  reads 0 (STOP) — fail-safe idle.
- **Rolling drive level.** Each tick the owning core drives its OUT pin
  with the tick counter's LSB (10 µs settle, `ESTOP_SETTLE_US`) and
  verifies the echo. The loop counts as closed only when the most recent
  drive-high echoed high (continuity) AND the most recent drive-low
  echoed low (no stuck-high short). Until both phases have been sampled
  the channel reads OPEN.
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

## 5. Proposed integrity-chain architecture

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

### 5.5 Supporting hardware diagnostics (raise DC)
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
  `heartbeat_ms × (max_missed_heartbeats + 1)` — today 1000 × 2 ≈ 1–2 s.
- Machine-side liveness is the library's own `check_heartbeats` on the
  machine's own `CLOCK_MONOTONIC` (`get_time_cb`). There is no wrapper
  watchdog and no "follow the remote's clock" option — the library never
  compares the remote's stamp (it only echoes it), and following it
  would freeze the machine clock during silence, disabling the watchdog.

## 6. Fault → reaction map (target state)

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

Each row is a bench test (observable via the LED ring + `/state.json` +
`machine_app_runner -v`):

1. Open channel A only (pull GPIO40) → cores diverge → **purple**, machine STOP.
2. Open channel B only (pull GPIO42) → same, opposite channel.
3. Short an IN pin high → stuck-high caught by low phase → STOP.
4. Freeze one core (debug stub) → publish-timeout/freshness → STOP.
5. Drop/garble packets (netem) → counter stall → machine MSG_LOST → STOP.
6. Kill the machine → heartbeat timeout → STOP (yellow ring).
7. Replay an old packet → stamp-signature mismatch → STOP.
8. Corrupt the GPIO pulldown config at runtime → integrity check → STOP.
9. Inject an identical logic fault into both cores' sensing → diversity
   must still diverge; if it doesn't, the diversity is insufficient.

Each must reach STOP within the reaction-time budget (§5.6 / F1).
