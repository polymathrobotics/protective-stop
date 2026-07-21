# Dual-Core E-Stop Integrity Chain — Safety Design (SIL2 target)

**Status:** DRAFT for discussion — no code yet.
**Standard frame:** IEC 61508 (SIL2, HFT=1, de-energize-to-safe).
**Scope:** the remote (ESP32 dual_core_safety) E-stop sensing → verdict →
pstop message path. `pstop_c` (machine *and* remote protocol library) stays
byte-for-byte unchanged; everything here lives in the application shell
(`main.c`, the comparator, the GPIO sensing) around it.

---

## 1. The property we must guarantee

> There is **no viable execution path** in which the pstop message says `OK`
> while the physical E-stop loop is open, stuck, or unread.

Equivalently: a sustained `OK` reaching the machine must be *causally and
freshly* derived from a correct, live read of the button loop on **both**
independent channels. Any fault — open loop, stuck pin, frozen task, bypassed
branch, lost packet, dead machine — must drive the system to STOP within the
process safety time.

---

## 2. Correction to an earlier claim: the black channel is end-to-end

An earlier note asserted the safety argument "ends at the remote" because the
machine can't see the GPIO. **That was wrong.** pstop_c is explicitly a
black-channel protocol:

- The machine echoes the remote's own data back every reply
  (`pstop_c/.../protocol.c:118-119`):
  ```c
  resp->received_counter = req->counter;   // remote's counter, echoed
  resp->received_stamp   = req->stamp;     // remote's stamp, echoed
  ```
- The machine also *validates* the reverse direction — it checks that the
  remote correctly echoed the machine's counter/stamp, with bounded loss and
  monotonic time (`protocol.c:26-46`: `MSG_LOST`, `OUT_OF_ORDER`).

So **both** ends can confirm their own data made the full round trip, and the
machine's *native* checks already trip to STOP if the remote's
counter/stamp sequence stalls or regresses. That gives us a closed loop —
remote → machine → remote — that we can **splice the GPIO into**, so a GPIO
fault breaks a chain the *machine* polices, not just the remote.

---

## 3. Today's design and its gaps

Current: each core drives `counter&1` on its loop, reads the echo, returns a
bool verdict; the comparator byte-compares the two cores' encoded messages and
sends only on agreement (2oo2-to-run / 1oo2-to-stop, de-energize to STOP).

| # | Gap | Why it matters at SIL2 |
|---|-----|------------------------|
| G1 | **Common-cause / systematic SW faults.** Two cores run identical code → an identical bug yields identical wrong `OK` → comparator agrees. | Byte-compare gives ~0 systematic-fault coverage (β≈1). This is usually the dominant SIL2 risk. |
| G2 | **Bypassable verdict.** `closed ? OK : STOP` is a branch; a wild write / optimizer / logic bug can reach `OK` without a read. | Defeats "OK ⟺ correct read." |
| G3 | **Predictable challenge.** `counter&1` is a known toggle; a fault that happens to track it isn't forced to reveal itself. | Lowers diagnostic coverage (DC). |
| G4 | **Loose coupling to the message.** The verdict is computed *beside* the message, not woven into the data that round-trips. | A stale/forged value can ride the protocol. |

---

## 4. The integrity-chain architecture

One token must circulate the **entire** physical+logical loop every cycle, and
must come all the way back, fresh and correct, to keep `OK` alive:

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
        ▼                                                                     │
   REMOTE verifies: my counter/stamp (and the echo signature) returned ◀─────┘
        │  only then →
        ▼
   advance token / emit next OK     ── else STOP (chain stalled anywhere)
```

The four mechanisms that make this hold:

### 4.1 Dynamic challenge–response loopback (fixes G3)

Replace `counter&1` with a per-core **rolling pseudo-random challenge** (LFSR
seeded per core). Optionally drive a *multi-bit* challenge per tick — i.e. a
genuine short serial loopback per tick — for much higher entropy per
measurement and near-zero chance of a fault "accidentally" matching. Verify the
full echo each tick. (Direct answer to the earlier "is it serial?" question:
today it is a 1-bit DC level; for SIL2 we make it a deliberate
challenge–response sequence.)

### 4.2 Liveness accumulator — no bypass (fixes G2)

Don't compute a boolean beside the message. Maintain a per-core accumulator
that *is* the safety value:

```
S ^= (Cn ^ En)         // 0 contribution iff En==Cn (loop intact) this tick
```

`S` stays at its known-good seed **only** if every tick's echo matched its
challenge. Any open/stuck/stale/skipped read injects non-zero into `S` and it
never self-heals. `OK` is gated on `S == seed` AND a freshness counter that
must advance every tick. Result: emitting `OK` provably requires a *history of
correct live reads*; there is no constant `OK` to fall through to, and the
data dependency also stops the compiler from hoisting/caching the read.

### 4.3 Splice the GPIO into the black-channel round-trip (fixes G4; uses §2)

Two couplings, both using only existing wire fields (pstop_c unchanged):

- **Counter backbone (machine-enforced).** Gate the counter advance on a
  correct echo: the remote only produces the next counter when the loopback was
  good. A GPIO fault → counter stops advancing → the **machine's own**
  `MSG_LOST` / heartbeat logic trips to STOP. This is what puts the GPIO "in the
  chain the machine polices," with zero machine changes.
- **Stamp signature (freshness/anti-replay).** Carry a rolling signature of the
  *measured echoes* in the low bits of `stamp` (kept strictly increasing so the
  machine's monotonic check and the bench clock-anchor still hold). The machine
  echoes it back verbatim in `received_stamp`; the remote checks the returned
  signature against the challenge sequence it drove. This proves the machine
  acted on the *exact, current* GPIO-derived message — not a stale or injected
  packet masking a present fault.

Net: a break **anywhere** — loop open, pin stuck, packet lost, machine dead —
stalls the circulating token. Remote → STOP; machine → STOP via native checks.

### 4.4 Dual-core diversity + comparator contract (fixes G1 — the SIL2 crux)

Identical cores make G1 a common-cause hole. Make the two channels **diverse by
construction**:

- core 0: active-**high** sense, challenge seed A, read via GPIO data register;
- core 1: active-**low** (inverted) sense, challenge seed B, read via driver API.

A systematic bug in one encoding does not reproduce identically in the other.
This forces a comparator change:

- **Today:** byte-identical `memcmp` (great for *random* faults, blind to
  systematic). Incompatible with diversity.
- **Proposed:** **semantic 1oo2D.** Each core independently produces a
  loop-coupled, fresh-verified verdict (its own `S`, its own diverse encoding +
  token). The comparator checks they *agree on meaning* and that *each* carries
  a valid, fresh liveness proof, then emits the single canonical pstop message.
  Any disagreement, any stale proof, any missing proof → send nothing → STOP.

Recommendation: **semantic 1oo2D**. It keeps "exactly one message out," keeps
de-energize-to-STOP, and is the only option that gives real systematic-fault
detection.

### 4.5 Supporting hardware diagnostics (raise DC)

- **GPIO config integrity:** periodically re-assert/verify pin direction, mux,
  and the input **pulldown** — a corrupted IO register that drops the pulldown
  is a classic dangerous-undetected fault (open loop would float, not read 0).
- **Output read-back:** confirm the OUT pin actually drives what was set.
- **Cross-channel read:** each core also reads the *other's* IN (read-only) and
  flags inconsistency — diagnoses a core-local sensing fault.
- **Watchdog coupled to the proof:** kick the HW watchdog **only** on a
  completed, fresh, correct loop+round-trip cycle. A hung/bypassed path → no
  kick → reset → STOP.
- **Physical independence:** separate pins/poles (done); ensure the two channels
  cannot be shorted together (routing/connector) to fake both at once; prefer
  separate IO banks / pull domains.

### 4.6 Fail-safe and timing

- Safe state = STOP = de-energized (no message → machine heartbeat-timeout).
- Diagnostic test interval = one 10 Hz tick (100 ms). Detection→STOP must fit
  inside the process safety time and is bounded by the machine heartbeat
  timeout, = `heartbeat_ms × (max_missed_heartbeats + 1)` (configurable). State
  both numbers in the safety case.
- **Machine-side liveness is the library's own logic, on a monotonic clock.**
  The machine always uses its own `CLOCK_MONOTONIC` as `get_time_cb`; pstop_c's
  native `check_heartbeats` (`machine.c`) detects remote silence, issues the
  STOP, and clears the remote for re-bond. There is no wrapper watchdog and no
  "follow the remote's clock" option — the library never compares the remote's
  stamp (it only echoes it, `protocol.c:119`), so following it was unnecessary
  and would have frozen the machine clock during silence, disabling the very
  watchdog we rely on. Liveness thus rests on validated library logic, not glue.

---

## 5. Why this leaves no false-`OK` path

| Fault | Detected by | Reaction |
|-------|-------------|----------|
| One loop opens (press / broken wire) | echo≠challenge → `S` corrupts; cores diverge | STOP (purple ring) |
| Pin stuck high | low-phase challenge mismatches | STOP |
| Pin stuck low / pulldown lost | high-phase mismatch; config-integrity check | STOP |
| Sensing task frozen | freshness counter stalls; comparator publish-timeout | STOP |
| Verdict branch bypassed | no branch — `OK` is a function of `S` | cannot produce OK |
| Stale/replayed packet | stamp signature won't match current challenge round-trip | STOP |
| Packet loss / dead machine | counter stalls → machine MSG_LOST/heartbeat | STOP (machine-side) |
| Random RAM/ALU corruption in one core | semantic cross-check diverges | STOP |
| **Systematic SW bug in sensing** | **diverse cores don't fail identically** | STOP |

---

## 6. Residual limitations (state in the safety case)

- The machine runs **unchanged** pstop_c, so it cannot itself *compute* whether
  `En==Cn` (it doesn't know the challenge). The GPIO-correctness **verdict** is
  computed on the remote; the black channel guarantees that verdict's
  **freshness and round-trip integrity end-to-end**, and the machine's native
  counter/heartbeat checks enforce STOP when the GPIO-gated chain stalls. Full
  *machine-side* recomputation of the loop would require a protocol/shared-secret
  extension — out of scope while pstop_c is frozen. Document this as the boundary
  of machine-side coverage; a faulty *remote* is covered by §4.4 diversity, not
  by the machine.
- Embedding the echo signature in the remote's `stamp` perturbs only its low
  bits and must be proven to keep that stamp strictly increasing (the remote
  stamps with esp_timer uptime). The machine merely echoes the remote's stamp
  (`protocol.c:119`) and runs its own monotonic clock, so there is no
  machine-side clock mode to reconcile — see Open Decisions.

---

## 7. Constraints

- **`pstop_c` unchanged**, machine and remote: only the existing wire fields
  (`counter`, `received_counter`, `stamp`, `received_stamp`, message type) are
  used. No new fields, no protocol version bump.
- The remote stays hand-written around `pstop_msg` + `protocol_data_t`
  (upstream provides no remote driver; this mirrors `examples/client`).
- Exactly one pstop message per tick, gated by the comparator.

---

## 8. Open decisions (need your call)

1. **Counter-only vs counter + stamp-signature.** Counter-gating alone gives
   machine-enforced stall-to-STOP and is simplest. Adding the stamp signature
   buys explicit anti-replay/freshness proof at the cost of stamp-encoding care.
   *(Lean: do both; they're complementary.)*
2. **Byte-identical vs semantic 1oo2D comparator.** Diversity (§4.4) requires
   semantic. *(Lean: semantic.)*
3. **Diversity scheme.** Inverted logic + different read path + different seed —
   confirm that's acceptable, or specify a stronger split (e.g., different
   challenge generators).
4. **Challenge width.** 1 bit/tick (minimal) vs multi-bit serial/tick (higher
   DC). *(Lean: a few bits/tick.)*
5. **Watchdog policy.** Add a HW watchdog kicked only on a valid full cycle?
6. **Process-safety-time / reaction-time budget** — what's the machine's
   required stop latency? Sets the heartbeat timeout and tick rate.

---

## 9. IEC 61508 mapping (for the assessor)

- **Architecture:** 1oo2D for the stop function (either channel trips),
  2oo2 to enable (both must agree to run). HFT = 1.
- **Diagnostic coverage:** continuous online challenge–response loopback
  (every tick) + config-integrity + cross-read + round-trip verification →
  target medium/high DC. Quantify per FMEDA.
- **Common-cause (β):** addressed by core diversity (§4.4) + physical channel
  independence (§4.5). The single biggest SIL2 lever here.
- **Safe failure fraction / safe state:** de-energize to STOP on every detected
  fault; STOP is the protocol default (no message).
- **Proof test:** the per-tick loopback is an online proof test; document the
  interval vs PST.
- **Systematic capability (SC 2):** this is a lifecycle obligation on the *new*
  sensing/comparator code — MISRA C, static analysis, unit + fault-injection
  tests, requirements traceability. `pstop_c` is leveraged as a
  validated/unchanged component and must not be modified (which is also why the
  integrity chain is built *around* it).

---

## 10. Verification / fault-injection plan

Each row is a bench test (observable via the LED ring + `/state.json` +
`machine_app_runner -v`):

1. Open channel A only (pull GPIO40) → cores diverge → **purple**, machine STOP.
2. Open channel B only (pull GPIO42) → same, opposite channel.
3. Short an IN pin high → stuck-high caught by low phase → STOP.
4. Freeze one core (debug stub) → publish-timeout/freshness → STOP.
5. Drop/garble packets (netem) → counter stall → machine MSG_LOST → STOP.
6. Kill the machine → heartbeat timeout → STOP (yellow).
7. Replay an old packet → stamp-signature mismatch → STOP.
8. Corrupt the GPIO pulldown config at runtime → integrity check → STOP.
9. Inject an identical logic fault into both cores' sensing → diversity should
   still diverge (validates §4.4); if it doesn't, the diversity is insufficient.

Each must reach STOP within the reaction-time budget (§4.6).
