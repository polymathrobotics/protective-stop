# E-Stop Integrity Chain — Decision Worksheet

Companion to **ESTOP_SAFETY_DESIGN.md**. This is the editable artifact: read
each decision, edit the **Decision:** line (tick a box `[x]`, or write your own
under "Other"), and add **Notes:** as you go. I'll implement from your marked-up
copy. Nothing here changes `pstop_c` — all of it lives in the application shell.

Legend: ⭐ = my recommended default · 💰 = effort · 🛡️ = safety/DC impact

---

## Group A — Loopback sensing

### A1. Challenge generator (per core)

**Context:** today the drive level is `counter & 1` — fully predictable, so a
fault that happens to track it isn't forced to reveal itself. We want each
core's drive value to be unpredictable-but-locally-known (the core generates it,
so it always knows what to compare the echo against).

**Options:**
- [ ] **A) Keep `counter & 1`.** 💰 none. 🛡️ poor — predictable, low DC. (Listed
  only for completeness; not advised.)
- [ ] ⭐ **B) Per-core 16-bit Galois LFSR**, seeded at boot from `esp_random()`,
  then advanced deterministically each tick. 💰 tiny. 🛡️ good — pattern is
  unpredictable across boots and to a stuck fault, yet fully reproducible by the
  owning core for echo comparison and signature recompute. Different
  seed/polynomial per core feeds the diversity story (D2).
- [ ] **C) Per-tick `esp_random()` directly.** 💰 tiny. 🛡️ good unpredictability
  but no clean reproducible sequence for the round-trip signature; you'd have to
  store every challenge. LFSR gives the same unpredictability with reproducibility.

**Decision:** ☐ A ☐ B ⭐ ☐ C ☐ Other: __________
**Notes:** _______________________________________________

### A2. Challenge width per tick (1 bit vs serial burst)

**Context:** more bits exercised per tick = more chances to catch a partial
fault = higher diagnostic coverage. Each bit is `drive → settle (~10 µs) → read`.

**Options:**
- [ ] **A) 1 bit/tick.** 💰 none. 🛡️ low DC (a single high/low compare/tick).
- [ ] ⭐ **B) 4-bit serial burst/tick** (~4 × (10 µs settle + read) ≈ <100 µs of
  the 100 ms tick). 🛡️ 16 patterns/tick, catches stuck/partial faults fast;
  negligible CPU. This is the "make it a real serial loopback" answer.
- [ ] **C) 8-bit (or wider) burst/tick.** 💰 still tiny. 🛡️ highest DC; more
  margin for the FMEDA. Slightly longer busy-wait per tick.

**Decision:** ☐ A ⭐ ☐ B ☐ C(width=__) ☐ Other: __________
**Notes:** _______________________________________________

### A3. Settle time per bit

**Context:** the loop is a wire + switch contacts; propagation is sub-µs, but
long harnesses + the pulldown RC need margin. Currently `ESTOP_SETTLE_US = 10`.

**Options:**
- [ ] ⭐ **A) 10 µs** (current). Fine for short bench wiring.
- [ ] **B) 25–50 µs** if the deployed harness is long / capacitive.
- [ ] **C) Measure it** (drive an edge, time the echo) and set with margin.

**Decision:** ☐ A ⭐ ☐ B(__ µs) ☐ C ☐ Other: __________
**Notes:** _______________________________________________

---

## Group B — Verdict integrity (no-bypass)

### B1. Liveness accumulator

**Context:** replace the bypassable `closed ? OK : STOP` branch with a value
that *is* the safety state. Proposed (per core):

```
each bit b in the tick's burst:  acc ^= (challenge_b ^ echo_b)   // 0 iff equal
fault if acc != seed at end of tick      // any miscompare ever → non-zero, sticky
```

`OK` is emitted only when `acc == seed` AND the freshness counter (B2) advanced.

**Options:**
- [ ] ⭐ **A) 32-bit sticky XOR accumulator + sticky fault flag** as above.
  💰 tiny. 🛡️ strong: `OK` becomes a pure function of correct live reads; no
  branch to shortcut; data dependency defeats compiler hoisting.
- [ ] **B) Per-tick compare only (no accumulator).** 💰 tiny. 🛡️ weaker — loses
  the "history of correct reads" property; a one-tick glitch wouldn't persist.

**Decision:** ☐ A ⭐ ☐ B ☐ Other: __________
**Notes:** _______________________________________________

### B2. Freshness mechanism

**Context:** prove the read is *this tick's*, not stale/frozen.

**Options:**
- [ ] ⭐ **A) Monotonic per-core read sequence**, checked by the comparator to
  have advanced every tick (a frozen core → no advance → STOP). Composs with the
  existing 80 ms publish-timeout.
- [ ] **B) Rely only on the existing publish-timeout.** 🛡️ weaker (catches a
  fully-hung task, not a task that runs but reuses a stale sample).

**Decision:** ☐ A ⭐ ☐ B ☐ Other: __________
**Notes:** _______________________________________________

### B3. Fault latch / re-arm policy

**Context:** when a sensing fault is detected, does it auto-clear when the fault
goes away, or latch until a deliberate operator action?

**Options:**
- [ ] ⭐ **A) Latch STOP until operator re-arm** (the same press→release that
  arms after bond). Consistent with the NEED_STOP philosophy: a sensing fault
  must not silently self-clear. 🛡️ strongest.
- [ ] **B) Latch until reboot.** 🛡️ strong but heavy-handed operationally.
- [ ] **C) Auto-clear when reads go good again.** 🛡️ weakest — a flapping wire
  could intermittently re-enable; not advised for SIL2.

**Decision:** ☐ A ⭐ ☐ B ☐ C ☐ Other: __________
**Notes:** _______________________________________________

---

## Group C — Splicing GPIO into the black channel

### C1. Counter-advance gating (machine-enforced stall→STOP)

**Context:** gate the pstop `counter` increment on a correct echo, so a GPIO
fault stops counter progress and the machine's **native** checks react. What
should the remote do on a bad echo?

**Options:**
- [ ] **A) Stop sending entirely** → machine heartbeat-times-out (≤ heartbeat
  timeout, e.g. 1 s). Simple; slower reaction.
- [ ] ⭐ **B) Send an explicit `STOP`** (verdict STOP) immediately, *and* hold the
  counter discipline. 🛡️ fastest machine reaction (STOP on the next packet, not
  a timeout); still degrades to timeout if packets are lost.
- [ ] **C) Freeze the counter (resend last)** → machine sees no progress →
  MSG_LOST path. 🛡️ machine-enforced but reaction depends on `max_lost_messages`.

**Decision:** ☐ A ⭐ ☐ B ☐ C ☐ Other: __________
**Notes:** _______________________________________________

### C2. Stamp echo-signature (anti-replay / freshness proof)

**Context:** carry a rolling signature of the *measured echoes* in the low bits
of `stamp`; the machine echoes it verbatim in `received_stamp`; the remote
verifies the returned signature against the challenge sequence it drove. Proves
the machine acted on the exact, current GPIO-derived message (not a stale or
injected packet). The carrier is the REMOTE's `stamp` (esp_timer uptime); it
must stay strictly increasing. The machine merely echoes it (`received_stamp`)
and runs its own monotonic clock, so there is no machine-side clock mode to
worry about.

**Options:**
- [ ] ⭐ **A) Yes — embed an N-bit rolling signature in `stamp[low N]`** (proposed
  N = 8), high bits remain the remote's monotonic uptime. Verify on
  `received_stamp`. 💰 small (encode + monotonic guard + verify). 🛡️ adds
  explicit end-to-end freshness/anti-replay on top of C1.
- [ ] **B) Counter-gating only (skip the stamp signature).** 💰 least. 🛡️ relies
  on the counter chain alone for end-to-end coverage; no explicit anti-replay.

**Decision:** ☐ A ⭐ (N=__) ☐ B ☐ Other: __________
**Notes:** _______________________________________________

### C3. Signature content

**Context:** what the rolling signature is computed over.

**Options:**
- [ ] ⭐ **A) Running CRC/hash of the echo bursts** (per core, folded). Detects
  any divergence over the window.
- [ ] **B) Just the latest tick's echo.** 🛡️ weaker window.

**Decision:** ☐ A ⭐ ☐ B ☐ Other: __________
**Notes:** _______________________________________________

---

## Group D — Redundancy & common-cause (the SIL2 crux)

### D1. Comparator contract

**Context:** diversity (D2) requires comparing *meaning*, not bytes; today's
byte-`memcmp` can't accommodate diverse cores.

**Options:**
- [ ] **A) Keep byte-identical `memcmp`.** 💰 none. 🛡️ great for random faults,
  ~0 for systematic. Forces identical cores → leaves G1 open. Only viable if you
  accept covering systematic faults purely by process (MISRA/V&V).
- [ ] ⭐ **B) Semantic 1oo2D.** Each core emits {verdict, liveness-proof, fresh
  seq}; comparator checks they *agree on meaning* AND each proof is valid+fresh,
  then builds the single canonical pstop message. 💰 moderate (rework comparator).
  🛡️ real systematic-fault detection; keeps one-message-out + de-energize-to-STOP.

**Decision:** ☐ A ☐ B ⭐ ☐ Other: __________
**Notes:** _______________________________________________

### D2. Diversity scheme (how different the two cores are)

**Context:** to break common-cause, the two channels should fail differently
under the same systematic bug. How far to push it?

**Options (cumulative — tick all you want):**
- [ ] ⭐ **a) Inverted sense logic** (core 0 active-high, core 1 active-low).
- [ ] ⭐ **b) Different read path** (core 0 reads the GPIO data register; core 1
  via the driver API).
- [ ] ⭐ **c) Different LFSR seed AND polynomial** per core.
- [ ] **d) Different accumulator/signature algorithm** per core (e.g. XOR-CRC
  vs additive checksum). 🛡️ higher β-reduction, 💰 more code/test.
- [ ] **e) Different drive polarity / idle level** per channel.

**Decision (set):** _________________________  ☐ Other: __________
**Notes:** _______________________________________________

### D3. Cross-channel read

**Context:** each core also reads the *other* core's IN pin (read-only) and
flags inconsistency — diagnoses a core-local sensing fault independently.

**Options:**
- [ ] ⭐ **A) Include it.** 💰 small. 🛡️ +DC on the sensing hardware.
- [ ] **B) Skip it.** Rely on the comparator's verdict cross-check only.

**Decision:** ☐ A ⭐ ☐ B ☐ Other: __________
**Notes:** _______________________________________________

---

## Group E — Hardware diagnostics

### E1. GPIO configuration integrity check

**Context:** a corrupted IO register (direction/mux/**pulldown**) could turn an
open loop into a false "closed" read. Periodically re-assert + verify the config.

**Options:**
- [ ] ⭐ **A) Re-verify every ~1 s** (re-read IO mux/dir/pull, compare to
  expected; mismatch → STOP). 💰 small. 🛡️ closes a classic dangerous-undetected
  fault.
- [ ] **B) Re-assert (blindly rewrite) every ~1 s** without read-back. 🛡️ weaker
  (no detection, just correction).
- [ ] **C) Skip.** Not advised at SIL2.

**Decision:** ☐ A ⭐ (interval=__) ☐ B ☐ C ☐ Other: __________
**Notes:** _______________________________________________

### E2. Output drive read-back

**Context:** confirm the OUT pin actually drives the level we set (catch a
driver stuck off / shorted).

**Options:**
- [ ] ⭐ **A) Read back the OUT pin's actual level each burst** (where the SoC
  exposes it) and include it in the accumulator. 💰 small.
- [ ] **B) Skip** (covered indirectly: if OUT is dead, the echo won't match).

**Decision:** ☐ A ⭐ ☐ B ☐ Other: __________
**Notes:** _______________________________________________

### E3. Watchdog coupling

**Context:** kick the hardware watchdog (TWDT) **only** on a completed, fresh,
correct loop+round-trip cycle, so a hung/bypassed path resets the chip → STOP.

**Options:**
- [ ] ⭐ **A) Dedicated safety WDT, kicked only on a valid cycle**, timeout
  ~500 ms–1 s. 💰 small. 🛡️ strong liveness backstop.
- [ ] **B) Reuse the existing app heartbeat/TWDT.** 💰 least. 🛡️ less targeted.
- [ ] **C) None.** Rely on the comparator publish-timeout only.

**Decision:** ☐ A ⭐ (timeout=__) ☐ B ☐ C ☐ Other: __________
**Notes:** _______________________________________________

---

## Group F — Timing & inputs (need your numbers)

### F1. Reaction-time budget

**Context:** sets the heartbeat timeout, tick rate, and proof-test interval, and
is the number the assessor will check. Please fill in from the robot's
requirements.

- Process Safety Time (max from hazard to safe state): **______ ms**
- Required E-stop→machine-STOP latency: **______ ms**
- pstop tick rate (today 10 Hz / 100 ms): **______ Hz**
- machine silence→STOP timeout — now the library's NATIVE check_heartbeats on a
  monotonic clock, = `heartbeat_ms × (max_missed_heartbeats+1)` (today
  1000 × 2 ≈ 1–2 s; tune via `[limits].max_missed_heartbeats` + the operator
  `heartbeat_ms`, NOT a wrapper watchdog): **______ ms**
- Diagnostic test interval (= tick): **______ ms**

**Notes:** _______________________________________________

### F2. SIL2 systematic-capability scope (process)

**Context:** SIL2 is also a lifecycle obligation on the *new* sensing/comparator
code (not on `pstop_c`, which is leveraged unchanged). Confirm which apply:

- [ ] MISRA C compliance for the new safety code
- [ ] Static analysis (e.g. cppcheck/clang-tidy/Coverity) in CI
- [ ] Unit + fault-injection tests (the §10 plan in the design doc)
- [ ] Requirements traceability for the safety functions
- [ ] FMEDA to quantify DC / SFF / β

**Decision (set):** _________________________  **Notes:** ____________

---

## Group G — Electrical / wiring (for the hardware build)

### G1. Channel independence

**Context:** software diversity is moot if the two channels share a failure.

- [ ] ⭐ **a) Separate GPIO banks** for the two channels where possible.
- [ ] ⭐ **b) Route so the two channels cannot short to each other** (a single
  short must not fake both loops closed).
- [ ] **c) Separate pull / reference domains** per channel.
- [ ] **d) Consider series resistors / protection** on the loop pins (ESD, fault
  current) per the harness environment.

**Decision (set):** _________________________  **Notes:** ____________

### G2. Sense idle level / pull direction

**Context:** today IN pins are pulled **down**, so an open loop reads 0 (safe).
With diverse inverted logic (D2a), core 1 may need a pull-**up** so its open
state is also the safe state. Confirm per the final logic choice.

**Decision:** _________________________  **Notes:** ____________

---

## Summary of my defaults (if you just want to accept the ⭐ set)

A1-B · A2-B(4-bit) · A3-A(10µs) · B1-A · B2-A · B3-A(latch→re-arm) ·
C1-B(explicit STOP) · C2-A(N=8) · C3-A · D1-B(semantic 1oo2D) ·
D2-{a,b,c} · D3-A · E1-A(1s) · E2-A · E3-A(~750ms) · F1=**you fill in** ·
G1-{a,b} · G2 per final logic.

Tell me to "build from the ⭐ defaults" or hand back this file edited, and I'll
turn §4 of the design doc into a concrete implementation plan (data structures,
the per-core diverse sense, the new comparator contract, and the
fault-injection tests) for review before writing code.
