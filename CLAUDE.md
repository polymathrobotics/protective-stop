# CLAUDE.md — Protective-Stop operating manual

Loaded automatically by Claude Code. Governs every session.

**How this file is organized.** §1 is what you must never violate — re-read it alone
at every checkpoint. §2–§3 are what you need before acting. §4–§7 are how work gets
done. §8 is lookup. Read all of it once; re-read §1 and §3 at every milestone
boundary, after any context compaction, and before each new wave of delegation. "It's
in my context" is not the test; whether you are applying it right now is.

**This repo is safety-certified and public.** Target: SIL 3 (IEC 61508) / PL e
(ISO 13849) for the on-demand stop — **allocated, not achieved**; the numeric claim
gates on FMEDA, fault-injection, and SC 3 process evidence. Two consequences: a change
to safety code is a *modification* with a procedure, and everything you write is
published, so it must be honest about claimed vs demonstrated.

---

## 1. Non-negotiables

### 1.1 Safety invariants (violating one is a defect, not a style choice)
- Safe state is **STOP = de-energized**. `OK == 0x00`, `STOP == 0x01`. Never invert;
  never add a state that defaults to running.
- Default and power-on state is STOP. OK is exceptional and continuously justified.
- The **OPEN→STOP edge is never filtered, delayed, debounced, retried, or buffered**.
  It propagates on the tick it is sampled. Only re-arm may be gated.
- Everything downstream of codeword selection is **monotonic toward STOP** — can raise
  to STOP, never lower to OK.
- OK derives **only** from a fresh, matched, both-phase read on this tick. No stale
  value, latched flag, default, or health/debounce branch may produce OK.
- The comparator transmits **only on exact agreement** of both cores' full encodings.
  Divergence, timeout, or un-primed ⇒ send nothing. **Silence is a stop.**
- **Core diversity must survive the optimizer.** The object-code diff gate proves it in
  CI. A refactor that collapses the two verdict expressions is a β≈1 common-cause
  failure.
- The transport is an **untrusted black channel**. All detection lives in the
  endpoints. No value received over the wire may widen the safe envelope.
- Timing config is validated against a compiled envelope at startup and on every
  reconfiguration, and out-of-envelope values are **refused, not silently clamped**.
  Host and ROS 2 floors are one envelope expressed twice — keep them in parity.
- Re-arm is gated on the operator allowlist, which **defaults to empty**.
- **A diagnostic that detects must act.** Detection without a forced STOP or controlled
  reset is a log line, not a diagnostic.
- **Never state a quantitative safety claim the FMEDA does not support.** SFF, DC, PFH
  and β are unquantified. A figure presented as achieved rather than allocated is a
  fabrication.

### 1.2 Process absolutes
- **No fabrication.** Never invent a statistic, API behavior, failure rate, or
  requirement. On a safety case a blank is a declared gap; a fabrication is a false
  claim.
- **Never edit the safety argument to make a change land.** Requirement text, status
  transitions, DU closures and coverage claims under `docs/safety/` are human-ratified.
- **`pstop_c/` is off-limits.** Pre-qualified, separate upstream track. Referenced at
  the interface, never edited here, never re-verified. Protocol and machine-safety
  changes go upstream and return as a version bump.
- **Never commit to `main`. Never `git push origin main`.** Every change — including a
  one-line doc fix you wrote yourself — lands via branch + PR. Public repo: history
  rewrites are unavailable. Fix forward.
- **Nothing secret ships.** No credentials, customer names, site identifiers. Scrub
  every fixture, log excerpt and capture before pushing.
- **Challenge before proceeding.** If an instruction conflicts with a requirement, an
  invariant, or sound design, say so in one line and propose the correct path first.

### 1.3 Do not build
New runtime dependency on a safety path without an explicit decision · dynamic
allocation on a verdict or transmit path · blocking I/O in the tick loop · logging that
can perturb stop-path timing · any retry/queue/buffer that could delay a STOP · a third
state between OK and STOP · re-implementation of `pstop_c` behaviour in the shell · wire
format changes not treated as a wire break · live network calls in CI · formatting
sweeps across `ros2/` (ament owns it).

---

## 2. What this is, and what governs

A wireless protective stop for mobile machines: an ESP32-S3 handheld remote with a
dual-channel E-stop loop and two cores in lockstep, a machine endpoint (ESP32 `machn`
with series relays, or a host/ROS 2 software machine), and `pstop_c` — a pre-qualified
protocol and machine-safety library — over an untrusted black channel. The hard
problems are systematic correctness, common-cause independence, and an assurance
argument that survives assessment.

**Authoritative, in precedence order** — all under `docs/safety/`:

| # | Document | Governs |
|---|---|---|
| 1 | `SYSTEM_DEFINITION.md` | System boundary, `F-xx` function decomposition |
| 2 | `HARA.md` | Hazards `H-nn`, safety goals `SG-n` |
| 3 | `SAFETY_REQUIREMENTS.md` | `SR-<area>-<nn>` — the requirements baseline |
| 4 | `FMEA.md` / `FMEDA.md` | Failure modes, `DU-n` register |
| 5 | `TRACEABILITY.md` | SR ↔ code ↔ test map, coverage position |
| 6 | `RECONCILIATION.md` | Where docs and code diverged, and how each resolved |

**The two conflict directions are different, and confusing them is dangerous.** Code
conflicting with a **requirement** ⇒ the code is a defect. Code conflicting with a
design document's **description of what is implemented** ⇒ the code wins and the
document is stale; correcting it is part of the change. If a requirement itself looks
wrong, STOP and flag it.

Everything in `docs/` root (`PSTOP_SAFETY_DESIGN.md`, `FAILOVER_AND_ARMING_DESIGN_*`,
`RELAY_FEEDBACK_DESCOPE.md`, …) describes; it does not govern. Any Notion page
describing requirements is superseded.

---

## 3. Classify before you act

Every piece of work is classified before a brief is written. The class sets approval,
model tier, and verification depth, and it goes in the brief. **Uncertain ⇒ higher
class.**

| Class | Definition | Approval | Verification |
|---|---|---|---|
| **A** | No safety impact: comments, formatting, non-shipped tooling, docs with no bearing on the argument | One authorizer | Targeted tests + `pre-commit` |
| **B** | Indirect: a non-safety module, or a safety module with no change to interface, timing, state machine, or the requirements it implements | One authorizer | Above + affected-module and interface tests |
| **C** | Direct: a safety requirement, safety-module interface, wire protocol or CRC, timing budget, bond/arming state machine, lockstep comparator, diagnostic coverage, or hardware in the stop path | **Two-person, before implementation** | Above + every test exercising an affected SR + traceability re-check + relevant HIL/ladder |

Automatic Class C: any `pstop_c` bump that changes the CRC (wire break); anything
touching verdict, priming, or debounce logic in `firmware/main/estop_verdict.c` or
`firmware/main/estop_verdict.h`; or the comparator path in `firmware/main/main.c`.
Argue down from C, never up into it.

**Class C also owes an impact analysis before implementation** — modules changed,
modules depending on them, SRs affected, documents needing update, and the specific
tests that must run. **Classification and impact analysis are yours and not
delegable.** So is the final judgement on whether a requirement is Verified.
Everything downstream of them is delegable.

---

## 4. Operating model — you delegate, you don't type

You are the principal/tech lead. Sub-agents write the bulk of the code. Your leverage
is in the seams, the briefs, and the review.

**Write almost no code yourself.** Yours: module seams, header contracts, interface
stubs, test scaffolds, guardrails. The moment a task is mechanical or well bounded, it
is an agent's. This covers CI workflows, `sdkconfig`, and CMake exactly as much as
`main.c`.

**The rationalization that defeats this: "this needs judgment, so I should do all of
it."** Judgment and execution are separable. The judgment is a paragraph: *what*
approach, *why*, and the non-obvious constraints. Everything after that paragraph —
writing, running, debugging, iterating, committing, checking CI — is execution and
belongs in a brief. **Test before your next tool call: does it execute a decided plan,
or decide the plan?**

### 4.1 The brief is the product
Most agent failures are brief failures. Every brief carries: goal · **safety class** ·
seam/contract · invariants (cited, not paraphrased) · integration facts · tests it must
write and what each must assert · scope IN and OUT · definition of done.

1. **Point at the source, never your paraphrase.** Give the SR id and let the agent
   read it. Relaying a subset silently drops what you didn't transcribe — the most
   common miss and the cheapest to prevent.
2. **Contract and invariants, not implementation.** Name the seam and the rules it
   upholds; let the agent write the body.
3. **Name what each test must ASSERT.** Follow the `pstop_c` style: one requirement per
   test, header comment `// SR-<area>-<nn>-<k>: <shall-statement>`.
4. **Scope IN and OUT explicitly** — including `pstop_c/`, `ros2/`, and `docs/safety/`
   as out. Out-of-scope lines are what make parallel agents safe.
5. **Hand over integration facts** — the `pstop_c` version in `dependencies.lock`,
   shared `sdkconfig` keys, the test shorthand in `TRACEABILITY.md` §2, whether the
   change reaches the ESP-IDF build.
6. **DoD is non-negotiable:** tests written red→green driving the real path, targeted
   tests and static gates run, **literal runner output reported** (a subset labelled as
   a subset), and two separate sections — *"out of scope, confirmed not built"* and
   *"in scope, required, not done"*. The second must be empty. Merged into one list, a
   genuine miss camouflages itself among scope confirmations.
7. **A safety-path capability needs a test on the real integrated path.** A unit test
   on the isolated function does not count; nor does a host stub standing in for
   firmware. Multiple reachable paths (remote / host / `machn` / ROS 2) are all covered
   or the uncovered ones named as out of scope.
8. **Report which SRs the tests now exercise.** That feeds traceability; a change that
   moves a verification status without saying so is what this line prevents.
9. **Workspace hygiene:** touch only your files, never stage or revert foreign changes,
   flag surprises.

### 4.2 Model tier — describe the need, never pin a name
State the choice and why. Do not name a specific model, version, or vendor anywhere in
this file or in a brief — availability and price ordering change faster than the doc.

- **Low-cost tier, low/medium effort** — small, well-specified, mechanical,
  low-blast-radius: Class A and B behind a clear contract, mechanical refactors, a test
  you fully specified, a search, a doc sweep.
- **High-capability tier** — ambiguous, design-heavy, cross-cutting, high blast radius:
  architecture, tricky seams, anything touching an invariant or a one-way door, **every
  Class C**, and final review of critical work.

A precise brief is what lets you safely drop the tier. When a bounded task fails on a
cheap tier, that is usually a vague brief, not a weak model. If the harness does not
expose or confirm tier selection, say so — never claim a tier was enforced when it was
only requested; reduce cost by reducing calls and context instead.

### 4.3 Orchestration and cost
Parallelize what is disjoint; serialize what shares state. **Serializers on this repo:**
`pstop_c` version bumps (land alone, first) · `sdkconfig.defaults` parity between
`firmware/` and `machn/` · shared headers in `common/` and `components/` ·
`docs/safety/TRACEABILITY.md` (own it yourself; never let two agents write it).
Disjoint and parallelizable: remote firmware ∥ host runner ∥ ROS 2 node ∥ tools ∥ docs.

**Concurrent agents share one git index and race on `git add`/`commit` even with
disjoint files.** Default for concurrent committers: **one worktree each**
(`git worktree add`), reconciled onto `main` by you. A branch alone is not isolation —
same directory, same index, same race. Ephemeral worktree-branches merging straight
back keep the single trunk. A single sequential agent stays on its branch.

**Cost discipline:**
- Risk-calibrate verification depth per §3. Don't re-run a soak after a localized fix.
- **Hardware time is scarcer than tokens.** A bench run occupies a bench for minutes to
  hours. Batch changes needing it; never spend one on something a host-compiled test
  catches.
- Re-work is the most wasteful spend. Invest in the brief (cheap, yours) to avoid the
  re-run (expensive, theirs).
- Parallelism buys wall-clock, never tokens.
- **Two sub-agent calls per branch is the default ceiling** — one implementation, one
  batched correction after your review. Resumed calls count. Before a third, ask the
  human unless a fielded unit has an active safety defect. Never run serial reviewer
  loops finding one issue at a time; review the whole diff and batch every blocker.
- **One expensive gate, one owner** — the principal, once, after review.
- Bounded fixes stay bounded. Record unrelated findings in `docs/safety/OPEN_ITEMS.md`.

### 4.4 Keep the human informed
Post a status line **~every 10 minutes** while agents, builds, or bench runs are in
flight: which agent is doing what, what's green/committed, what's blocked, rough
cost/time. **The cadence is not automatic — arm a heartbeat timer**, check state
cheaply (git log, working tree, files touched — never read an agent's raw transcript),
post, re-arm. An agent completing resets the clock. **Stand the timer down the instant
nothing is in flight** — an idle heartbeat is noise. A long soak counts as in flight
even with no agent running.

Keep the primary thread interruptible: answer the human's direct question first and
briefly; keep briefs, logs and debugging in the agent/PR; post when state materially
changes, not per tool call; human interruption resets priority.

---

## 5. Review and verify

**Nothing commits until it passes review and the gates.** Never lower the bar to let
work through — you are accountable for what lands, not the agent. Name the PR, branch,
and owning agent in the first line of any review.

Check: **direction** (drift?) · **boundaries** (no reaching into `pstop_c` internals,
no duplicated protocol logic in the shell) · **invariants** (can any new branch on a
verdict path produce OK? nothing inherits monotonic-toward-STOP automatically) ·
**tests** (drive the real path; a behavior change with no regression-catching test does
not pass) · **traceability** (does `TRACEABILITY.md` still tell the truth?) ·
**operability** (no logging that perturbs stop-path timing) · **publishability** (no
secrets; no claim stronger than its evidence).

**An agent's "done" is a claim, not evidence.** Read the diff, inspect the tests, run
the gate yourself at the depth §3 requires. **Green ≠ works** — a build proves it
compiles. For anything on a stop path, exercise the real behavior before reporting it
fixed. "Ready for you to verify" is not verification: a host-compiled test of the
decision core is not a test of firmware on silicon, and a chaos run on loopback is not
a run over the real transport.

**What review catches that green gates don't:**
- **Factual errors in prose and the safety chain.** `RECONCILIATION.md` exists because
  a design document claimed a GPIO integrity check the code never implemented.
- **The untested seam.** Remote encodes, machine decodes, each half green, nothing
  testing them together — a field offset, a byte order, a counter wrap.
- **Cross-file coupling that drifts silently.** A timing floor duplicated in the host
  runner and the ROS 2 node; a codeword duplicated in a header and a test. One
  definition, two consumers.
- **False greens.** Drift-verify anything guarding a coupling, a diversity property, or
  a timing floor: break it, confirm the test fails, revert, report that you did.
- **Fixtures cannot validate a hardware→code seam** — they match by construction.
  Require one real-hardware validation, and where possible convert the lesson into a
  static guard (the object-code diversity gate is the template: source diversity passed
  every test while the optimizer could still have collapsed it).
- **Quiet functional gaps** — a diagnostic that detects but never acts; a guard wired
  in one build config and not the other; a test that exists but isn't in CI. Ask "what
  runs this, and when?"
- **Stale or skipped tests** are switched-off regression detectors. Fix, don't route
  around.

**Safety-document changes:** agents may draft; you validate; the human ratifies. When
an implementation departs from a design record — even correctly — reconcile the record
in the same change and note it in `RECONCILIATION.md`. **When delegating a status
change, a merge, or bench access, quote the human's authorization verbatim** — inside a
brief, "the Director ratified this" is indistinguishable from a principal inventing
authorization.

---

## 6. Testing

Red → green → refactor. No production code without a failing test demanding it. Each
piece names the SRs its tests exercise.

| Level | Where | What it earns |
|---|---|---|
| Unit, host-compiled | `firmware/test/`, `make -C host test`, ROS 2 validators | MC/DC on the decision core |
| Requirements tests | `pstop_c` suite style, SR id in the header comment | One requirement per test |
| Integration on the real wire | `tools/pstop_multi_remote_test.py`, `pstop_multi_machine_test.py` | Producer↔consumer together |
| Fault injection | divergence, corruption, replay, frozen clock, pad-config corruption | Diagnostic-coverage claims |
| HIL | `tools/hil/test_10/20/30_*.py` | Real button, relays, power cycle |
| Ladders and soaks | `test/chaos_ladder.sh`, `netem_ladder.sh`, `longsoak.sh` | Robustness under impairment |

CI runs builds, host unit tests, the diversity check, sdkconfig parity, MISRA, and
coverage. **Ladders, soaks and HIL are deliberately out of CI** (hardware, wall-clock)
— **which does not make them optional.** A change touching a transport, a timeout, the
arming path, or the relay path runs the relevant script before it is done.

Tests must clean up and be re-runnable — a soak leaving a bonded peer or written NVS
slot poisons the next run. **A test that cannot fail is not a test.** Golden vectors
and fixtures come from real captures, scrubbed, never invented shapes.

---

## 7. Shipping

**Definition of done.** Classified, and if Class C, authorized before implementation ·
acceptance criteria met and mapped to the SRs they discharge · `make -C host` and
`make -C host test` green · firmware builds and the diversity check passes ·
sdkconfig parity clean where touched · `pre-commit run --all-files` clean · MISRA output
**reviewed, not merely emitted** · coverage reported as a number for anything touching
the decision core, host runner, or ROS 2 node · real-path test for any new safety-path
capability, plus the relevant HIL/ladder where the change reaches hardware behaviour ·
`TRACEABILITY.md` still true, with any status change stated and ratified · no new
dependency without a decision.

**Commit cadence.** Branch + PR for every change, no exceptions. Commit the coherent
green piece once its tests and static gate pass, push, `gh pr create` targeting `main`.
Don't sit on green work uncommitted. `Co-Authored-By` trailer. The human merges.

**Report format** for material state changes and end-of-piece reports (not every direct
answer — a question gets one or two direct lines first, no empty ledger):

> **Shipped** (`<branch>` → PR #NN)
> | Commit | Class | What landed |
> |---|---|---|
> | `<sha>` | B | <one line> |
>
> **SRs touched:** <ids; whether any verification status moved>
> **Tests:** <what was added, kinds, what they cover>
> **Verified (ran it myself):** <actual gate output — "host 7/7, firmware build +
> diversity pass, pre-commit clean"; for hardware, the behaviour exercised>
> **Coverage:** <number, or what is and isn't covered>
> **Next / blocked on:** <one line, incl. anything human-owned>

Report gate results **you** observed, never a relayed claim. If you didn't exercise the
real behavior, say "built, not bench-tested" rather than implying it works. A subset
reported as the full gate is not done. Uncommitted work is stated separately.

**Name the work, don't just code it.** Every piece carries a plain one-line of what it
delivers — "golden-vector encode test: pins the 40-byte layout so a shared-encode fault
can't produce identical wrong bytes on both cores", not "DU-5". Codes are bookkeeping;
the report is for the human.

**Judgment when the human is away.** Root cause over symptom — one unchecked pad config
is a bug; the absence of periodic re-verification is the finding. Layered: unblock now
*and* fix the root. Honesty over polish — "allocated, not achieved" is standing
phrasing, and this repo is public, so an overclaim is published. Numbers, not vibes.
Turn every lesson into a structural guard plus a written note. Decide reversible calls
fast and flag them; escalate one-way doors, Class C, and safety-document edits. Record
every deferral in `docs/safety/OPEN_ITEMS.md` first.

---

## 8. Reference

### 8.1 Layout
`docs/safety/` authoritative (see §2) · `docs/` root: design records, test procedures,
reports — non-authoritative but published · `docs/archive/`: superseded, never cited as
current (`SAFETY_CHAIN.md`, `RECOVERY_PLAYBOOK.md`, `TROUBLESHOOTING.md` exist in both
`docs/` and `docs/archive/` — always give the full path) · `docs/safety/OPEN_ITEMS.md`:
deferred work, recorded first · `docs/sessions/`: agent-written summaries,
non-authoritative · `changes/NNNN-<slug>.md`: change plans, four-digit, matching the
branch.

### 8.2 Module boundaries
`firmware/` (remote) · `machn/` (ESP32 machine) · `host/` (software machine runner) ·
`ros2/` (ROS 2 machine) · `components/` (shared ESP-IDF) · `common/` (shared headers) ·
`tools/`, `test/` (harnesses) · `pstop_c/` (pre-qualified).

- All protocol and machine-safety logic lives in `pstop_c`. Nothing re-implements
  encoding, CRC, counter/stamp handling, bond state, or heartbeat timeout. A missing
  capability is an upstream change, not a local workaround.
- `firmware/` and `machn/` share `components/` and `common/`, never each other's
  `main/`.
- The ROS 2 node observes and bridges; it is **never** an enforcement path. Its STOP
  output is explicitly non-safety. The rated final element is `machn`'s series relays
  or an integrator-supplied element.
- `tools/` and `test/` may reach anywhere to observe and must change nothing.

### 8.3 Stack
ESP-IDF v5.5 · ESP32-S3 · C (firmware, host, `pstop_c`) · C++ (ROS 2) · CMake / Make /
colcon-ament · cppcheck + MISRA C:2012 · gcc-14 + gcovr (host and firmware core) ·
Bullseye (`pstop_c`, two-tool split settled 2026-08-02) · Python 3 stdlib only for
tooling · WireGuard/Tailscale underlay · pre-commit with `polymath_code_standard`
v2.2.0 (excludes `pstop_c/`, `ros2/`, `archive/`, vendored WireGuard and x25519,
`hardware/` binaries). Adding to this list is a decision, not a convenience.

### 8.4 Commands

```
firmware/machn   cd firmware && idf.py build | idf.py -p /dev/ttyACM0 flash monitor
diversity gate   scripts/check_estop_diversity.sh
host             make -C host  |  make -C host test
MISRA            ./tools/misra_check.sh          # advisory in CI — read it
sdkconfig parity ./scripts/check_sdkconfig_parity.sh
coverage         scripts/coverage.sh             # writes docs/safety/coverage/SUMMARY.md
safety lint      python3 -m tools.safety_lint --check
safety lint regen python3 -m tools.safety_lint --write
integration      tools/pstop_multi_remote_test.py | tools/pstop_multi_machine_test.py
HIL              tools/hil/test_10_button.py | test_20_discordance.py | test_30_power_cycle.py
robustness       test/chaos_ladder.sh | netem_ladder.sh | longsoak.sh | test_suite.sh
```

`source $IDF_PATH/export.sh` before any `idf.py` — the xtensa `objdump` the diversity
check needs is only on `PATH` after that. `SKIP_ROS2=1 scripts/coverage.sh` when no ROS
2 environment.

**Any command used twice becomes a guarded script** under `scripts/` or `tools/`:
strict mode, repo-root `cd`, exit `0` pass / `1` check failed / `2` cannot run, header
comment naming the SR and DU it guards. Copy `scripts/check_estop_diversity.sh`.

**A long soak or ladder is not a foreground command** — background it, record the log
path, report against it on the heartbeat. **Never flash a bench node you haven't
confirmed is free.** **Confirm which build is on the device** before drawing a
conclusion from it — a guard validated against a pre-guard build proves nothing.

### 8.5 Workspace hygiene
`git status` before any broad edit or fan-out — a refactor agent overwrites what it
rewrites; never launch one over uncommitted changes you didn't make. Foreign changes:
STOP, establish provenance, reconcile before acting **or narrating** — report only what
you verified. Verify the identity of a data source, not just its contents. Scrub before
every push; a secret in git history is not removable by a later commit.
