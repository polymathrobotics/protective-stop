<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Stage 1 — Gapless forced region switch (pin → make-before-break)

Status: **IMPLEMENTED (36c768d + b4f7b5e) — pending bench validation (§8).**
Date: 2026-08-13. Parent plan: `NONBLOCKING_DERP_TLS_PLAN.md` (Stage 1 of 4).
Implementation deltas from this design (all safety-neutral): no httpd→wg_mgr
poke (the 3 s self-heal cadence IS the next tick — pin latency ≤3 s); the
no-live-home case fires nothing (the DERP task's own home-reconnect already
connects to the override); two extra executor fixes found during
implementation — the "already effective" check and the want-set exclusions
compared against `ml_effective_home_region()`, which under a pin equals the
override from the instant it is set, so both had to compare against the LIVE
home conn's region instead (§4.3 of commit b4f7b5e; also prevents a duplicate
aux to the region the home actively serves during the prove window).
Repro + evidence: run-26 + real-flush wire data (memory
`reference_pstop_switch_home_reconnect_dropgap`; peer artifacts
`flush-20260813T112019Z-*`).

Scope decisions (operator, 2026-08-13):
1. **Switch-path only.** The dead-home (RST/flush) reconnect is out of scope
   — different mechanism (nothing alive to keep serving), handled by the
   async engine (Stages 2–3).
2. **Command routing through wg_mgr** — the negotiator stays the single
   writer of the MBB command interface.
3. **Pin fallback = retry forever, never gap.** A pin toward an unprovable
   region keeps the old (working) home and retries with backoff; green is
   never risked to honor a pin faster.
4. **Delivery: new PR stacked on #94.** Gate: ~25-switch forced-switch soak,
   0 green drops, MBB-commit (no-teardown) logs confirmed.

---

## 1. Problem (one paragraph)

`POST /api/settings {"derp_region":N}` (machn, admin) applies a region pin by
firing `ML_EVT_DERP_RECONNECT` (`ml_config_httpd.c:649`). The DERP task
handler (`ml_derp.c:1272-1309`) **tears down the live home conn, then
blocking-reconnects** (DNS→TCP→TLS ~1-2 s on the DERP I/O task). During that
window the task cannot service rx, and the machn's inbound heartbeat
processing gaps; when the gap crosses the 2 s machine timeout the dual-core
verdict drops ROBOT_OK → nuisance fail-safe disarm (~1 in 5 switches on the
bench, self-recovers ~12-20 s, but latches disarmed pending a re-arm
gesture). Meanwhile the codebase already contains a **gapless** home switch —
the §7 MBB executor (`derp_mbb_tick`, `ml_derp.c:904`): open the target as an
aux, prove it (rx echo / PONG), then swap `derp_home_slot` **by index, no
teardown**; the old home drains as an aux via the want-set. Autoneg uses it;
the pin does not.

## 2. Design in one sentence

Make the pin a **first-class MBB source**: the httpd handler only records the
override; the wg_mgr negotiator (already the MBB single-writer) issues an MBB
command toward the override whenever the home conn is alive; the legacy
teardown path remains only for the no-live-home case (cold boot / already
dead — where there is no gap to protect).

## 3. Invariant change (the one semantic edit)

Today's executor invariant **I2** — "a lock landing mid-switch wins
instantly: ABORT" (`derp_mbb_tick`, `ml_derp.c:~928`) — was written for locks
preempting an *autoneg* switch. Stage-1 revises it:

> **I2':** an override aborts an in-flight MBB **only if it names a different
> region than the MBB target** (operator preemption). An MBB whose target
> *equals* the override is the override being applied — it proceeds.

Everything else in the executor (prove gates, generation preemption,
`SWITCH_ADVERT` commit, rollback) is reused verbatim.

## 4. Changes by file

### 4.1 `components/microlink/src/ml_config_httpd.c` (handler, ~646-665)

- **Remove** the direct `xEventGroupSetBits(..., ML_EVT_DERP_RECONNECT)`.
- Keep: `derp_region_override = N`, `config_save_settings()` (NVS), and the
  coord re-advertise (`microlink_request_announce`) — the re-advertise is
  independent of transport switching and must stay (PS-wedge fix).
- Add: `ml_wg_notify_region_pin(ml)` — a thin poke so the negotiator
  evaluates the pin on its next tick without waiting for a natural pass
  (see 4.2). Log line switches from "slot-0 reconnect" to "pin routed via
  negotiator (MBB when live)".

### 4.2 `components/microlink/src/ml_wg_mgr.c` (negotiator)

`neg_apply_target()` override branch (~1251-1263) currently:
`priority_peer_region = target; neg_cancel_mbb(); derp_home_region = target;
fire ML_EVT_DERP_RECONNECT`. Replace with:

```
if (ml->derp_region_override != 0) {
  ml->priority_peer_region = target;            /* advert: unchanged */
  if (home_conn_alive(ml)                        /* NEW: gapless path */
      && ml->derp_home_region != target) {
    if (ml->mbb_target_region != target) {
      s_neg_pending_source = MICROLINK_REGION_SRC_LOCK;
      ml->mbb_target_region = target;            /* same single-writer issue */
      ml->mbb_generation++;
      s_pin_mbb_requests++;
      log "PIN via MBB: region %u -> %u (gapless; old home drains)"
    }
    /* else: already in flight toward the pin — let it run */
  } else if (ml->derp_home_region != target) {
    /* no live home = nothing to protect: legacy immediate rehome */
    s_diag_rehome_applied++;
    ml->derp_home_region = target;
    fire ML_EVT_DERP_RECONNECT                    /* unchanged cold path */
  }
  return;
}
```

- `home_conn_alive(ml)`: `ml->derp[ml->derp_home_slot].connected` — read on
  the wg_mgr task; a torn read costs one conservative fallback to the legacy
  path (same as today's behavior), never a wrong commit.
- **Damping exemption:** the pin branch sits *above* the §8 damping guards
  (stability window, dwell, switches/hour circuit-breaker) exactly as the
  override branch does today — an operator pin is deliberate; it must not be
  suppressed. The MBB dedupe (`mbb_target_region == target → return`) is the
  only rate limit it needs.
- **Ban exemption:** `neg_region_banned()` is NOT consulted for the pin
  (matches today: the lock ignores autoneg's bad-region cooldowns).

**Outcome consumption (retry-forever):** where the negotiator consumes
`mbb_outcome` (ML_MBB_OUTCOME_ROLLED_BACK), add: if
`ml->derp_region_override == rolled_back_target`:
- do **not** ban the region (a pin is a command, not advice);
- schedule re-issue with backoff: 5 s → 15 s → 60 s, then every 60 s
  (`s_pin_retry_at_ms`, RAM-only), re-issuing the same
  `mbb_target_region`/generation bump when due;
- increment `s_pin_mbb_retries`; status stays `pin_pending` (see 4.4).
The old home keeps serving throughout — green never at risk. Pin lands on
the first successful prove once the region becomes reachable.

### 4.3 `components/microlink/src/ml_derp.c` (executor, minimal)

- **I2 → I2'** (~928): `if (override != 0 && override != s_mbb.target) abort`.
- No other executor change. The aux open reuses the want-set; during its
  blocking TLS handshake the home conn keeps being **pumped**
  (`derp_pump_home_rx`, commit 6832794) — Stage-1 composes with the pump:
  home stays connected *and* its rx is serviced during the new conn's
  handshake. The residual CPU burst is the same one switches #1-#4 survived
  in run-26 *without* a live home; with the home alive and pumped the
  timeout race is removed, not just narrowed.
- `ML_EVT_DERP_RECONNECT` handler: **unchanged** (still the correct path for
  RST/EOF dead-home recovery and the cold rehome).

### 4.4 Observability (small, same files)

- Counters (wg_mgr, exposed via `ml_wg_get_neg_diag` extension or a new
  4-word getter): `pin_mbb_requests`, `pin_mbb_commits` (from outcome
  consumption), `pin_mbb_retries`.
- `/admin/api/status`: `pin_pending` = (override != 0 &&
  effective_home != override) — lets the soak driver and the fleet console
  distinguish "pin applied" from "pin retrying".
- Existing `mbb_pending_region` / `mbb_state` already surfaced — reused for
  validation.

## 5. Ownership / concurrency review

Unchanged model. The negotiator remains the only writer of
`mbb_target_region`/`mbb_generation` (httpd never touches them — it writes
`derp_region_override` exactly as today, already a published cross-task
word). The executor remains the only writer of `derp_home_slot` and conn
state. `derp_home_region` gains no new writer: in the MBB path the executor
sets it at commit (existing); in the legacy path the negotiator sets it
(existing). The single new cross-task read (`derp[home_slot].connected` from
wg_mgr) is a volatile word read with a fail-conservative fallback. No locks
added; no pstop_c contact (`components/microlink` only — reliability, not
safety; every failure mode below fails toward "old home keeps serving" or
"legacy behavior", both green-safe or no-worse-than-today).

## 6. Edge cases

| # | Case | Behavior |
|---|------|----------|
| 1 | Pin while autoneg MBB in flight toward a **different** region | Negotiator issues pin target → generation bump preempts (executor restarts toward pin). I2' aborts nothing (override == new target). |
| 2 | Pin while autoneg MBB in flight toward the **same** region | Dedupe: command already in flight; executor proceeds; commit satisfies the pin. |
| 3 | Pin to the current home region | `derp_home_region == target` → no-op (advert refresh only). |
| 4 | Pin cleared (override → 0) mid-MBB | I2' no longer matches (override 0 ≠ abort condition — add explicit: target-source was LOCK and override cleared → negotiator `neg_cancel_mbb()` on its next pass; executor aborts on generation bump). Autoneg resumes ownership. |
| 5 | Pin changed N→M mid-MBB | Negotiator issues M, generation bump restarts executor toward M. Retry state resets. |
| 6 | Home conn dies mid-MBB (flush during switch) | Executor's existing stability gates fail → rollback; RST path reconnects home as today (out of scope); negotiator retries pin once home is back (4.2 fallback picks legacy if home still dead at retry). |
| 7 | Pinned region genuinely dead | Retry-forever with 60 s cap; `pin_pending` visible; old home serves indefinitely. Operator sees unfulfilled pin in status rather than a disarm. |
| 8 | Reboot with pin set (NVS) | Boot path connects directly to override region (existing `ml_effective_home_region` — no live home to protect; unchanged). |
| 9 | Remote peers' stale netmap | Unchanged: `microlink_request_announce` re-advertise still fires from httpd (4.1). MBB commit additionally sends NOTE_PREFERRED on both conns (existing). |

## 7. What Stage-1 does NOT fix (explicit)

- **Dead-home reconnect** (edge-flush RST, drops #2/#3 of run-26): the
  blocking reconnect with no live home remains; that is Stages 2–3 (async
  handshake state machine). Stage-1 removes the *forced-switch* trigger and
  proves the MBB-for-pin plumbing those stages build on.
- The reconnect-storm/RTT-inflation under sustained stress (scale ceiling):
  also Stages 2–3.

## 8. Validation plan (gate for the PR)

**Results so far (2026-08-13, machn OTA'd with bcc9d5d, verified behaviorally
via the new `pin_pending` monitor field):**
- **8.1 matrix: PASS.** Pins 9→2→10→12→9: 4/4 committed gaplessly in 12-15 s
  (pin tick ≤3 s, prove ~10 s, index swap), **`derp_reconnects` stayed 0 for
  the whole matrix** (legacy path was +1/switch), green held on every 3 s
  sample, 3/3 direct throughout.
- **8.2 negative: legacy semantic inherited, not testable on this bench.**
  Pinning a nonexistent region (999) COMMITS rather than retries: unknown
  regions fall back to the compiled default DERP host (`ml_derp.c` region
  lookup, documented pre-existing behavior identical in the legacy path), so
  the conn connects and proves against a real server. The retry-forever
  machinery therefore engages only for genuinely unreachable regions
  (server/network down) — exercised by code review + the rollback path;
  every failure mode degrades to "old home keeps serving".
- **8.3 preemption: PASS.** Pin →2 re-pinned →10 4 s later: executor
  restarted to 10 immediately (generation bump), exactly one commit, landed
  on 10, green held, reconnects 0.
- **8.4 run-27 gate soak: IN PROGRESS** (started 14:26 Z, 8 h, 19 min
  cadence, ~25 switches, cycle [9,2,10,12]).

1. **Bench live-switch matrix** (fast): pin 9→2→10→12→9 with all 3 remotes
   bonded; for each switch confirm in logs: `PIN via MBB` → `MBB AUX_OPENING`
   → `MBB COMMIT` (and **no** `DERP reconnect requested` line); green held;
   `pin_mbb_commits` increments; `derp_reconnects` does NOT increment.
2. **Negative prove test:** pin to an unreachable/bogus region id → verify
   `pin_pending`, retries at 5/15/60 s cadence, old home serving, green
   held; then pin back → immediate recovery.
3. **Preemption test:** start pin 9→2, immediately re-pin →10 (case 5);
   verify single commit to 10.
4. **25-switch soak** (run-27): run-26 driver re-used verbatim (20 min
   cadence, cycle [9,2,10,12], ~8 h). PASS = **0 green drops** across ≥25
   switches + every switch shows an MBB COMMIT (no teardown). Contrast
   metric vs run-26: drop rate 3/13 → 0/25; `derp_reconnects` stays ~flat
   (was +1/switch); RTT median should no longer step up per switch.
5. **Opportunistic real-flush overlay:** peer watchers stay armed; a flush
   during run-27 exercises the *unfixed* dead-home path — expected outcome
   documented (possible disarm, same as run-26 drops #2/#3), NOT a Stage-1
   failure. Distinguish via logs (`DERP reconnect requested` = flush path).

## 9. Deliverables

- One PR stacked on #94 (branch `derp/pin-mbb` off `derp/relay-recovery`):
  - commit 1: negotiator + httpd routing (+counters/status)
  - commit 2: executor I2' + LOCK-source abort handling
  - commit 3: docs (this file moves to Implemented status + soak results)
- run-27 soak report (25+ switches, table + verdict) before requesting
  review.

## 10. Effort / risk

Small diff (~150-200 LOC net, three files), no new tasks, no new locks, no
sdkconfig change, no pstop_c contact. Highest-risk element is the outcome/
retry bookkeeping in the negotiator (new small state machine: `s_pin_retry_at_ms`
+ backoff): mitigated by the negative prove test (8.2) and by the fact that
every retry failure mode degrades to "old home keeps serving" (green-safe).
