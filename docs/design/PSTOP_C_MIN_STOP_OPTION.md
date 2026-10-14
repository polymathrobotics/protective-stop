# Option record — moving the min-STOP-duration arming policy into pstop_c

Status: **NOT adopted.** The policy ships in the machine wrapper
(`host/machine_app_runner.c`, see `FAILOVER_AND_ARMING_DESIGN_2026-07-21.md`).
This document specifies the library-side alternative in enough detail to
decide on it during certification scoping, with design rationale and a
risk register. All code references are pstop_c @ `d9f4970`
(`components/pstop/upstream/pstop_c/`).

## 1. When this option becomes necessary

The wrapper-owned policy assumes the arming-duration requirement is a
*deployment* policy layered on top of the certified stop function. If the
certification scope instead classifies "arming requires a deliberate
press" as part of the **safety function itself** (e.g. it becomes a claim
in the safety case, or an assessor rules that safety behavior must not
depend on integrator wrapper code), the enforcement must move inside the
certified artifact. Enforcement location is a trust decision:

| | Wrapper (current) | Library (this option) |
|---|---|---|
| pstop_c modified | no | yes (~15 lines core logic) |
| Every integrator gets the policy | no — each machine wrapper must implement it | yes — one implementation, one test suite |
| Policy under certification evidence | no (documented, tested, but outside the artifact) | yes |
| Re-validation burden now | none | full (see §4) |
| Risk of divergent/incorrect integrator copies | real (each new machine host re-implements) | eliminated |

The wrapper approach is correct for one bench and one machine host. The
moment there are multiple machine integrations, the "every integrator
re-implements a safety-relevant veto" argument favors the library.

## 2. Proposed change (design)

### 2.1 Where arming happens today

The restart-state machine lives entirely in `machine.c`
(no other file reads or writes `robot_state`):

- `machine.h:38-40` — `restart_state`: `OK / NEED_STOP / STOP_RECEIVED`.
- `handle_stop_msg` (`machine.c:116-144`) — an accepted STOP from the
  cycle-owning remote sets `restart_state = STOP_RECEIVED`
  (`machine.c:126,136`).
- `handle_ok_msg` (`machine.c:85-112`) — with `restart_state ==
  STOP_RECEIVED` and no remote still reporting STOPPED, the arming branch
  at `machine.c:103-109` sets `robot_state = OK`. This is the **only**
  arming site in the library.
- Time is already abstracted: `env.get_time_cb`
  (installed by the host, monotonic ms) drives heartbeat checks.

Today any single STOP→OK message pair completes arming regardless of
duration — confirmed by the library's own test
(`machine_test.c:498-523`). A 100–200 ms EMC-induced loop blip on the
remote therefore performs the arming gesture (observed live, 2026-07-21).

### 2.2 The patch

1. **Config** — `pstop_application_config_t` (`pstop_application.h:77-91`)
   gains `uint64_t min_stop_ms;`. Default **0 = disabled**, which makes
   the patched library bit-for-bit behavior-compatible with d9f4970
   until a host opts in.
2. **State** — `robot_state_t` (`machine.h:42-60`) gains
   `uint64_t stop_episode_start;`. Written in `handle_stop_msg` at the
   two `restart_state = STOP_RECEIVED` assignments (`machine.c:126,136`),
   from `env.get_time_cb()` — only when entering the state (episode
   start, not refreshed per STOP message).
3. **Guard** — top of the arming branch in `handle_ok_msg` (immediately
   before `machine.c:103`):
   if `min_stop_ms != 0 && (now - stop_episode_start) < min_stop_ms`,
   do NOT arm: set `restart_state = NEED_STOP`, `remote_stop_id = 0`,
   keep `robot_state = STOPPED`, reply STOP. (Identical resulting state
   to the wrapper veto and to the library's own heartbeat-timeout reset,
   `machine_stop_robot`, `machine.c:334-341`.)
4. **Resets** — `stop_episode_start = 0` wherever the cycle resets
   today: `machine_stop_robot`, the UNBOND full-reset
   (`machine.c:154-158`), and init (`machine.c:314-316`). A zero
   episode-start with `min_stop_ms` enabled must veto (treat "no
   recorded episode" as too short — fail toward STOPPED).

Semantics identical to the shipped wrapper policy, including the reply:
the OK message that would have armed is answered with STOP.

### 2.3 Design rationale

- The guard sits at the single arming site rather than in message
  pre-processing, so the STOP path is provably untouched (a filter ahead
  of `handle_stop_msg` could delay stops; this cannot).
- Duration is measured on the machine's clock via the existing
  `env.get_time_cb` abstraction — no new time dependency, no reliance on
  remote-reported stamps (which a faulty remote could fabricate).
- `remote_stop_id = 0` on veto forces the next arming attempt to begin a
  fresh cycle — consecutive blips cannot concatenate into a valid press.
- Default-off config keeps every existing deployment and every existing
  test green without modification; the feature is opt-in per machine.

## 3. Behavior change to sign off (same as wrapper, restated)

With the policy active, a transient STOP shorter than `min_stop_ms`
while the robot is running stops the robot and does **not** auto-resume;
recovery requires a deliberate press-and-hold. Strictly more
conservative, but it converts brief nuisance stops from self-recovering
into operator-attended events. This tradeoff was accepted for the
wrapper policy on 2026-07-21 and carries over unchanged.

## 4. Risk register

| # | Risk | Severity | Mitigation |
|---|---|---|---|
| R1 | Touching the certified artifact invalidates existing verification: `machine_test.c` has 20+ restart-state assertions built on "any STOP→OK arms". | High (process) | Patch is default-off, so existing tests pass unmodified; ADD tests: blip vetoed pre-arm, blip while running, consecutive blips, blip-then-valid-press, timeout-mid-episode, `min_stop_ms=0` regression pair. Re-run the full suite; re-baseline coverage/requirement traceability (#40/#41 upstream tagged tests by requirement ID — the new behavior needs a requirement first). |
| R2 | New state variable desync: `stop_episode_start` stale across re-bonds or timeouts could validate an unrelated later release. | High (functional) | Reset at every cycle reset (§2.2 item 4); zero-value fails toward veto, never toward arm. This mirrors the wrapper's episode-reset rules, which were edge-case analyzed and bench-tested. |
| R3 | Clock misuse: a host installing a non-monotonic `get_time_cb` (wall clock) makes durations jump backward/forward. | Medium | Document the monotonic requirement on the config field (it already implicitly exists for heartbeats — a wall-clock host is already broken); negative/overflow deltas evaluate as "too short" with unsigned arithmetic → veto (fail-safe). |
| R4 | Divergence between library policy and any wrapper policy left enabled (double enforcement with different thresholds → confusing veto attribution). | Low | On adoption, the wrapper veto must be REMOVED (its status_cb latch may stay as belt-and-braces); precedence documented in the host README. |
| R5 | Wire/interop: remotes or other machines misinterpret the new STOP reply to an OK. | Low | No wire format change; replying STOP to an OK is an existing protocol situation (`machine.c:89-92` does it for NEED_STOP today). Remote-side behavior verified on this bench: the chip treats the reply type as display-only; resync uses counters/stamps. |
| R6 | Upstream drift: the patch must be maintained against pstop_c's evolution (e.g. the d9f4970 BOND-rule changes touched adjacent lines). | Medium (process) | This is the standing cost of the option — it is an upstream contribution candidate, not a local fork patch. Recommend proposing it upstream (polymathrobotics/protective-stop) so it ships with the certified library rather than as a divergence. |
| R7 | The veto path itself has a defect. | Low | The veto writes the exact state triple the library's tested timeout path writes; property to verify in review: no path through the guard can increase capability (arm) — every outcome is `STOPPED/NEED_STOP` or unchanged. |

## 5. Recommendation

Keep the wrapper policy as deployed. Raise this option at certification
scoping with one question: *is minimum-press-duration arming part of the
certified safety function?* If yes — do not maintain it as a local
patch; contribute it upstream to pstop_c (default-off config), with the
new requirement + tests from R1, and retire the wrapper veto per R4.
If no — this document records why the wrapper enforcement is sufficient
and what was considered.
