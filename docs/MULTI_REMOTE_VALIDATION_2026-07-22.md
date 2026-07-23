# Many-Remotes-to-One-Machine Validation (2026-07-22)

Validates the pstop_c machine logic when **multiple remotes** heartbeat one
machine, across the corner cases a fleet actually hits. Two-part (hybrid):
exhaustive software remotes for coverage, plus the two real chips end-to-end
over the real transport.

## The logic under test (pstop_c, unmodified)

`machine.c` implements a **fail-safe OR**: the robot runs (OK) only while
*every* bonded remote is OK **and** no arming cycle is pending. Any of the
following drives STOP:

- a remote sends STOP,
- a remote goes silent past `heartbeat_ms x (max_missed_heartbeats+1)`,
- a remote unbonds (disconnect),
- a remote sends a malformed / bad-CRC / invalid-type message (rejected; its
  heartbeat is not fed, so it times out to STOP),
- the machine reaches capacity and rejects a new remote.

**Arming ownership**: the first non-`stop_only` remote to STOP owns the
STOP->OK re-arm cycle (`remote_stop_id`). A *different* remote can STOP the
robot but cannot itself re-arm — the owner must, or the owner must
unbond/time-out first (which releases ownership). Fail-safe: it is only ever
*harder* to arm. `stop_only` operators may STOP but never arm.

## Safety invariants asserted (all held)

| # | Invariant |
|---|-----------|
| I1 | any bonded remote sending STOP -> every remote sees STOP |
| I2 | a remote silent > timeout -> robot STOP |
| I3 | robot never OK without a deliberate held STOP->OK cycle |
| I4 | a STOP shorter than `min_stop_ms` (500 ms) cannot arm |
| I5 | a freshly (re)bonded remote forces re-arm (NEED_STOP) |
| I6 | the `(max_remotes+1)`th remote is rejected; armed peers undisturbed |
| I7 | unbonding the arming owner drops the robot to STOP/NEED_STOP |

## Part 1 — software-remote corner-case matrix  (32/32 PASS)

`tools/pstop_multi_remote_test.py` — spins up `host/machine_app_runner`
(linking the unmodified library) and drives it with N scripted remotes that
speak the real 40-byte wire protocol (CRC16 0x8D95), each with its own
device_id + counter handshake. Groups:

- **A** single-remote arming policy (arm / veto / re-arm)
- **B** two-remote arming + any-STOP OR + ownership
- **C** heartbeat loss (network drop / power-off) + re-arm on reconnect
- **D** unbond / disconnect (owner leaves, ownership release, last-remote)
- **E** capacity overflow (`max_remotes=3`), allowlist deny, `stop_only`
- **F** malformed: UNKNOWN, sustained bad-CRC, out-of-range type
- **G** network chaos via `tools/pstop_chaos_proxy.py` (loss/delay/jitter/dup):
  arm through proxy, heavy loss -> STOP, recovery -> re-arm, churn never
  spuriously arms.

Run: `python3 tools/pstop_multi_remote_test.py` (exit 0 = all invariants held).

## Part 2 — real two-chip end-to-end

Both flashed chips against one `machine_app_runner` on the laptop
(`machine.toml`: max_remotes=3, max_missed=1, min_stop_ms=500, allow_unlisted):

- `pstop-01d778b4` (0x01D778B4) — new chip, USB-NCM tether (10.42.0.1:8890)
- `pstop-01d7791c` (0x01D7791C) — old chip, Tailscale (100.110.35.58:8890)

Observed on the machine:

```
P1 near-simultaneous OK  -> ANOMALY: arming VETOED from 0x01D778B4 (100ms < 500ms)   [I4]
P2 NEW held STOP 1200ms -> OK -> ARMED by 0x01D778B4; Robot Status = OK
P3 OLD STOP while armed  -> Robot Status = STOP                                       [I1]
P6 both released to STOP -> disarmed; both chips pstop_last_msg=1 (STOP)              fail-safe
```

Confirms bond, arm, and the any-STOP OR work end-to-end with real hardware over
both transports.

## Test-only software E-stop stub (added, used, REMOVED)

The remotes have no physical E-stop switch wired, so they always read STOP and
cannot exercise the OK/arming path. A **gated** software override was added
ONLY for this test:

- `CONFIG_DCS_ESTOP_SW_STUB` (Kconfig, **default n**), forcing a build `#warning`.
- `POST /api/estop_stub?state=ok|stop|clear` overriding `compute_verdict()`
  (fail-safe default = passthrough to the real hardware read).
- Every piece tagged `TODO(SAFETY): REMOVE before certification`.

**Safety compliance**: a protective-stop verdict must derive ONLY from the
dual-channel hardware loop. The stub deliberately bypasses that cross-check, so
it was **fully reverted after testing** — the code is gone from the tree
(`git` clean), both chips were restored to clean `b19a754`, and the
`/api/estop_stub` endpoint now 404s on both. It was never committed or shipped.

To reproduce the stub for future bench work, re-add the gated option (never
enable it in a production/certified build) — see git history of this doc's
session, or the pattern above.

## Findings

No unsafe behavior. All STOP/timeout/reject paths are fail-safe. Observations
(all safe-by-design):

- Arming ownership sticks to the first arming remote until it unbonds or times
  out; a second remote can STOP but not re-arm. Intended, fail-safe.
- UNKNOWN and out-of-range message types are rejected (not fed to the heartbeat)
  -> STOP arrives via timeout, not instantly. Same effect as silence.
- httpd user-handler budget (`dcs_support.c max_user_uri_handlers=16`) was at
  its limit; adding one more handler (the stub) silently failed to register
  until bumped. Worth watching if more `/api` routes are ever added in prod.
