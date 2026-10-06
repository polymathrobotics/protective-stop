---
title: Multiple Remotes, Single Machine
---

# Multiple Remotes, Single Machine

Several remotes can bond to one machine.
This page describes how the machine behaves with two remotes, A and B, and how many remotes one machine can hold.
Both sections describe the pstop_c machine (`pstop_c/pstop/src/pstop/machine.c`) plus the shared role policy in `common/pstop_aux_channel.h`.
The software backend in `ros2/protective_stop_machine` and the ESP32 machn use both unchanged.
For one remote on several machines, see [Single remote, multiple machines](single_remote_multiple_machines.md).

## Model

- **Fail-safe OR.** The machine runs only while every bonded remote reports OK.
- **Roles.** Each remote announces `operator` or `stop_only` in every frame, and the machine follows the latest frame.
  A stop-only remote can stop the machine but never re-arm it.
  An unspecified role counts as stop-only.
- **Arming cycle.** Re-arming is a STOP followed by an OK from the same remote, at least `min_stop_ms` apart (default 500 ms).
- **Owner.** The operator whose STOP opens the cycle becomes the owner (`remote_stop_id`).
  Ownership stays with that remote after the machine runs again.
- **NEED_STOP.** While the machine is in `NEED_STOP`, every OK is answered with STOP.
  The only exit is a STOP from the owner, or from any operator when nobody owns the cycle.
- **Release.** Ownership is cleared when the machine stops through a heartbeat timeout, when the owner unbonds, when the last remote leaves, or when a remote unbonds while the machine is stopped.
  Ownership is also dropped when the owner announces stop-only.
- **Timeout.** A remote is dropped after `heartbeat_ms x max_missed` of silence (default 400 ms x 5 = 2 s).
- **Slots.** The software backend holds 4 remotes at once.

## Two-remote scenarios

A and B are both operators unless stated.
"Armed" means the machine is running.
"Gesture" means a STOP followed by an OK at least `min_stop_ms` later.

| # | Situation | Machine | To run again |
|---|-----------|---------|--------------|
| 1 | A bonds first, B bonds later | A's bond starts `NEED_STOP`; B joining changes nothing | Either operator does the gesture; the first STOP owns it |
| 2 | Armed, B bonds | Keeps running | Nothing |
| 3 | Armed by A, A stops | Stops, A's gesture re-opens the cycle | A does the gesture; B must be sending OK |
| 4 | Armed by A, B stops | Stops, `NEED_STOP` | A does the gesture; B must be sending OK; B cannot re-arm |
| 5 | A and B both stopped | Stops | Both release; then the owner's gesture |
| 6 | Armed by A, stop-only B stops | Stops, `NEED_STOP` | A does the gesture |
| 7 | Armed by A, A silent | Runs for up to 2 s, then stops and clears ownership | B (operator) does the gesture |
| 8 | Armed by A, A sends UNBOND | Stops and clears ownership | B (operator) does the gesture |
| 9 | Armed by A, B silent | Stops after the timeout and clears ownership | A or B does the gesture |
| 10 | Armed by A, B sends UNBOND | Keeps running | Nothing |
| 11 | Stopped, B sends UNBOND | Ownership cleared, `NEED_STOP` | Either operator does the gesture |
| 12 | A returns after a drop | A re-bonds as a new remote; B stays bonded | Either operator does the gesture if stopped |
| 13 | Both drop or unbond | Stops, `NEED_STOP` | First re-bond starts `NEED_STOP`; then the gesture |
| 14 | Armed by A, A demotes to stop-only | Keeps running; ownership released | B (operator) does the gesture after the next stop |

### 1. Initial bond

The first remote to bond puts the machine in `NEED_STOP` with no owner.
A second remote bonding later does not change the machine state.
Whichever operator sends STOP first becomes the owner and must send the OK.
The other remote only needs to be sending OK.

### 2. A new remote joins an armed machine

A bonding remote starts in `BONDED` and takes part in the OR after its first OK.
It does not interrupt a running machine.

### 3. The owner stops

A's STOP sets `STOP_RECEIVED` with A as owner.
A's OK after `min_stop_ms` re-arms the machine.
An OK sooner than `min_stop_ms` is answered with STOP, and the cycle stays open for a later OK.
While B is still holding its stop button, the machine stays stopped.

### 4. A non-owner stops

B's STOP sets `NEED_STOP` because B does not own the cycle.
In `NEED_STOP`, B's own STOP and OK cannot re-arm the machine, even though B is an operator.
Only A's STOP restores `STOP_RECEIVED`, so A must do the gesture.
If A is unavailable, the machine stays stopped until A's slot ends (scenarios 7 and 8).

### 5. Both remotes stopped

The machine answers STOP while either remote is in `STOPPED`.
The owner's OK completes the cycle only after the other remote has also sent OK.

### 6. Stop-only remote stops

A stop-only remote's STOP stops the machine.
It never opens a cycle, so the machine is left in `NEED_STOP`.
Re-arming needs an operator.
When nobody owns the cycle, the first operator's STOP takes ownership.

### 7. Owner goes silent

The machine keeps its state until the timeout.
At the timeout A's slot is marked unknown, the machine stops, and ownership is cleared.
B then does the gesture: STOP, wait `min_stop_ms`, OK.
B sending OK alone is answered with STOP, because the machine is in `NEED_STOP`.

### 8. Owner unbonds

A's slot is released, ownership is cleared, and the machine stops.
The next steps are the same as scenario 7, without the 2 s wait.

### 9. Non-owner goes silent

The timeout stops the machine and clears ownership.
An operator must do the gesture before it runs again.
This applies to a flaky link on any remote, so a two-remote setup needs the gesture each time a remote drops out.

### 10. Non-owner unbonds while armed

B's slot is released and the machine keeps running on A alone.
The stop condition in `handle_unbond_msg` only fires for the owner, for the last remote, or when the machine is already stopped.

### 11. Non-owner unbonds while stopped

The machine calls `machine_stop_robot()`, which clears ownership and sets `NEED_STOP`.
Any operator can then open the next cycle, which differs from scenario 4, where A keeps ownership.

### 12. A dropped remote returns

A is bonded again as a new remote and does not own anything.
It must send OK like any other remote.
If the machine is stopped, either operator does the gesture.
If B is still bonded, A's bond does not force `NEED_STOP`.

### 13. All remotes gone

The last remote leaving stops the machine.
The next bond forces `NEED_STOP` with no owner, as in scenario 1.

### 14. Live role change

An owner that announces stop-only releases ownership, and a half-open cycle is voided.
The machine keeps running.
That remote's next STOP stops the machine, and it cannot re-arm.
An operator must do the gesture.

### Practical consequences

- Re-arming after a non-owner stop needs the owner present.
  Plan for the owner's remote to be physically reachable.
- A heartbeat timeout of any remote needs a new gesture.
  This is intended, but it makes a flaky link on one remote visible on the whole machine.
- A second operator is a backup.
  It can arm only once the owner's slot has ended or the machine has no owner.
- Check `status_reason`, `remotes[].in_use`, and `remotes[].stop_only` in the machine snapshot to see who owns the cycle and which role each remote announced.

## Scale ceiling (machn)

**Status:** documented limit, 2026-08-12. Raising it is future work (see *Raising the ceiling*).
**One-line:** a single machine (machn) reliably holds **~3 bonded remotes** (comfortable) / **4 (marginal)**; **5+ collapses** — machn's own processing latency, not any link, grows with remote count until the 5 Hz safety heartbeat exceeds the 2.0 s timeout.

### Symptom
Adding remotes to one machine uniformly inflates the pstop heartbeat rtt of **every** bonded remote — including a same-subnet LAN-direct remote (PS) that is normally ~200 ms. Past the ceiling, rtt oscillates and machn disarms (fail-safe STOP) repeatedly, then persistently. Because the elevation is uniform across all remotes regardless of their path (LAN-direct, inter-subnet, hairpin, relay), the delay is **machn-side processing**, not per-link.

### Measured curve
Bench: machn `192.168.107.192`, all remotes DERP-region-locked to 9, **no region switching**, build `5ef3baa`. Bisect: add one remote, watch 7 min, record disarms + peak rtt. Edge confirmed quiet by the DUT-host session throughout (external flush ruled out).

| Remotes | Disarms / 7 min | Peak rtt | Verdict |
|--------:|:---------------:|:--------:|:--------|
| 2 | 0 | ~200 ms | stable, healthy margin |
| 3 | 0 | 668 ms | **stable — comfortable ceiling** |
| 4 | 0 | 1034 ms | stable but **marginal** (>½ the 2 s budget) |
| 5 | 15 (collapse after ~2 min, then persistent) | 1525 ms | **broken** |
| 6 | many, oscillating | 500–1300 ms | broken |

Per added remote, peak rtt rises ~300–400 ms (roughly linear-to-superlinear). At 5 remotes the peak crosses the 2.0 s heartbeat timeout and machn cannot hold armed.

### It is NOT the recent DERP fixes (build-independent)
Reverting machn to **`1874ba1`** — the commit *before* `f2f883e` (immediate-reconnect) and the defer-aux work, i.e. none of the recent DERP-resilience changes — **still broke at 6 remotes** with the identical signature. So the ceiling is a pre-existing property of machn's multi-remote handling, exposed by scaling up, not a regression from PR #94's changes.

### Mechanism (most likely)
`components/microlink/include/microlink_internal.h` (~L137) already documents the class:

> *"each DERP (re)connect is a full TLS handshake whose CPU burst starves lwIP/httpd (400–700 ms latency spikes; an httpd handler-budget exhaustion took down the admin API during the multi-remote validation). An undamped negotiator is a reconnect-storm generator by construction."*

Under N-remote load, machn's single DERP I/O task does more work (per-remote disco/CMM, home-conn keepalive servicing, occasional reconnects). When the home DERP conn's read starves, machn tears it down and reconnects; each reconnect is a blocking TLS handshake (~1 s CPU burst) that stalls the task that also drives pstop rx/tx → all remotes' rtt spikes → feedback loop. The blocking-TLS-in-the-safety-loop pattern is the same root the defer-aux work targets, here amplified by remote count rather than an edge flush.

### Recommendation (current)
- **Deploy ≤ 3 remotes per machine** for comfortable margin; **4 is a hard, marginal ceiling**; do not exceed 4.
- Multi-remote validation soaks should run at ≤ 4 remotes until the ceiling is raised.

### Raising the ceiling (future — tracked in the scale-ceiling task)
Candidate levers, in rough priority:
1. **Non-blocking / async DERP TLS handshake** — drive the handshake across I/O-loop iterations so a (re)connect never blocks the pstop rx/tx path. This is the single highest-value change (also fixes the defer-aux self-deadlock and the edge-flush home-rx stall in one architecture change).
2. **Reduce home-conn reconnect triggers under load** — understand why the home conn read-starves at scale (keepalive cadence, read-watchdog thresholds) and throttle/avoid spurious teardowns.
3. **Offload TLS or move aux/standby connects off the safety I/O task** (separate task / core).
4. **Per-remote disco/CMM cost reduction** at scale.

### Caveats
- Measured on the office bench with all remotes on region 9 (dfw, a far relay); a nearer region or true LAN-direct-only fleet may shift the numbers, but the machn-side scaling is the invariant.
- Separate, unrelated limits observed the same day: remote must pin its machine in the peer table at 128-node scale; the sustained-ENOTCONN kick doesn't always recover; `/api/pstop_peer?clear=1` is broken (use `ip=127.0.0.1&port=1` to unbond).
