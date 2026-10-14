# Multi-remote scale ceiling (machn)

**Status:** documented limit, 2026-08-12. Raising it is future work (see *Raising the ceiling*).
**One-line:** a single machine (machn) reliably holds **~3 bonded remotes** (comfortable) / **4 (marginal)**; **5+ collapses** — machn's own processing latency, not any link, grows with remote count until the 5 Hz safety heartbeat exceeds the 2.0 s timeout.

## Symptom
Adding remotes to one machine uniformly inflates the pstop heartbeat rtt of **every** bonded remote — including a same-subnet LAN-direct remote (PS) that is normally ~200 ms. Past the ceiling, rtt oscillates and machn disarms (fail-safe STOP) repeatedly, then persistently. Because the elevation is uniform across all remotes regardless of their path (LAN-direct, inter-subnet, hairpin, relay), the delay is **machn-side processing**, not per-link.

## Measured curve
Bench: machn `192.168.107.192`, all remotes DERP-region-locked to 9, **no region switching**, build `5ef3baa`. Bisect: add one remote, watch 7 min, record disarms + peak rtt. Edge confirmed quiet by the DUT-host session throughout (external flush ruled out).

| Remotes | Disarms / 7 min | Peak rtt | Verdict |
|--------:|:---------------:|:--------:|:--------|
| 2 | 0 | ~200 ms | stable, healthy margin |
| 3 | 0 | 668 ms | **stable — comfortable ceiling** |
| 4 | 0 | 1034 ms | stable but **marginal** (>½ the 2 s budget) |
| 5 | 15 (collapse after ~2 min, then persistent) | 1525 ms | **broken** |
| 6 | many, oscillating | 500–1300 ms | broken |

Per added remote, peak rtt rises ~300–400 ms (roughly linear-to-superlinear). At 5 remotes the peak crosses the 2.0 s heartbeat timeout and machn cannot hold armed.

## It is NOT the recent DERP fixes (build-independent)
Reverting machn to **`1874ba1`** — the commit *before* `f2f883e` (immediate-reconnect) and the defer-aux work, i.e. none of the recent DERP-resilience changes — **still broke at 6 remotes** with the identical signature. So the ceiling is a pre-existing property of machn's multi-remote handling, exposed by scaling up, not a regression from PR #94's changes.

## Mechanism (most likely)
`components/microlink/include/microlink_internal.h` (~L137) already documents the class:

> *"each DERP (re)connect is a full TLS handshake whose CPU burst starves lwIP/httpd (400–700 ms latency spikes; an httpd handler-budget exhaustion took down the admin API during the multi-remote validation). An undamped negotiator is a reconnect-storm generator by construction."*

Under N-remote load, machn's single DERP I/O task does more work (per-remote disco/CMM, home-conn keepalive servicing, occasional reconnects). When the home DERP conn's read starves, machn tears it down and reconnects; each reconnect is a blocking TLS handshake (~1 s CPU burst) that stalls the task that also drives pstop rx/tx → all remotes' rtt spikes → feedback loop. The blocking-TLS-in-the-safety-loop pattern is the same root the defer-aux work targets, here amplified by remote count rather than an edge flush.

## Recommendation (current)
- **Deploy ≤ 3 remotes per machine** for comfortable margin; **4 is a hard, marginal ceiling**; do not exceed 4.
- Multi-remote validation soaks should run at ≤ 4 remotes until the ceiling is raised.

## Raising the ceiling (future — tracked in the scale-ceiling task)
Candidate levers, in rough priority:
1. **Non-blocking / async DERP TLS handshake** — drive the handshake across I/O-loop iterations so a (re)connect never blocks the pstop rx/tx path. This is the single highest-value change (also fixes the defer-aux self-deadlock and the edge-flush home-rx stall in one architecture change).
2. **Reduce home-conn reconnect triggers under load** — understand why the home conn read-starves at scale (keepalive cadence, read-watchdog thresholds) and throttle/avoid spurious teardowns.
3. **Offload TLS or move aux/standby connects off the safety I/O task** (separate task / core).
4. **Per-remote disco/CMM cost reduction** at scale.

## Caveats
- Measured on the office bench with all remotes on region 9 (dfw, a far relay); a nearer region or true LAN-direct-only fleet may shift the numbers, but the machn-side scaling is the invariant.
- Separate, unrelated limits observed the same day: remote must pin its machine in the peer table at 128-node scale; the sustained-ENOTCONN kick doesn't always recover; `/api/pstop_peer?clear=1` is broken (use `ip=127.0.0.1&port=1` to unbond).
