<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Closing the all-path stall class (run-33 residual) — design

Status: REVIEWED (adversarial agent, APPROVE-WITH-CHANGES) — this revision
incorporates all 7 required changes. Scope: the single remaining disarm class after
PR #96 — a ~2-4 s machn-side stall that ages ALL remotes' heartbeats
simultaneously (run-33 +167m: LAN-only PS included, ages fresh at samples,
self-healed 11 s). Plus the queued low-risk items (operator directive).

## 1. Evidence and the prime suspect

At the event: all three remotes' rtt elevated 1.3-1.5 s and ages crossed
2.0 s between 2 s samples; machn's `wg_max_iter_ms` gauge did NOT register a
new worst — but its high-water mark already sat at 5.2 s, so any stall
under 5.2 s is invisible to it (gauge blind spot, see §3). The DERP task's
rx-poll gap DID set a new worst (4.5 s) in the same window. machn-local
gateway-ping loss (11-15%) accrues while external probes measure the LAN
fabric at 0% loss — the loss is machn-side egress, not the wire.

The one remaining UNBOUNDED crypto path on the heartbeat task (`ml_wg_mgr`
loop, 10 ms cadence):

- The WG-rx drains (loop-head unbounded `while(xQueueReceive)` plus FOUR
  other processing sites: the peer-add and disco interleaves via
  wg_mgr_drain_wg_rx, the post-disco tail drain, and the pre-registration
  drain) are "safety first" — but HANDSHAKE packets share that FIFO with
  the heartbeats, and an inbound INITIATION costs ~4 X25519 ≈ 45 ms
  MEASURED (11.3 ms/op, docs/CRYPTO_SPEEDUP_REPORT.md; a response ≈ 2 ≈
  23 ms). A wave of ~44-88 initiations stalls heartbeat processing 2-4 s.
  Every other burst source in this loop is already budgeted (disco opens,
  peer-adds with safety-decrypt interleave, CMM sends).
- THE COUPLING MECHANISM: each handshake is processed holding
  LOCK_TCPIP_CORE (CONFIG_LWIP_TCPIP_CORE_LOCKING). A wg_mgr crypto burst
  therefore also stalls the DERP task's lwIP sockets (the observed 4.5 s
  rx-poll gap) and machn-local ICMP egress (the observed gw-ping loss) —
  one lock explains all three observations. Task priorities exonerate CPU
  starvation (safety 8 > wg_mgr 7 > DERP 5).
- Secondary: `wireguardif_periodic()` can INITIATE rekeys inline for all
  peer slots in one call — unbounded, deferred evidence-first (§5).

Plausible trigger: SESSION-ESTABLISHMENT PHASE-LOCK — sessions
(re)established together (boot, or a prior stall) rekey phase-locked every
~120 s (REKEY_AFTER_TIME) indefinitely; re-ingest is verifiably hitless to
WG sessions and cannot phase-lock rekeys itself. Unproven — §3's
instrumentation decides it from the next event (a 120 s comb in the
existing wireguardif_periodic timing log would confirm).

## 2. Fix A — handshake budget on the WG-rx drain (core, ~25 LOC)

ONE budgeted drain helper replaces ALL FIVE wg_rx processing sites (a
single shared per-pass budget — a handshake deferred at one site must not
be processed unbudgeted at another):

- Drain is SNAPSHOT-BOUNDED: `uxQueueMessagesWaiting()` at entry caps the
  pops this call — a requeued handshake cannot be re-popped in the same
  pass (the naive while-loop would livelock spinning on its own requeue).
- DATA packets (heartbeats, replies, cookies): drain freely, unchanged.
- HANDSHAKE packets (len>=4 && data[0] 0x01/0x02 — verified identical
  framing on both producer paths): consume from
  `ML_WG_HANDSHAKES_PER_PASS` (default 3 ≈ ≤135 ms worst per pass at the
  measured 45 ms); excess handshakes REQUEUE to the tail with a per-packet
  attempt counter (new u8 in ml_rx_packet_t padding). On requeue FAILURE
  (queue refilled by a concurrent producer) or attempt cap
  (`ML_WG_HS_REQUEUE_MAX`, default 3): free(pkt.data) + count
  (`wg_hs_dropped`) — never leak, never spin. Deferrals count
  (`wg_hs_deferred`).
- Fail-safe direction: a deferred/dropped handshake can only delay a
  REKEY — initiators retransmit at 5 s with ~60 s of session margin
  (REJECT_AFTER_TIME 180 s vs rekey start 120 s); the active session keeps
  decrypting heartbeats. Worst case = the existing rekey-late nuisance
  stop, never a missed stop.

## 3. Fix B — stall-EVENT instrumentation (telemetry only, ~40 LOC)

High-water gauges can't attribute events (blind under their own mark).
Add a small event ring (8 entries, DERP-task/wg-task-owned, monitor-read):

- wg_mgr: when an iteration exceeds 1000 ms, record {boot_s, dur_ms,
  wg_pkts, handshakes, peer_adds, disco_opens, periodic_ms,
  disco_probe_ms} for that iteration (per-pass counters; the last two are
  already timed at their call sites).
- DERP ring for iterations > 1000 ms: {boot_s, dur_ms, home_cstate
  (DERP_CS_* — separates residual blocking-DNS from rx backlog),
  last_dns_ms, rx_gap_ms}.
- Entries published via a release-stored write index (the in-tree
  zero-copy ring pattern); readers may see a torn in-flight entry, never a
  torn published one.
- NEW edge-drop counters at BOTH wg_rx producers (net_io UDP + DERP
  route): today they drop silently on a full queue; post-Fix-A a
  storm bigger than the drain rate sheds heartbeats AT THE EDGE while
  every iteration stays short — without `wg_rx_enqueue_drops` the rings
  are blind to exactly the mode Fix A creates.
- Export all of it in /admin/api/monitor; the soak driver already
  snapshots the monitor at each disarm, so the next event self-attributes.

## 4. Queued low-risk items riding along

- **#42 partial (gauge ONLY):** export `fleet_region_missing` (bool: fleet
  peer configured with region 0). The reviewed-out fallback is DROPPED:
  `ML_DERP_REGION` is 9 while this fleet lives on region 2 — a compiled-
  default aux would be a permanently useless TLS conn, and injecting
  synthetic regions into `ml_wg_collect_safety_regions` could re-home the
  machine itself (it also feeds the home-fallback candidate list). Root
  cause of the learning gap stays open (#42) — likely server-side map
  content for machine-role nodes.
- **ops:** run-33's START line printed a hardcoded stale build label;
  interpolate EXPECTED_FW instead (driver-only).

## 5. Explicitly out of scope

- Timeout/margin changes (2.0 s stands; operator-only).
- pstop_c: untouched.
- wireguardif_periodic rekey-initiation bounding: DEFERRED unless the §3
  rings attribute an event to it (touching wireguard_lwip's periodic path
  has wider blast radius than the rx-drain budget; do it evidence-first).
- ENOTCONN kick (#32), region sweep (#30), host-test PR: unchanged queue.

## 6. Validation

Bench: build both roles; flash uniform; verify budget behavior (handshake
storm synthesized by rebonding all remotes at once = flash-boundary reboot
does exactly this) and rings populate on induced stalls. Then the operator
gate: 8 h zero-exclusion soak, normal protection (hourly reports, disarm
class diagnosis via the new rings, HIL gesture rearm, driver preflight).
Success: 0 drops, or any drop fully self-attributed by the rings.
