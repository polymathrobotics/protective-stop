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

> Scope caveat: "all five sites" covers the `wg_rx_queue` ingestion model
> only. With `CONFIG_ML_ZERO_COPY_WG=y` (default n, no in-tree config
> enables it), direct-UDP WG packets bypass the queue — `zc_pcb_recv_cb`
> calls `wireguardif_network_rx()` inline on tcpip_thread — so a handshake
> wave on that path runs unbudgeted under LOCK_TCPIP_CORE. Thread the same
> budget into `ml_zerocopy.c` before enabling zero-copy.

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

## 7. Phase 2 (2026-08-15): re-ingest quiescence + region stash + ring v2

Status: DRAFT for review. Evidence: run-34 rings (steady-state 1.0-2.0 s
stall clusters, every entry adds=4 with ~70-90% of the duration
unaccounted) and the post-run storm (2,998 ms cluster during flush
recovery, compounding losses). Root chain, source-verified: the 5-min
re-register re-ADDs every peer; existing-peer re-adds already skip the
WG reinstall (hitless re-ingest), but each one still calls
ml_peer_nvs_save() → dirties the peer cache → the 5 s-debounced
ml_peer_nvs_flush_if_due() commits to FLASH in-loop — and a flash write
suspends flash-resident execution on BOTH cores, stalling wg_mgr, the
DERP task and ICMP egress at once (the "unaccounted" time; also the
likely body of run-33's 2-4 s event and run-34's forensic 2.7 s gap).

### 7a. Skip-unchanged re-adds (fix 3 core, ~20 LOC)
In add_peer(), for an EXISTING peer, compare the material fields
(public_key match is given; disco_key, hostname, vpn_ip, and the update's
carried region/endpoints vs stored) and RETURN EARLY when nothing
changed: no memcpys, no maybe_rehome, no ml_peer_nvs_save (⇒ no dirt ⇒
no flash flush), no pacing slot consumed (adds_this_pass unchanged —
skipped re-adds are ~µs). Counter: peer_readds_skipped. Effect: a
5-min re-register of an unchanged map becomes telemetry-free and
flash-free; re-ingest clusters only occur when the map genuinely changed.

### 7b. Flush quiet-gate (fix 3 belt, ~6 LOC)
ml_peer_nvs_flush_if_due() additionally defers while
uxQueueMessagesWaiting(peer_update_queue) > 0 (an ingest burst is in
progress — the worst moment to suspend the cores). Flash writes then land
in genuinely quiet passes. The existing 5 s debounce stays.

### 7c. Learned-region stash (fix 4 / task #42, ~18 LOC)
Both existing region writes already preserve-on-zero; the wipe is
REMOVE→re-ADD: the fleet peer is pinned but NOT is_safety_peer, so the
hitless-reingest remove-veto does not protect it, and a fresh slot has
nothing to preserve. Minimal fix that changes no remove/veto semantics:
a 4-entry {pubkey, region} stash written when a PINNED peer with a
nonzero learned region is removed; add_peer consults it when the incoming
update carries region 0. Counter: region_stash_restores. (The
server-side question — why machn's map omits the fleet region — stays
open in #42.)

### 7d. Ring v2 + soak-driver self-audit (fix 5, ~15 LOC + ops)
- wg ring entries gain cmm_sends (site exists at the CMM pacing loop).
- wireguardif stamps worst_rx_gap_at_ms when the per-peer worst updates
  (one field + one line in wireguardif.c, accessor alongside the existing
  one) — makes cumulative-gauge events datable (run-34's 2,720 ms and the
  DUT's 19.6 s reconcile become decidable).
- ops driver: hourly line gains machn pstop-side rebond totals and
  rx_worst_gap(+at) so blind-window drops self-audit.

### 7e. Explicitly out of scope (unchanged)
Timeouts; pstop_c; wireguardif_periodic bounding (rings show ≤62 ms all
night — exonerated); moving flush to another task (flash suspension is
core-global — the fix is writing less and at quiet times, not elsewhere).
