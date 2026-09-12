# Ethernet relay-path investigation, 2026-09-11

## Result

Events `308/366/469` support **relay return-path impairment**. Both the
machine host's DERP-9 connection and, during the second episode, the DUT's
home DERP-9 connection were affected. The exact reset generator and underlying
infrastructure mechanism remain unresolved. Event `474` shows intermittent
late replies and stale-stamp rejection; a later DERP EOF is not its established
cause. These events do not establish another site-WAN disruption pair.

All qualification resets remain. The active campaign is
`campaign-20260910T231555Z`, Ethernet run
`bf47386e2297422295a5392eaddb1834`, on firmware `v1.2-29-g901cafd` / ELF
`89a1018c1`. Collector SHA-256 remains
`b55094f1eaf00a0ebc8587f271cbbd0ee40502df991aa25370898c2efacfd7f4`.
Peer generation remains
`c51a8398c7f761436af2a397febc40a4df415abdb8ea06bea39931184ac8f074`.

## Accepted frames versus raw receipt

Machine-side packet/journal review supplied these references. Raw packets
received after protocol rejection do not restart the safety clock.

| Event | Last accepted frame, UTC | Observed ROS STOP, UTC | Accepted-to-STOP |
|---|---|---|---|
| 308 | counter 32870, 02:07:55.691 (rounded) | 02:07:57.360402 | approximately 1.669 s |
| 366 | counter 559, 02:09:55.492262 | 02:09:57.159304 | 1.667042 s |
| 474 | counter 393, 02:12:04.492379 | 02:12:06.158330 | 1.665951 s |

These satisfy the observed STOP limit of 2 seconds. The configured
silence/freshness budget stays **1600 ms**. Automatic raw-based latency fields
of 1180.1, 929.6 and 264.7 ms used later rejected packets and are unsuitable
for this comparison. Original observer records remain available.

## Reconciliation with independent peer critique

- The peer reported DERP-9 read resets at 02:07:56 and 02:09:55, followed by
  missing-peer routes during the second episode. Both machine nodes share
  that return connection, so simultaneous freezes of DUT slots 0 and 3 are
  expected from a shared relay-path failure.
- Longer DUT telemetry disproved the initial **machine-host-only** account:
  home region 9 is disconnected in every 5-second sample from
  02:10:01.399400 through 02:10:41.394758; auxiliary region 2 stays connected.
  Home reconnect completes by 02:10:46.373269, recording **43120 ms**.
  Selected replies remain at 374285, with raw age already 48440 ms at
  02:10:42, before recovering. The peer accepted this correction.
- `derp_reconnects` counts **completed** reconnections
  (`ml_derp.c:1185-1215`). A flat value at onset cannot exclude an ongoing
  reconnect. The receive-error cause and connected-state fields changed
  earlier. Missing-peer routes are consistent with the DUT's absence from
  its home relay; region-mesh repopulation is not independently proved.
- `469` reports the preceding storm's cumulative gap counter after re-arm;
  it is not independently established as another initiating outage.
- At `474`, replies and echoes progress intermittently during 02:12:01-06.
  Slot 0 recovers without rebonding; slot 3 rebonds after protocol expiry.
  The peer withdrew attribution to its later 02:12:36 EOF.
- Exact 120000 ms spacing between the first two freezes is not specific to
  WireGuard rekey. The DUT keypair-age gauge rolls over near 02:09:30 and
  02:11:30, before the freezes; keypair-expired remains zero. Rekey-at-freeze
  is weakened by these observations, not assumed impossible.
- Bench journal review found no corresponding networkd reload, public-address
  change or established Tailscale connection reset. One captured
  02:09:58.039246 RST answers a **new SYN** to `192.73.248.83:443`; its
  source process is unestablished. Claims of categorically no RST, or that
  reset-by-peer authenticates the reset generator, are unwarranted.

The adaptive scheduler correctly failed expired evidence even when early HTTP
reads succeeded: event 308's projected age was 1650.257 ms; event 474's was
1632.925 ms. Five early reads around 474 completed before their respective
prior expiries. There was no deadline extension.

## Why direct Ethernet has not returned

The direct retry loop is active. Between 00:14:01 and 02:14:26:

| DUT metric | Before | After |
|---|---:|---:|
| `direct_retry_rounds` | 2087 | 2409 |
| `cmm_rx_count` | 14269 | 16195 |
| `relay_disco_resets` | 495 | 573 |
| `relay_refetch_reqs` | 34 | 41 |
| `regains_safety` | 4 | 4 |

`direct_relay_bound` is 1 after fallback and the retry due time keeps being
re-armed. The refetch interval reaches its 1800-second cap. The target peer
`100.110.35.58` remains present, allowed, health-tracked and pinned in retained
peer diagnostics. Fleet connectivity is separately reported direct.

`ml_wg_mgr.c:4195-4298` performs candidate exchange and forced sweeps, with
timer resets and control-plane refetch escalation. Incrementing rounds means
the branch executes; it does not prove every probe was emitted or delivered.
The retained HTTP interface does not provide target-specific DISCO TX/RX or
the target's complete candidate list. `pp_*` is exclusively the configured
priority peer (`ml_wg_get_direct_diag`), not a substitute for all health-tracked
peers. Its empty endpoint fields cannot establish this target lacks candidates.
The monitor's `connection: wifi` string is hardcoded; `advert_lan_ip` and
`active_iface` identify this Ethernet run.

The peer's passive UDP capture reportedly sees no inbound traffic attributable
to PSTOP06 since 00:24:40, while another remote, `pstop-01d7f498`, forms a direct
path through the same site. This argues against a blanket site-wide UDP block;
it does **not** exclude per-flow/per-destination NAT or routing failure. Missing
traffic at the receiver cannot distinguish failure to emit from loss in transit.
Candidate ports also need DUT-specific attribution, not just a shared public IP.

The peer subsequently verified authenticated discovery attribution for PSTOP06
only at ports 1392 (04:04:15), 1476 (08:58:29) and 1735 (22:59:10) on
September 10. Of the currently probed ports 1098/1392/1416/1464/1783, only 1392
has that historical attribution; 1464 belonged to the other remote and 1783
was a bench-host STUN observation. The candidate list's origin is unverified.
The peer accepted retaining DUT emission, endpoint advertisement and per-flow
NAT/filtering as alternatives rather than assigning a discovery root cause.

PSTOP06 Ethernet (`10.74.30.17`) is a separate LAN host. The bench is not its
Ethernet NAT gateway, and its local conntrack cannot establish whether a gateway
mapping for `10.74.30.17:51820` exists. Required discriminating evidence is a
gateway-side mapping/packet view paired with the peer's authenticated discovery
identity and timestamps. This remains a network-owner evidence request.

One source-level recovery-policy gap is supported: `ml_derp.c:1334-1348`
selects fast periodic home retry using only `config.priority_peer_ip`, while
this application also tracks safety peers dynamically. After confirmed pause,
GET-only diagnostics at 02:27:39 found `priority_peer_ip: ""`; build sdkconfig
also sets `CONFIG_ML_PRIORITY_PEER_IP=""`. The machine peer remains health-tracked
and pinned. This supports a health-only safety peer missing the fast-retry gate,
which instead selects the 60-second periodic policy. Exact attempt timing still
does not establish how much of the 43-second reconnect this gate caused.
Changing the priority setting also affects other behavior; no such workaround
or firmware deployment was performed. A narrow health-aware retry predicate
needs independent review and deterministic testing before deployment.

Read-only hardware snapshot:
`/tmp/opencode/pstop-soak-20260910/paused-discovery-20260911T022740Z.json`.
It retains filtered settings, state, monitor and the two relevant peer entries;
firmware identity is unchanged, Ethernet is active, both slots report STOP,
and the machine peer remains relay-bound.

## Evidence and continuation

Root: `/tmp/opencode/pstop-soak-20260910/`. Detailed annotation:
`campaign-20260910T231555Z/ethernet/events-000308-000366-000469-000474-assessment.json`.
Follow each event's `evidence_batch`: 366 points to batch 365 and 469 to 433.

Bench snapshots `capture-archive/20260911T021217Z` and `20260911T021652Z`
have SHA-256 manifests. The latter preserves 37,695,811 bytes; packet timestamps
confirm 02:07:30-02:13 coverage in the uplink and Ethernet HTTP captures, with
no partial final records. This capture point does not observe all DUT-to-gateway
Ethernet unicast traffic.

Peer inner capture was reported as
`/home/iliabara/pstop_soak_8h/archive/20260910/soak_20260910T190556.pcap`.
After correcting an initial live-file claim, the peer supplied finalized
references following rotation: 928504 bytes, 8290 packets, actual coverage
02:05:56.184079-02:20:55.893239 UTC, SHA-256
`4625c220a0fcd4226b4866ce85eae8a238d16e4ac66db7d5169e7676f7c84c0f`.
Journal snapshot `archive/20260910/events_021745Z.jsonl` has 2571 lines and SHA-256
`d5dd93aa3dfff86a30def8b00c3fbf53a9b7b65cda6153f446ece922bf2b4c76`.
These peer-supplied hashes were not independently rehashed by the bench.
The peer added a separate header-only
DERP TCP recorder around 02:13 on `enx30d042ff393a`, snaplen 96, 15-minute rings,
filter `tcp port 443 and (net 192.73.240.0/20 or net 209.177.156.0/24)`.
It has no retrospective coverage for these failures and is outside the frozen
observer identity set.

The 02:18:10 RUNNING status (292.97 clean seconds, no passes) was superseded
by the campaign block below. Qualification is not currently running.

## Subsequent site-egress pair 8 and campaign block

Events `575/582` at 02:20:51.524 / 02:21:59.638 are corroborated by the bench:

- Established DERP-2 TCP connections reset at 02:20:50.157875 and
  02:21:58.190630, **68.032755 seconds** apart; control/logtail resets accompany
  both halves. New host STUN observations are `104.59.120.100:41641` at
  02:20:50.410407 and `192.184.222.186:1821` at 02:22:01.193530.
- Selected `send_fail` advances 55 -> 56 -> 66; aggregate ENOTCONN
  110 -> 112 -> 132; selected rebonds 22 -> 23. DUT completed home reconnects
  advance 23 -> 24 -> 25, taking 836 ms and 2827 ms respectively.
- The first half's approximately 1287 ms accepted-frame gap is below budget:
  **no machine STOP**. IDs 2571-2574 were unseen, not proven transmitted.
  The send-failure counter increment still resets qualification.
- In the second half, last accepted counter 2903 is at 02:21:58.124716;
  ROS STOP is at 02:21:59.750059, **1625.343 ms** later. First raw return is
  accepted BOND2 at 02:22:05.913111, a **7788.395 ms** receipt gap. BOND1 was
  not observed. `REMOTE_RESUMED.silent_ms=6187.6` starts at silence *detection*,
  not last receipt, and understates the full gap if misused.

This is the eighth supported shared site-egress pair across September 10-11;
it does not prove the gateway's exact failover, NAT or state-table mechanism.
The peer retracted claims that every half caused STOP: this half and the earlier
16:08 near-miss did not. Gateway-side logs/mapping evidence remain required.

Detailed annotation: `ethernet/events-000575-000582-assessment.json` in the
campaign. Snapshot `capture-archive/20260911T022659Z` preserves 39443044 bytes,
including both `uplink.pcap15` and `uplink.pcap16` across the second half's
actual ring boundary, plus Ethernet HTTP headers and a SHA-256 manifest.

At **02:22:07.976678** the supervisor entered **BLOCKED** because three re-arms
had occurred within 30 minutes. It terminated the collector, exited, and removed
its USB management-only guard in cleanup. No new gesture followed. After PAUSE
was requested, the bench verified MainPID 0, no service cgroup, Restart=no,
both old PIDs absent, and acquisition of the collector lock. At 02:27:16 it
explicitly acknowledged **PAUSED ownership with qualification still BLOCKED**;
`blocked-before-maintenance.json` preserves the original status. This was a
maintenance acknowledgement, not a campaign restart or acceptance decision.

The stopped polling-cadence watcher confers no pass. Ethernet's initial
30-minute review and actual first-refresh trace are retained; the current
campaign never reached USB qualification. Both four-hour windows remain unmet.

## Consolidated eight-pair arithmetic cross-check

The peer supplied a 16-half table with accepted-frame references and ten full
pcap hashes. Recomputing its timestamps confirms **11 ACTIVE-to-STOP
transitions**, **4 armed sub-budget halves without STOP**, and **1 half already
deactivated**. The observed STOP latencies range from **1623.446 to 1691.236 ms**,
within the 2-second limit. The four no-STOP halves are 2/2 (790.019 ms),
4/1 (1554.315 ms), 5/1 (944.922 ms) and 8/1 (1286.597 ms); half 1/2 happened
before re-arm. These are effects in this reviewed sample, not a qualification
pass or a universal safety claim.

Last-accepted-reference separations between the halves, in pair order, are
68.032202, 68.097156, 67.996379, 68.154031, 68.142863, 67.152618, 66.175425 and
68.139944 seconds. These reference separations are not independently measured
network-onset times; for example, pair 8's bench reset-log separation is
68.032755 seconds.

The initial hash discrepancy is resolved: the text API strips one trailing LF,
and the peer's advertised 7120 was a character count rather than byte size.
Restoring exactly one LF independently reproduces v1's **7176-byte** identity,
SHA-256 `03549fd71908db0157b71633486535628201079c7c91587bb7b8ba8aff433892`.
The 7175-byte API representation (`c0f384b3...`) is also retained.

The peer accepted the source-table corrections: event 575 belongs to site pair
8; underlay absence evidence starts at 00:24:40; excluded USB-stack faults are
distinct from included WAN effects during USB exposure; and pair 7's rejected
wire BOND counter 1 is distinct from the observer's bond-event ordinal 13.
Version **2.1** includes all these clarifications and unchanged row data. Its
8822-byte identity was independently reproduced after restoring the API's final
LF: SHA-256 `bbe03bf097046704b30d03d27b89d40ae9b5c71521b763133d7db345a66bad67`.
The peer updated its `_v2.md` path in place for v2.1; the intermediate
`fdaf4f8e...` v2 artifact was overwritten and is not preserved. The canonical
remote path is now `site_egress_pairs_machine_side_20260911_v2.1.md`, with
`_v2.md` retained as a byte-identical alias. The peer reports both posted
versions read-only and commits to new filenames for future revisions. Use the
full hash to identify the reviewed version; no reconstruction claim is made
for the overwritten intermediate artifact.

Both local copies are under `/tmp/opencode/pstop-soak-20260910/`:
`site_egress_pairs_machine_side_20260911.md` (v1) and
`site_egress_pairs_machine_side_20260911_v2.md` (v2.1). Each has an adjacent
`.review.json` with recomputed arithmetic and peer-reported pcap hashes;
`site-egress-table-imports.json` records the API and reconstructed hashes.
This verifies the table identities and arithmetic, not a new independent
decoding of the underlying pcaps. Qualification remains PAUSED/BLOCKED.
