# Tailscale link + chaos test results — 2026-07-20

Firmware under test: `edd6a00` (pstop_c `d9f4970`, trinity Tailscale
fixes, disco-key rescue, reply-drain fix). Machine:
`host/machine_app_runner` (same pstop_c SHA), `host/machine.toml`
defaults (heartbeat 1000 ms, max_missed 1, max_lost 10). Chip on USB-NCM
with Tailscale primary; direct WG path via the tether endpoint
throughout (verified per-minute). Bench IPs are environment-specific and
omitted here.

## Phase 1 — 40-min baseline soak (pstop 10 Hz over Tailscale)

| Metric | Result |
|---|---|
| ICMP over tunnel (2 Hz, 4792 pkts) | **0.00 % loss**, avg 21.9 ms, max 732 ms |
| pstop heartbeats | 23 911 sent, 23 907 replies = **99.983 %** |
| Rate | exactly 9.99 Hz |
| Rebonds / lockstep mismatches / reboots | **0 / 0 / 0** |
| Path checks (`tailscale ping`, 40×) | 40/40 pong, all direct, ~67 ms |
| send_fail | 21 (0.09 %, clustered at the endpoint transition) |

Finding fixed during this phase: a transient reply backlog (DERP window
right after a peer switch) stood forever because the comparator read one
reply per tick — constant false 602 ms RTT and eaten MSG_LOST margin.
Fixed in `edd6a00` (drain all queued replies, newest wins). Post-fix
steady-state pstop RTT over the tunnel: **5 ms**.

## Phase 2 — chaos ladder (impairment proxy in the pstop path)

`tools/pstop_chaos_proxy.py` interposed (chip → :8891 → machine :8890),
impairments per direction; machine ARMED (robot=OK) for all phases, so
any STOP = false trip. Run by `test/chaos_ladder.sh`.

| Phase | Injected (each direction) | Reply loss | Rebonds | Mismatch | Machine |
|---|---|---|---|---|---|
| baseline-proxy | none | 0.00 % | 0 | 0 | **OK** |
| loss-1…30 | 1→30 % drop | 2.2→50.5 % (≈ compound 2p) | 0 | 0 | **OK — no false STOP up to 30 %/dir (50 % round-trip)** |
| delay-50…500 | +50→500 ms fixed | ≤0.43 % | 0 | 0 | **OK** (RTT tracked 2× delay, up to ~1 s) |
| jitter-reorder | 50 ms + 0–150 ms jitter | 5.7 % | 0 | 0 | **OK** (heavy reordering absorbed) |
| dup-5 / dup-20 | 5 / 20 % duplication | (extra replies) | 0 | 0 | **OK** (dups logged, rejected as counter regress) |
| corrupt-2 | 2 % single-bit flips | 4.3 % | 0 | 0 | **OK** (CRC rejects, no reply sent) |
| combo-bad | 5 % loss + 100 ms + 80 ms jitter + 5 % dup + 1 % corrupt | 6.7 % | 0 | 0 | **OK** |

Wire totals injected: 3 788 drops, 1 221 dups, 134 corruptions over
~75 000 datagrams. Chip: **boot_count stayed 0** (no crash/reset),
0 send failures, 0 lockstep mismatches through the entire ladder.

## Phase 3 — outage / fail-safe timing

| Outage (100 % loss) | Machine behavior | Recovery |
|---|---|---|
| 0.5 s | **rode through** — no STOP | seamless, 0 rebonds |
| 1 s | STOP on heartbeat timeout (~1–2 s window as configured) | chip re-bonded automatically; machine latches NEED_STOP |
| 2 s / 5 s | STOP held | re-bond automatic; **arming requires an operator press→release of the E-stop switch** (no auto-resume) |

Upstream `d9f4970` semantics confirmed live: a re-BOND while the machine
holds the remote in OK answers STOP and latches NEED_STOP — one-way
loss cannot auto-resume the robot.

## Bugs found by this campaign (all fixed)

1. `edd6a00` — chip comparator reply backlog stands forever after a
   transient path change (false RTT, eaten MSG_LOST margin).
2. `b98ee85` — allowlisted peer with rotated disco key dropped
   pre-decrypt; node-key rescue now runs first, allowlist enforced
   post-identification (found live: laptop tx climbing, rx 0).
3. `274f671` — host runner's anomaly tracker poisoned by a corrupted
   (bad-CRC) packet: a bit-56 stamp flip logged false "stamp not
   monotonic" on every subsequent message. Library was never fooled;
   tracker now gated on valid checksum.

## Phase 4 — WG/Tailscale-layer chaos (tc netem on the encrypted UDP 51820 flows)

Run by `test/netem_ladder.sh` (self-installing scaffolding with
per-phase qdisc-counter proof — REQUIRED, the tether iface loses
all qdiscs on every chip reboot/USB re-enumeration; an earlier
unverified run silently measured a clean link). Firmware `4062d33`
(egress pinned to the WG netif + a test-only `pstop_sim` auto-arm pulse,
since REMOVED from production firmware — arming a production unit needs
a physical press).

| Phase | Injected (underlay, both dirs) | Reply loss | Machine | Path |
|---|---|---|---|---|
| baseline | none | 0.00 % | OK | direct |
| loss-10 | 10 % WG-packet drop | 19.0 % (≈ compound) | **OK — no false STOP** | direct |
| delay-150 | +150 ms | 0.23 % | **OK** | direct (brief relay flirt) |
| blackhole 5 min | 100 % direct-path drop | 0.73 % overall | STOP on timeout (correct), then heartbeats **resumed via DERP** | direct → relay "dfw" |
| restore | cleared | 0.08 % | OK (auto re-armed by the test pulse) | **direct again immediately** |

Blackhole timeline: replies dark ~10–66 s after the cut (machine safely
STOPped), then the chip re-bonded THROUGH DERP and ran the protective
link at 10 Hz / ~201 ms RTT for the rest of the outage. Direct path
re-formed within the first sample after the blackhole lifted. (The
10–66 s dark window was subsequently cut to ≤10 s — see
`FAILOVER_AND_ARMING_DESIGN_2026-07-21.md`.)

## Bug 4 (the critical one) — plaintext downgrade, found by this phase's prep

`92f418c` — with the pstop socket bound INADDR_ANY, lwIP routes per
packet by destination; whenever the WG session lapsed, the 100.64/10
peer fell through to the default route and the safety heartbeat rode
the USB tether IN PLAINTEXT (the bench laptop's weak-host model
accepted it, so everything "worked" and looked healthy). Fixed by
source-binding the socket to the VPN IP for Tailscale peers — egress
is pinned to the WG netif, and a downed tunnel now fails safe (sendto
error → machine STOP) instead of failing open. Verify on any future
transport claim: tcpdump the underlay and confirm no plaintext
application ports.

## Verdict

Loss to 30 %/direction, delay to 500 ms, heavy reordering, 20 %
duplication, 2 % corruption, and their combination produce **zero false
STOPs, zero rebonds, zero device crashes**; sub-second outages ride
through; ≥1 s outages fail safe and require deliberate re-arming.
