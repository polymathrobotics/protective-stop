# Transport test campaign — USB-NCM / Ethernet (PoE) / WiFi — 2026-07-20/21

Protective-stop link validated end-to-end (chip remote ↔ machine_app_runner,
pstop_c `d9f4970`, 10 Hz heartbeats over Tailscale/WireGuard) across three
transports, each with: a 30–40 min soak, a protocol-level impairment ladder
(UDP chaos proxy: loss / delay / jitter-reorder / duplication / corruption /
outages), and a WG-underlay netem ladder (tc on the encrypted 51820 flows:
loss / delay / direct-path blackhole / restore). Machine armed for all
phases via press or (test-only, since reverted) pstop_sim pulse, so any
unscheduled STOP = false trip. Companion doc with USB-round details:
`CHAOS_RESULTS_2026-07-20.md`.

Final firmware under test: `9c18f6f` (production — pstop_sim reverted and
404-verified). Machine: `host/machine.toml` defaults (heartbeat 1000 ms,
max_missed 1, max_lost 10).

## Cross-transport summary

| Metric | USB-NCM | Ethernet (PoE) | WiFi (PS off) |
|---|---|---|---|
| Soak reply rate (30–40 min) | 99.983 % | **100.000 %** (17 852/17 852) | 98.09 % |
| Soak rebonds / false stops | 0 / 0 | 0 / 0 | ~1 per 10–15 min (burst > 1.1 s → fail-safe stop → self-heal) |
| ICMP over tunnel | 0 % loss, avg 22 ms | 0 % loss, avg 19 ms | avg 47 ms, max 1.7 s spikes |
| tailscale ping (disco) | 67 ms | 73 ms | 113 ms |
| False STOP under ≤30 %/dir loss, ≤500 ms delay, reorder, 20 % dup, 2 % corrupt | none | none | none beyond WiFi's own baseline |
| Direct-path blackhole → DERP failover | dark ~10–66 s, then 10 Hz @ ~201 ms | dark ~10–61 s | dark ~10–41 s |
| Restore → direct path | immediate | immediate | immediate |
| Device crashes/reboots (entire campaign) | 0 | 0 | 0 |

Protocol-ladder compound-loss math held on every transport (e.g. 30 %/dir
→ ~51 % reply loss) with **zero false stops and zero rebonds** in every
loss/delay/dup/corrupt phase; ≥1 s full outages fail safe (STOP within the
configured ~1–2 s), sub-second outages ride through.

## Bugs found and fixed by the campaign

1. `edd6a00` — comparator read one reply per tick; a transient burst left a
   permanent reply backlog (false 602 ms RTT, eaten MSG_LOST margin). Now
   drains the socket each tick, newest reply wins.
2. `b98ee85` — allowlisted peer with a rotated disco key was dropped before
   the node-key rescue could identify it (live symptom: laptop tx climbing,
   rx 0, no pongs ever). Rescue now runs pre-allowlist, filter enforced
   post-identification.
3. `274f671` — host runner's anomaly tracker recorded stamps before CRC
   validation; one corrupted packet poisoned it into permanent false
   "stamp not monotonic" spam. Tracker now gated on valid checksum.
4. `92f418c` — **CRITICAL: plaintext downgrade.** INADDR_ANY socket +
   per-destination lwIP routing meant a lapsed WG session dropped the
   100.64/10 peer onto the default route: the safety heartbeat rode the
   USB tether in cleartext (bench laptop's weak-host model made it look
   healthy). Egress is now source-bound to the VPN IP for Tailscale peers
   — tunnel down = sendto fails = machine STOP. Fail safe, never fail open.
5. `79f2587` — runtime WiFi enable path left modem power-save on
   (WIFI_PS_MIN_MODEM): radio deaf to replies between DTIM beacons →
   re-bond storm (3→67 rebonds in minutes). WIFI_PS_NONE now forced;
   post-fix WiFi is usable (storm → ~1 isolated burst event per 10–15 min).
6. Upstream pstop_c bump (`b499d74`): CRC16 poly change is a wire break —
   chip + machine must always be updated together (deployment rule).

## Open findings for follow-up (priority order)

1. **Auto-arm anomaly (SAFETY — closed 2026-07-21, see
   `FAILOVER_AND_ARMING_DESIGN_2026-07-21.md`).** In WiFi mode the machine
   re-armed (STOP_RECEIVED → OK) several times with no operator and no sim
   pulse. Evidence: machine3.log (36 STOP_RECEIVED events; re-arms
   mid-soak), chip `pstop_mismatch=1` (so the STOPs were BOTH-channel
   agreed verdicts — the lockstep cross-check only catches asymmetric
   faults). Leading hypotheses: (a) WiFi TX bursts (20 dBm) common-mode
   coupling into both protective-loop wires in the shared harness → brief
   agreed-STOP, recovery OK = arming gesture; (b) a restart-state path in
   upstream pstop_c's re-bond handling. Actions taken: loop-read release
   debounce (3 ticks) on the chip, and a MACHINE arming policy requiring
   a minimum STOP duration (500 ms) so single-tick blips can't arm.
2. **Machine `max_lost_messages` too tight for WiFi.** 10 (=1 s at 10 Hz)
   trips on ordinary WiFi bursts; align with the 2 s heartbeat-timeout
   policy (≈20) so the heartbeat timeout, not the counter gap, is the
   stop authority.
3. **DERP failover dark window was 10–60 s** (closed 2026-07-21: pong-recency
   watchdog on the priority peer, measured 9.3 s — see
   `FAILOVER_AND_ARMING_DESIGN_2026-07-21.md`).
4. **Tailscale subnet-route hijack (ops).** A tailnet subnet router
   advertising the robot's LAN steals operator-laptop traffic to the
   chip's LAN IP (policy rule beats the connected route). Workaround
   applied on the bench (`ip rule pref 5000`); document for the fleet.
5. **Don't OTA with a flapping interface enabled** — route churn kills
   the upload TCP session (two failed uploads until WiFi was disabled).
6. Cosmetic: `pstop_rtt_ms` quantizes to ~102 ms on tunneled paths (reply
   lands just past the 50 ms sync-recv window). Widen the window or
   timestamp on the drain path if sharper telemetry is wanted.

## Recommended next steps

1. Re-run the WiFi soak with the debounce + arming policy in place to
   prove zero unscheduled arms over a long window.
2. Transport policy for deployment: **Ethernet primary, WiFi fallback**
   (with the PS fix and retuned max_lost), USB-NCM for bench/service.
3. Consider adopting trinity `4c2f28f`'s remaining pieces after review if
   heap-constrained variants ship; port the restart-breadcrumb half of
   `78b93c8` for fleet diagnostics.
4. Wire-level transport audits (tcpdump the underlay for plaintext
   application ports) as a standing checklist item on any transport claim.
5. Long-duration soaks (24 h+) per transport before certification runs;
   the tooling (`test/chaos_ladder.sh`, `test/netem_ladder.sh`,
   `tools/pstop_chaos_proxy.py`) is committed and reusable.

## Bench state at campaign end

Chip on PoE Ethernet (bench LAN IP + Tailscale IP are
environment-specific), production firmware `9c18f6f` (pstop_sim REMOVED,
404-verified), bonded to the laptop machine at 10 Hz, machine disarmed
(NEED_STOP) awaiting a deliberate press. WiFi disabled (NVS), USB tether
idle. All tc/netem scaffolding removed; the `ip rule` subnet-route
workaround was still present on the bench laptop.
