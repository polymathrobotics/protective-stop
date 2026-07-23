# Peer capacity scaling — 16 → 128 peers (2026-07-21)

Raises the Tailscale peer capacity to 128 on the ESP32-S3 (8 MB PSRAM)
**without** degrading the stability or latency of the one safety-critical
peer (`priority_peer_ip`, the 10 Hz protective-stop link). All changes are
in `components/microlink` (+ nested `wireguard_lwip`) and `firmware/`;
`pstop_c` is untouched.

## Principle: memory scales, CPU bursts don't

A full code trace found that **memory is not the limiter** — at 128 peers
the WG device struct (~114 KB) and peer table (~34 KB) are PSRAM-resident,
and per-packet peer lookups are µs-scale linear scans dwarfed by the
~30 ms NaCl box-open that follows. Every historical wedge in this firmware
was **CPU load starving the single wg_mgr loop** that also drains the
pstop heartbeat queue. So the load paths were armored first; the capacity
constants were raised last.

The NaCl layer recomputes the X25519 DH on **every** box open/seal (no
shared-key cache), so each inbound disco packet from an unknown/rotated
key costs ~30 ms — and the 2026-07-20 rotation-rescue deliberately admits
unknown keys to that cost. Unbounded, a 100-peer tailnet's magicsock
probing (or a malicious flood) is a multi-second stall. The fix is a
budget, not a revert.

## Load-path armor (landed before the capacity raise)

All budgets exempt the priority peer; all live in `ml_wg_mgr.c` /
`microlink_internal.h` / `ml_peer_nvs.c`.

| Change | Constant | Effect |
|---|---|---|
| WG-rx queue drained **first** in the loop + depth 8→32 | `ML_WG_RX_QUEUE_DEPTH` | pstop heartbeats no longer wait behind (or overflow from) a disco box-open burst — the single highest-value latency fix |
| Inbound disco box-open budget per loop pass | `ML_DISCO_OPENS_PER_PASS=4` | bounds the unknown-key admission; priority peer's disco key exempt (cleartext key at header offset 6, one memcmp, no crypto) |
| Peer-add pacing | `ML_PEER_ADDS_PER_PASS=4` | a 100-peer MapResponse used to drain in one pass (~10 s of X25519 stall); now ~4 adds/10 ms, loop returns for heartbeats |
| NVS peer-cache write debounce (5 s) | `ml_peer_nvs_flush_if_due` | first sight of 100 peers rewrote the growing blob once **per peer** (~465 KB flash into a 24 KB partition, cache-stalling both cores); now the PSRAM working copy is authoritative and flash catches up ≤1×/5 s (+ flush on deinit) |
| CallMeMaybe broadcast pacing | `ML_CMM_SENDS_PER_PASS=2` | post-STUN CMM to all sessionless peers was an all-at-once seal burst + inbound ping storm; now resumable 2/pass |
| Heartbeat seal cap + per-peer phase jitter | `ML_HB_SEALS_PER_TICK=8` | a shared trust-renewal moment (AP blip) aligned every peer's next heartbeat into one tick; jitter de-aligns the fleet, cap bounds the tick |
| Sessionless handshake-retransmit cap | (active-clear) | the lease/demotion path re-armed `peer->active`, and wireguardif then retried an initiation (~90 ms) every 5 s **forever** for peers that never answer; non-priority peers with no session get `active` cleared (inbound initiations still establish sessions) |
| Dedup ring 16→64, pending-probe table 32→64 | `WG_DEDUP_RING_SIZE`, `MAX_PENDING_PROBES` | dual-path dedup and probe tracking sized for 100+ peers |

## Capacity raise

`CONFIG_ML_MAX_PEERS` 16→128 (Kconfig range 1→128, sdkconfig +
sdkconfig.defaults); `WIREGUARD_MAX_PEERS` 16→128; the WG device
`heap_caps_calloc(MALLOC_CAP_SPIRAM)` explicit (was policy-dependent
`mem_calloc`), with an internal-RAM fallback; `cfg.max_peers=128` in
dcs_support.c; settings-blob validation and web-UI cap 64→128. Memory
delta ≈ **+130 KB PSRAM, ~0 internal RAM** (+1 KB BSS for the probe
table). No wire-protocol change; the settings-blob `max_peers` is uint8
with append-only versioning, so no NVS-format break.

## Deliberately NOT changed

- **NVS peer *cache* stays 64** (`CONFIG_ML_NVS_MAX_PEERS`): 128 entries
  (11.8 KB blob) can't safely coexist with a populated allowlist in the
  24 KB NVS partition. Growing the partition moves `ota_0` → full reflash
  - NVS wipe + re-provisioning — a flash-layout breaking change deferred
  to a hardware/major revision. The cache is only a boot accelerator;
  the control plane repopulates the rest.
- **WG sessions stay lazy** (peers added passive, sessions form on peer
  initiative) — no separate slot allocator needed; idle PSRAM slots cost
  only the 400 ms periodic's `valid` check.
- **Allowlist** Kconfig is already 512; keeping it *populated* in
  deployment remains the primary outbound-load bound.

## Validation (bench, 2026-07-21)

Real 100-peer tailnets aren't available on the bench, so
`tools/disco_storm.py` injects well-formed **unknown-key** DISCO pings at
the chip's UDP 51820 — driving the exact worst-case box-open path a large
tailnet produces. Concurrent 10 Hz pstop link measured throughout (new
USB chip, no loops wired so verdict is STOP — the test is heartbeat
continuity under crypto load, not arming).

| Storm rate | pstop sent/replies | send_fail | rebonds | RTT | boot_count | machine heartbeat timeout |
|---|---|---|---|---|---|---|
| baseline | lockstep | 0 | 0 | 5 ms | 0 | none |
| 40 pps | 1973/1973 | 0 | 0 | 5 ms | 0 | none |
| 600 pps (≈1.5× budget capacity) | 3336/3336 | 0 | 0 | 5 ms | 0 | none |

At 600 pps — above the ~400 opens/s budget — excess disco crypto is shed
while the heartbeat stays fully isolated: RTT unchanged, no dropped ticks,
no rebonds, wg_mgr stack healthy. **The 128-peer build boots clean
(`boot_count=0`), confirming the explicit PSRAM placement holds the
internal heap** — the v15.6 "64 peers exhausts internal RAM" warning does
not apply under the current SPIRAM-malloc policy + explicit placement.

### Real-tailnet validation (2026-07-21, Polymath tailnet, 343 nodes)

Run on the live Polymath tailnet — no synthetic control plane needed.
The bench chip registered (`vpn_ip 100.117.20.69`, `state CONNECTED`) and
pulled the **full 128 peers** (capped from 343 real nodes: macbooks,
robots, peplinks, desktops — all organically DISCO-probing it), while
heartbeating pstop to the laptop machine over the WireGuard tunnel.

| Metric | Result over the ingestion + steady window |
|---|---|
| peers pulled | **128 / 128** (cap), `ml_state=4` throughout |
| pstop cadence | exactly 10 Hz (1056 sent / 106 s), replies in lockstep |
| rebonds | **1** (the initial post-boot bond) — no storm |
| mismatches / send-fails | **0 / 0** |
| boot_count / crashes | 0 / none |
| comparator tick | never starved — 10 Hz held under full 128-peer DISCO load |
| 6-min steady hold | up 398 s, bc 0, 3729 sent, 1 rebond, **1 mismatch / 3729 ticks** (0.03 %, single core-publish-timeout, absorbed by machine loss-tolerance), 6/128 peers upgraded to direct |

This is the max-load case the earlier revision of this doc flagged as
unvalidated: the load bounds + shared-key cache + 128-peer NVS all held
together on a genuine 128-peer tailnet. Note the pstop RTT read ~110 ms
here — a **bench-topology artifact**, not a regression: the chip's only
uplink is the laptop's shared connection, so reaching the laptop's *own*
Tailscale IP hairpins through DERP. A remote and robot with independent
internet form a direct path (~20–70 ms, as measured in
`TRANSPORT_TEST_REPORT`).

Still worth doing later: a mass simultaneous trust-lease flap (toggle the
uplink with 128 peers bonded) to exercise the heartbeat-spread + retransmit
caps at scale.

## Optional follow-ups (not done)

- NaCl `beforenm` shared-key cache per known peer (~32 B/peer): removes
  the per-open/seal X25519 for known peers — a large multiplier on every
  budget above. Worth it if organic load ever approaches the budgets.
- Honor runtime `config.max_peers` as the actual `add_peer` bound (today
  the compile-time `ML_MAX_PEERS` is the only bound; the runtime knob is
  cosmetic).
