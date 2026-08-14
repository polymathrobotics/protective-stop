<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Heartbeat path diversity — relay-leg mirroring (design + resource analysis)

Status: DESIGN (operator GO 2026-08-14; resource analysis first per operator).
Goal: a heartbeat gap requires MULTIPLE simultaneous path deaths, so false
protective stops become rare at the EXISTING 2 s failsafe latency (operator
ceiling: ≤4 s; nothing here changes any timeout).

## 1. What already exists (and where it is blind)

`peer->dual_path` (wireguard_lwip) is ON for every safety peer, both roles:
when a DIRECT endpoint exists, every encrypted WG packet (heartbeats, replies,
rekeys) is also mirrored over DERP (`derp_output_fn`), and the receiver's
standard WG anti-replay window (RFC2401, wireguard.c:320) silently drops the
duplicate. Dedup is free, protocol untouched.

run-29/30 disarm ledger against that machinery — the blind spots:

| Blind spot | Ledger evidence | This design |
|---|---|---|
| B1. Relay-bound peer = DERP **single**-path (no direct endpoint ⇒ nothing to mirror; wireguardif.c:163 branch) | disarms #1–#3: relay-bound remotes gapped while machn's home conn reconnected | fix (relay-leg diversity) |
| B2. The DERP leg (mirror OR sole leg) routes to ONE conn (peer-region, fallback home; `derp_route_conn`) and **drops silently** if it's down | every storm window where home was mid-reconnect | fix (pool fallback + second leg) |
| B3. Stale direct endpoint (NAT wiped): direct leg blackholes; covered ONLY if the DERP leg survives | full-wipe disarms | hardened by B1/B2 fix |
| B4. wg_mgr/lwIP stall (machn-side, all-remote simultaneous gap) | disarms #4/#10/#12 class; `wg_max_iter_ms` gauge in run-30 | NOT addressed here (separate: ingest bounding) |

## 2. Design: safety-frame DERP leg diversity

One change, both roles, entirely in `ml_derp_queue_send()` + `derp_route_conn`
usage (components/microlink):

For frames whose destination is a SAFETY peer (existing
`ml_wg_is_safety_pubkey()` predicate at the enqueue path — already used for
priority-queue selection):

1. **Primary leg (today's):** peer-region conn, else effective-home conn.
2. **NEW second leg:** one ADDITIONAL connected pool conn, chosen as:
   the peer's home-region conn and the effective-home conn when both are up
   and DISTINCT; else any other CONNECTED pool conn (a DERP server only
   delivers to clients connected to it — but both ends hold multi-region
   pools via the safety want-set, so cross-region delivery is real: verified
   by the DUT's 4-slot pool in the field).
3. **Fallback (replaces silent drop):** if the primary is down, the frame
   goes to ANY connected conn instead of being dropped.
4. Non-safety frames: unchanged (single leg).
5. Receiver: no change anywhere — WG anti-replay dedups; DERP servers ignore
   frames for absent clients (today's semantics).

Failure algebra after: a heartbeat gap now requires
(direct dead) AND (DERP leg A dead) AND (DERP leg B dead-or-same-server-down)
inside one 2 s window. Tonight's ledger reread: every network-class disarm
had ≥1 surviving pool conn at the disarm instant.

## 3. Resource analysis (operator-required, first)

Frame: pstop heartbeat ≈ 60 B payload → WG transport ≈ 92 B → DERP
SendPacket framed ≈ 92 + 45 (hdr+key) ≈ ~140 B → TLS record ≈ ~170 B wire.

**Bandwidth** (worst case, all legs active):
- Remote (1 machine): 5 Hz × 1 extra leg ≈ **0.85 kB/s up** (plus existing).
- Machine (3 remotes, replies): 15 Hz × 1 extra leg ≈ **2.6 kB/s up**.
- At 10-remote scale: ~8.5 kB/s extra — noise against any uplink; DERP
  servers see one extra small TLS record per heartbeat.

**CPU** (per extra leg, machine @15 Hz worst case):
- No extra WG encrypt (the SAME encrypted packet is mirrored — encryption
  happens once, before transport fan-out).
- Per leg: one queue enqueue + one memcpy (~140 B) + one AES-GCM TLS record
  (HW AES, ~10–20 µs incl. overhead on S3).
- Total ≈ 15 × ~30 µs ≈ **<0.5 ms/s = <0.05 % CPU**. The prio-queue drain is
  already budgeted per loop (ML_DERP_TX_PRIO_DEPTH drained fully first).

**Memory:**
- Zero new buffers or structs. Each mirrored frame is one extra
  `ml_derp_tx_item_t` (existing pool/queue) alive for <1 loop iteration.
- Queue headroom: prio queue depth 12 vs machine worst case 3 remotes ×5 Hz
  ×2 legs = 30 items/s, drained every ~1 ms loop → instantaneous occupancy
  ≤2–3. At 10 remotes ×2 legs = 100/s → occupancy still <8. **Bump
  ML_DERP_TX_PRIO_DEPTH 12 → 16** as margin (16 × ~24 B item = +96 B RAM).
- Net RAM delta: **~+100 B**. Flash: ~1–2 kB code.

**Radio/coexistence:** DERP legs are TCP on the existing conns — no new
sockets, no new RF behavior.

## 4. Implementation sketch (~60–100 LOC)

- `ml_derp_queue_send()`: after the existing enqueue, if
  `ml_wg_is_safety_pubkey(dest)` → compute second leg conn (distinct,
  connected) → enqueue a second item tagged with that conn's region so
  `derp_route_conn` picks it; if the primary’s region conn is down at DRAIN
  time, existing home-fallback already applies — extend the final drop case
  to try any-connected before dropping (counter: `derp_tx_leg2 +
  derp_tx_rescue_routes` for observability).
- Item struct gains nothing (region_id field already routes).
- Both roles get it by construction (shared component).

## 5. Validation (operator directive #3)

Bench: verify duplicate suppression (machine `pstop_replies` rate unchanged,
WG replay drops counter increments), then **stop run-30 and start run-31**
(12 h zero-exclusion, same format) the moment the build is verified. Success
metric: network-class disarms → 0 through ≥1 real storm; only B4
(wg_mgr-stall class) may remain, tracked by `wg_max_iter_ms` correlation.

## 6. Explicitly out of scope
- Any timeout change (2 s stands; operator ceiling 4 s held in reserve).
- ESP-NOW/BLE sidechannel (separate feasibility study in flight).
- B4 wg_mgr ingest bounding (own fix, pending run-30 correlation data).
- pstop_c: untouched (transport-only; dedup is WG-native).
