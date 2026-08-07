<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Outbound cold bond over DERP — the definitive root cause (read this first)

**If a chip cannot ORIGINATE a WireGuard session to an off-link peer (`errno 128`
on the bond `sendto`, repeating every 5 s, "DISCO PONG unmatched … via DERP,
active_probes=0"), and the tailnet is small (< 128 nodes) so peer-cap trim is
irrelevant — you are hitting the single-DERP-home-region limit. It is NOT a
peer-cap trim. Stop looking at the peer count.**

This document exists because this exact class of failure has been rediscovered
and "fixed" at least five times (2026-05 through 2026-08), each time closing a
*different* link of the chain below and declaring victory. Read the whole chain
before touching code.

## The failure in one paragraph

The chip is its own Tailscale/WireGuard node (`microlink`). It keeps **exactly one
DERP relay connection**, homed on **one region** (the priority/fleet peer's
region, elected in `ml_wg_mgr.c::maybe_rehome_to_priority`, `:749-766`). The DERP
send path (`ml_derp.c::ml_derp_queue_send`, `:473` → `ml_derp_tx_task` →
`derp_send_packet`) has **no per-peer region routing** — every relayed frame
egresses that one home-region socket. A DERP server only relays between peers
connected to it, so a safety peer that homes on a **different** region never
receives the chip's relayed WireGuard handshake init or DISCO ping. The session
never forms → the bond `sendto` has no keys → `errno 128` (lwIP `ERR_CONN`,
"No valid keys"). The far peer still reaches the chip *inbound* (it opens the
chip's advertised `PreferredDERP` region itself), so relayed pongs/CallMeMaybe
arrive — producing the tell-tale **directional asymmetry**: inbound works,
outbound origination is dead.

## Why it kept getting rediscovered — the four-precondition chain

An outbound cold bond over DERP needs ALL FOUR of these to hold. Each was fixed
and declared "solved" in isolation, on a bench where the *other three were
satisfied by luck* (usually because the far end was a full-tailscaled laptop that
homed on the same region and upgraded to a direct path):

| # | Precondition (the chip must…) | Landed fix | Fingerprint when THIS link breaks |
|---|---|---|---|
| **A** | home/reach a DERP connection on the **peer's region** | `777be05` (rehome to priority region) | link only forms after a manual `tailscale ping`; **errno 128** on a small tailnet |
| **B** | send its INIT **over DERP**, not to a dead unvalidated direct endpoint | `827386f` (add_peer DERP-only) | silent blackhole, rx flat — **not** errno 128 |
| **C** | have the far peer **hold its key** (not 128-cap trimmed) | `785ac53` (machine pins operators) | errno 128 on a **> 128-node** tailnet, `send_fail` climbing |
| **D** | get the far peer to **lazy-add it** on first INIT (no zombie keypair) | v15.8 active-INIT + `8f0d6d2` health-kick | INIT arrives but session never completes |

**Precondition A is the one that stayed open**, because the chip's DERP transport
is architecturally single-home-region. It only *looks* solved on a bench where the
chip and its peer happen to share a region (all Polymath bench units home on
region 2/sfo, so ESP32↔ESP32 bonds fine and hides the bug). It fails against any
peer on another region: full-Tailscale machine runners / ROS2 nodes / operator
handhelds that home on their own nearest region, or genuinely geographically
distributed remotes and machines.

## errno 128 has TWO causes — disambiguate before acting

`errno 128` = `ENOTCONN` = lwIP `ERR_CONN` = "peer is in the WG table but there
are no session keys." That is the symptom of **both**:

- **Precondition C (peer-cap trim):** the far peer trimmed the chip from its
  128-peer netmap. Only possible on a **> 128-node** tailnet. Fix: pin (make the
  remote an operator on the machine) or prune the tailnet. `/admin/api/peers`
  shows length 128 on both ends.
- **Precondition A (region-home mismatch):** the chip's one DERP connection is on
  a different region than the peer, so its INIT never reaches the peer, so the
  peer never keys up. Possible on **any** tailnet size, including a 3-node bench.
  `/admin/api/monitor` `derp_home_region` ≠ the peer's region; the peer is absent
  from the chip's outbound but present in inbound.

`errno 118` (`EHOSTUNREACH`/`ERR_RTE`) is different again — "NO PEER FOUND", the
peer isn't in the WG table at all (netmap-absent). Don't confuse the two.

**Triage:** if the tailnet is < 128 nodes and you see errno 128, it is
**precondition A (region homing)**, full stop — do not go down the peer-cap path.

## The fix (DEPLOYED + VALIDATED — `7867d4b` on main, 2026-08-05) — multi-region DERP relay

> **Status: precondition A CLOSED.** Validated on the deployed fleet build via the
> reproduce rig below on real silicon: forcing a bonded remote's `derp_region`
> override to region 9 (mismatched with its machine's region 2) opened an
> auxiliary DERP conn to region 2 (`pool=[(0,9),(1,2)]`) and the safety bond held
> unbroken — `bond=2`, rebonds flat, **zero errno-128**, heartbeats flowing, **no
> manual `tailscale ping`.** An earlier airtight run forced `wg_direct=0` and
> confirmed frames delivered via the aux conn with zero errno-128. Restoring the
> home region reaps the now-redundant aux (time-since-unwanted reap, `c571580`).
> The Tailscale-ACL / netmap-bounding half is a separate deployment concern
> (`TAILSCALE_ISOLATION.md`), not firmware.

The durable fix is the magicsock model: the chip maintains a **small pool of DERP
connections** (home region + one per distinct region among its pinned/priority
safety peers, capped at `ML_DERP_MAX_CONNS`) and routes each peer's relayed frames
to **that peer's** home region. mbedTLS is already PSRAM-resident
(`CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y`) with dynamic buffers, so extra connections
cost PSRAM, not internal RAM; the relay work stays on the dedicated DERP task(s),
never the wg_mgr/heartbeat loop. Scoped to pinned safety peers so it respects the
peer-scaling armor (`PEER_SCALING_DESIGN.md`: non-priority sessionless peers must
not retry-storm). Two aggravators are fixed alongside so a relayed path can
self-heal: `process_disco_pong` no longer ignores a `via_derp` pong for liveness,
and pinned-peer retransmit state is not wiped each tick.

Complementary (deployment, not a substitute): pin all pstop nodes to a shared
DERP region and deploy `TAILSCALE_ISOLATION.md` grants so the netmap is bounded.

## How to REPRODUCE it on a bench (the step every prior fix skipped)

You cannot rely on two physical sites. Force a region mismatch instead: override
one unit's `derp_home_region` to a region different from its bond peer's (see the
`derp_region` override in settings / `POST /admin/api/settings`), reflash, and
point its slot at the peer. It will fail with errno 128 while both are healthy on
the tailnet — the controlled cross-region repro. The fix makes it bond with **no**
manual `tailscale ping`. Validate against this rig before declaring victory.

## See also

`DERP_REACHABILITY_INVESTIGATION.md` (precondition B, inbound return-flow),
`TWO_SITE_FAILOVER_2026-07-21.md` (precondition A/D, but tested against a
direct-upgrading laptop), `PEER_SCALING_DESIGN.md` (the armor a fix must respect),
`MULTI_REMOTE_MULTI_MACHINE.md` §5–6, `TAILSCALE_ISOLATION.md`.
