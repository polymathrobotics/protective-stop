<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# DERP-relay reachability failure — firmware investigation

**Working assumption (deliberate bias): this is solvable in ESP32 firmware.**
A laptop running full Tailscale on the *same* tailnet, behind the *same* class
of NAT, does **not** suffer this — so the gap is in the chip's lightweight
WireGuard/DERP stack (`microlink`), not an unavoidable network law. Infra
workarounds (subnet router, self-hosted DERP) are to be treated as **last
resort, accepted only with definitive proof the fix is impossible in firmware.**

_Investigation date: 2026-08-02. Author host tailnet IP: 100.110.35.58._

## 1. Symptom

From this host, the remote DUT **pstop-01d7f344 @ 100.75.70.74** (fleet id
`3c0f02d7f344`, at a remote bench) is reachable only via **DERP(sfo) relay**;
no direct WireGuard path forms. Over that relay:

| Probe | Result | Implication |
|---|---|---|
| `tailscale ping` (disco) | ✅ pong via DERP(sfo), 40–160 ms | small UDP relays fine |
| TCP `connect :80` | ✅ 3-way handshake completes | SYN/SYN-ACK/ACK traverse |
| HTTP GET `/state.json` | ❌ **"Empty reply"** / timeout | **response data never returns** |
| HTTP/1.0 | ❌ same | not keep-alive/chunked specific |
| MSS clamp to 1140 (my OUTPUT SYN) | ❌ no change | not a segment-size blackhole |
| relay-3 power-cycle of the DUT | ❌ no change | not a wedged httpd |
| DUT → fleet check-in (outbound) | ✅ works | **outbound path is healthy** |

Path MTU over the relay measured ~1250 B (1200 payload OK, 1236 dropped) — but
the MSS-clamp result rules MTU size out as the whole cause.

## 2. The key asymmetry (the crux)

The DUT's **SYN-ACK** (one small packet, DUT→me) *does* arrive — so the chip can
put at least one packet back to me over DERP. Yet the **HTTP response** (also
small, < MTU) never arrives. And the DUT's **outbound** fleet check-ins work.

So: **my-host→DUT works; DUT→my-host works for the handshake but not for the
subsequent flow.** That is a *path-selection / fallback* behaviour, not a raw
connectivity wall.

## 3. Leading firmware hypothesis

After the incoming handshake the chip **learns my endpoint and switches to a
DIRECT path** for the return flow; the bench NAT one-way-breaks that direct
attempt; and the chip **does not fall back to DERP for the return flow** (or
roams onto a dead endpoint and stays there). The "direct-path healing /
send-fail" fixes (`66df9ae`) address this area but evidently not for hard NAT.

Full Tailscale (the laptop) avoids this because it does continuous **disco path
discovery + DERP-home + bidirectional path validation** and never commits to a
direct endpoint it hasn't verified *both ways*. The chip's `microlink` stack
does endpoint *learning* but not the full validation/fallback state machine —
**that is the firmware gap to close.**

## 4. Candidate firmware fixes (for the subagent to investigate + test)

Ranked cheap→involved:
1. **Fleet-configured peer (cheapest, test FIRST).** It may be enough to have
   **pstop-fleet configure the DUT with a peer** (the management host / a
   machine) so the DUT initiates + maintains the link *outbound*, which is the
   direction that already works. The OTA-push channel already proves the DUT
   pulls config outbound. **If this alone restores reachability, document it
   loudly here so we stop rediscovering it.**
2. **Never use an unvalidated direct endpoint.** Require a bidirectional
   round-trip (challenge/response) on a candidate direct path before switching
   off DERP; keep DERP as the live path until then.
3. **DERP fallback for the return flow.** On repeated send-fail to a direct
   endpoint, fall back to DERP mid-flow (don't strand the return path).
4. **Sticky DERP-home + keepalives.** Maintain the DERP relay session with
   periodic keepalives so relayed data always has a working path.
5. **MTU/MSS correctness over DERP.** Ensure the WG netif MTU + TCP MSS clamp
   match the DERP-reduced path (~1250) so no data segment blackholes.
6. **Study tailscaled's path selection** (disco, ICE-like probing, DERP-home,
   endpoint verification) and port the missing pieces into `microlink`.
7. **Resource budget is not a blocker** — extra state/buffers in **PSRAM
   (external)** are acceptable if that's what robust fallback needs.

## 5. Why this matters for production (context, not scope-creep)

Production remotes (handhelds) and machines (robots) will sit behind customer
CGNAT / corporate firewalls and will frequently be **DERP-only**. If the chip
can't carry its own safety-link traffic over DERP under hard NAT, the pstop
function's **availability** degrades in exactly those conditions (fail-safe: the
machine STOPs, but the robot can't run). The same fix that restores admin HTTP
restores the **safety link's** robustness. Complementary production directions
(secondary to the firmware fix): self-hosted DERP near sites; same-LAN direct
when remote+machine are co-located; an outbound-initiated fleet command channel
for management so inbound reachability is never required.

## 6. Test assets available to the subagent

- **Remote DUT:** `100.75.70.74` (the broken one — the target). Admin OTA:
  `POST /admin/api/ota` with `admin:<CONFIG_ML_ADMIN_PASSWORD from
  firmware/sdkconfig.credentials>`. Relays via the bench console
  `http://100.116.240.10/pstop/api/relays/{ch}/{on|off}` (ch3 = DUT power).
- **Reference "works" case:** this host's full Tailscale (100.110.35.58) —
  study why it succeeds where the chip fails.
- **Fleet:** bench console proxies `http://100.116.240.10/pstop/fleet-api/…`;
  fleet device id `3c0f02d7f344`; OTA-push at `fleet-api/remotes/<id>/push`.
  Fleet creds live in the gitignored `tools/app_source.py`. **The fleet IP is
  PROPRIETARY — never commit it to the public repo.**
- Local units `10.42.0.41`, `10.43.0.195` are in use by other agents — leave
  them alone; use the remote DUT + this host.

## 7. Status log

- 2026-08-02: opened. Root cause not yet fixed; investigation handed to the
  firmware networking subagent (branch work). **Update this file with the fix.**

- 2026-08-02: **ROOT CAUSE FOUND + FIXED IN FIRMWARE.** Remote DUT
  `100.75.70.74` now reachable over DERP. Branch `fix/derp-firmware-reachability`,
  build `ad75887-dirty` (sha256 `8dfbf4ef…`), fleet-pushed to `3c0f02d7f344`.

  ### Root cause — the chip installed an UNVALIDATED direct endpoint as the live WG endpoint
  This was candidate #2 (not #1). The chip was trusting the netmap/STUN-advertised
  endpoint as a working direct path *without any DISCO validation*:

  1. `ml_wg_mgr.c::add_peer()` passed the first netmap endpoint
     (`p->endpoints[0]`) into `wg_peer.endpoint_ip` for every peer that
     advertised one.
  2. `wireguardif_add_peer()` (`wireguardif.c:1084-1087`) copies that straight
     into `peer->ip` / `peer->port` — the *live* endpoint.
  3. `wireguardif_peer_output()` (`wireguardif.c:158`) routes on that field:
     `peer->ip` non-any ⇒ send via **direct UDP** (`wg_udp_output_cb`), and the
     DERP relay callback (`wg_derp_output_cb`) is used *only* when the endpoint
     is blank. So **every WG packet — including the handshake RESPONSE — went to
     the unvalidated direct endpoint and never to DERP.**

  Behind the DUT's hard / endpoint-dependent NAT that endpoint is unreachable in
  the peer→chip direction, so everything the chip sent blackholed. Meanwhile
  relayed **DISCO pongs still worked** because `process_disco_ping()` sends the
  pong *explicitly over DERP* (`ml_derp_queue_send`), bypassing the WG endpoint
  selection entirely. That is precisely the observed asymmetry: **`tailscale
  ping` OK / TCP + HTTP dead**, relay `rx` stuck at ~368 B.

  Worse, for a **non-priority** peer this state was *permanent*: the direct→DERP
  fallback in `disco_periodic_probes()` is gated on `has_direct_path`, but that
  flag is only ever set by a validated DISCO pong — never by the netmap install.
  So the peer had `peer->ip = <dead direct>` with `has_direct_path = false`, and
  the fallback that would have reverted it to DERP never fired.

  Full Tailscale never hits this: magicsock is **DERP-homed** and only promotes a
  direct endpoint after DISCO validates it **both ways**. The chip did endpoint
  *learning* but skipped that validation gate — exactly the firmware gap §3
  predicted.

  ### The fix (one site, `ml_wg_mgr.c::add_peer`)
  Never install a netmap-advertised endpoint as the live WG endpoint. Always add
  the peer DERP-only (`wg_peer.endpoint_ip = any`, `port = 0`); keep the netmap
  endpoints in `p->endpoints[]` purely as DISCO probe candidates. The live direct
  endpoint is now installed **only on proof of bidirectional reachability**:
  - a txid-matched DISCO **direct pong** → `process_disco_pong()` →
    `wireguardif_update_endpoint()` + `wireguardif_connect()`, or
  - a genuinely **received direct WG packet** → `update_peer_addr()` roaming
    (DERP-injected packets carry src `0.0.0.0` and are correctly skipped).

  DISCO probing of `p->endpoints[]` is unchanged, so genuinely reachable direct
  paths still upgrade within one probe interval; the existing lease/pong-watchdog
  reversion (`wireguardif_connect_derp`) still demotes a dead direct path back to
  DERP. This is the magicsock model: DERP-home first, upgrade only after
  validation. Priority (safety) peer is unaffected in spirit — it already gets an
  immediate forced DISCO probe on add plus `dual_path` mirroring over DERP during
  any direct transition, so its failover window is if anything *safer* (no
  blackhole window from a bogus netmap endpoint).

  ### Evidence (before → after)
  | | Before (`3ba0633`) | After (`ad75887-dirty`) |
  |---|---|---|
  | `curl http://100.75.70.74/state.json` | timeout / empty reply | **HTTP 200, 10/10, ~0.16 s** |
  | TCP `connect :80` | timed out | connects |
  | relay counters | `tx 2784 rx 368` (rx flat) | `tx 9668 rx 33324` (rx climbing) |
  | path | DERP(sfo), no direct | DERP(sfo), no direct (same NAT condition) |
  | admin monitor (2216 B body) | dead | HTTP 200 |

  Reproduced the *same* DERP-only condition that failed before
  (`tailscale ping … via DERP(sfo)`, "direct connection not established") and it
  now serves HTTP reliably. `wg_pbuf_fails=0`, `wg_dedup_drops=0`,
  `free_heap≈8.1 MB` — no resource pressure.

  ### Files changed
  - `components/microlink/src/ml_wg_mgr.c` — `add_peer()`: start every peer
    DERP-only; do not install unvalidated netmap endpoints.

  Note: candidate #1 (fleet-configured peer) was **not** needed — inbound HTTP
  from an arbitrary tailnet host is restored purely in firmware. No infra
  workaround (subnet router / self-hosted DERP) required.
