<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Production networking — safety-link robustness under real NAT

The pstop safety link (remote ↔ machine) rides WireGuard over a Tailscale-style
mesh. In production, remotes (handhelds) and machines (robots) sit behind
customer CGNAT / corporate firewalls, so the link will frequently be
**DERP-relayed** rather than direct. It is fail-safe (a degraded link → machine
STOPs) but availability matters: a robot can't run when its link is down.

## 1. The core firmware fix (done — `ml_wg_mgr.c`)
The chip was installing an **unvalidated** netmap/STUN endpoint as the live WG
endpoint, so under a hard/endpoint-dependent NAT every packet the chip sent
(incl. handshake responses) blackholed while relayed DISCO pongs still worked —
the "ping OK / data dead" failure. Fix: **DERP-home every peer; promote a direct
endpoint only after bidirectional DISCO validation** (tailscaled/magicsock's
model). Validated 2026-08-02:

| Scenario | Before | After |
|---|---|---|
| Hard-NAT remote DUT (DERP-only) | HTTP dead; safety link couldn't form | **HTTP 200 reliably; full HIL smoke (arm/press→STOP/discordance) over DERP** |
| Co-located / same-LAN unit | direct path | **still DIRECT** (`tailscale ping … via 10.43.0.195:51820, 20 ms`) — no regression |

So a validated direct path still forms within one probe interval; only the
*unvalidated* commit is removed. This is the primary, sufficient fix.

## 2. Robustness layers (secondary — deploy per site as needed)
Ranked by leverage; none is required for correctness, all improve availability
under adverse NAT:

1. **Same-LAN direct preference.** When a remote and its machine are co-located
   (the common case — a handheld near its robot), keep them on a network where
   the direct WG path forms locally (confirmed working above). Lowest latency,
   no relay dependency. Prefer this in deployment layout.
2. **Self-hosted DERP near sites.** For links that must relay, a DERP node close
   to the deployment cuts latency/jitter and removes dependence on public DERP
   capacity. The safety heartbeat (400 ms / 2.0 s timeout) tolerates the
   measured public-DERP RTT (~40–160 ms) today, but a local DERP widens the
   margin and improves determinism — worth it for short-PST vehicles.
3. **Outbound-initiated fleet command channel (management, not the safety
   link).** Never rely on *inbound* reachability to a NAT'd device for admin /
   diagnostics / OTA. The device already pulls OTA outbound; generalize that to
   a command/telemetry channel (device polls the fleet, executes, posts result).
   Then admin access is a non-issue regardless of NAT. (Device side is a bounded
   firmware addition; needs a fleet-side command queue.)

## 3. Heartbeat-timing under relay — validated tolerance
The safety link's liveness = `heartbeat_ms × max_missed` = 400 × 5 ≈ 2.0 s
(`max_missed` raised 3 → 5 on 2026-08-04 to ride through ~90-min Tailscale
control-plane re-syncs; was 400 × 3 ≈ 1.2 s).
Over the public DERP relay the remote DUT bonded and streamed OK/crc=ok with the
machine armed and STOPping correctly on press/discordance — i.e. the timing
budget already absorbs public-DERP latency + jitter. Re-confirm this margin for
any short-PST (fast/heavy) vehicle against its process-safety-time (HARA OD-2).

## 4. Status
- [x] Core firmware fix — merged + hardware-validated (hard-NAT + same-LAN).
- [ ] Self-hosted DERP — deployment/infra choice per site (not firmware).
- [ ] Outbound fleet command channel — firmware device-side prototype pending a
      fleet-side command queue (management robustness, not safety-critical).
