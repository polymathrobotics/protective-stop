# Two-Site Failover & Transport Robustness — 2026-07-21

Live validation of the pstop remote (`pstop-01d7791c`, ESP32-S3 on the USB-NCM
bench tether) bonding to a **real, geographically remote** machine node running
`machine_app_runner` on a laptop (`iliabara-framework16`, `100.81.222.123`) over
Tailscale/WireGuard. framework16 is NAT'd/DERP-homed on `sfo`; the remote reaches
it across the public internet — a realistic robot↔operator split, not a LAN loop.

## Fixes validated in this run

1. **Autonomous cold connect (no `tailscale ping` from the machine).** Three
   root causes were found and fixed:
   - **DERP region mismatch** (the decisive one). microlink holds a single DERP
     connection and a DERP server only delivers to peers connected to it. The
     chip reported a hardcoded `PreferredDERP` (`ML_DERP_REGION`=dfw) so it homed
     on dfw while the machine homed on sfo; its relayed DISCO/WG-init never
     reached the machine. Fix: `maybe_rehome_to_priority()` homes the chip's DERP
     connection on the **priority peer's** region and advertises it as
     PreferredDERP (`ml_wg_mgr.c`, `ml_coord.c`; commit 777be05).
   - **Zombie keypair** after the peer restarts: `microlink_notify_priority_health()`
     lets the app force a fresh handshake instead of waiting out a dead-but-"up"
     session (commit 8f0d6d2).
   - **Disco-first initiation** so a bare DERP WG-init isn't required to be
     answered first (commit 6d28757).
2. **Per-unit identity**: node `pstop-01d7791c` / device ID `0x01d7791c`; VPN IP
   `100.117.20.69` stable across OTA.

Result: chip cold-boots → bonds to framework16 with **no manual intervention**,
upgrades to a **direct path**, machine receives the (correct, fail-safe) STOP
heartbeats continuously — this remote has no operator switch wired, so STOP is
its correct verdict.

## Failover ladder (`test/netem_ladder.sh`)

tc+ifb netem applied to the direct WireGuard underlay (UDP 51820) only, in both
directions, with qdisc packet counters logged to **prove** each impairment was
actually in force. HTTP/tether untouched. Sampled every 5 s.

| Phase | Impairment (300 s blackhole, else 180 s) | Δsent | Δrepl | reply gap | new rebonds | RTT max | Outcome |
|-------|------------------------------------------|-------|-------|-----------|-------------|---------|---------|
| n0 baseline | none (120 s) | 1073 | 1058 | 15 | 1 (initial settle) | 137 ms | healthy |
| n1 | 10 % loss | 316 | 302 | 14 | 0 | 115 ms | tolerated; egress drifted to DERP |
| n2 | 150 ms delay | 1674 | 1673 | 1 | 1 (at onset) | 301 ms | tolerated |
| **n3** | **100 % blackhole** | **2989** | **2989** | **0** | **0** | 226 ms | **gapless DERP failover** |
| n4 restore | none (240 s) | 2381 | 2381 | 0 | 0 | 127 ms | direct path re-upgraded |

### Headline

During the **300 s total blackhole of the direct WireGuard path** — proven by the
UDP 51820 qdisc counters freezing (egress `1692→1692`, ingress `6199→6199`, i.e.
zero packets) — heartbeats **continued uninterrupted over DERP**: `Δsent == Δrepl
== 2989`, **zero reply gap, zero rebonds**. The safety link never dropped despite
the direct underlay being 100 % dead.

- **Total rebonds across the entire ~17 min chaos run: 2** — both at benign
  transitions (initial settle, delay onset); **none during the blackhole**.
- `send_fail = 0` throughout. Post-test: back on a **direct** path, bonded,
  healthy (15 000+ heartbeats, ~0.2 % cumulative reply lag).
- The chip fails over to DERP eagerly (even under 10 % loss egress moved off the
  direct path) — robustness-biased, correct for a safety link. RTT to this remote
  machine is ~100 ms on both direct and DERP, so the failover is latency-neutral
  here.

### Fail-safe note

The machine's STOP-on-loss is inherent and was continuously exercised (this
remote sends STOP by design). A *true* total loss (both direct **and** DERP) would
freeze heartbeats → machine heartbeat-timeout → STOP, then self-heal on restore
via the health-driven re-handshake. The ladder shows a single-path kill does
**not** even reach that state, because DERP carries the link through.

## Additional path-failure cases (chip transport toggles)

Complementing the underlay ladder, exercised via the chip's own reversible
pause toggles:

- **DERP kill, direct path up** (`/api/derp` paused ~24 s): heartbeats continued
  **seamlessly** on the direct path — `sent`/`repl` lockstep, **0 new rebonds**.
  The mirror image of n3: killing either single path leaves the other carrying
  the link.
- **Full transport cut → self-heal** (`/api/wg` paused ~20 s): `sent`/`repl`
  froze (link fully down → machine heartbeat-timeout → **STOP**, the intended
  fail-safe), 1 rebond fired. On restore, **replies resumed within ~3–6 s** with
  no manual intervention (1 further rebond). This is the both-paths-down case the
  single-path tests never reach.

Total rebonds across the entire session (ladder + both toggle tests) = 4, each
tied to a deliberate cut; final state healthy, direct path, ~0.36 % cumulative
reply lag.

## Summary

| Failure | Fail-safe? | Link maintained? | Recovery |
|---------|-----------|------------------|----------|
| 10 % loss (direct) | n/a (stayed up) | yes | — |
| 150 ms delay (direct) | n/a | yes | — |
| 100 % blackhole (direct only) | n/a (DERP carried it) | **yes, gapless** | 0 rebonds |
| DERP down (direct up) | n/a | **yes, seamless** | 0 rebonds |
| Both paths down | **yes — machine STOP** | no (by design) | **~3–6 s autonomous** |

The transport survives any single-path failure without interrupting the safety
heartbeat, and a total loss degrades safely to STOP with fast autonomous
recovery. No manual `tailscale ping` is ever required.

## Artifacts

- `test/netem2.csv` — per-sample metrics.
- `test/netem_phases2.log` — per-phase qdisc packet counters (impairment proof).
