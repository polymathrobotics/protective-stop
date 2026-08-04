# Connectivity soak: `tools/soak_disconnect_monitor.py`

A diagnostic / acceptance-validation tool for long-run remote &harr; machine
connectivity over Tailscale. It polls one or more pstop **remotes** and one or
more **machines** on a fixed cadence, records every connectivity disruption with
enough multi-unit context to tell the causes apart, and can exit on an
"N-hours-clean" acceptance criterion.

It is **not part of the shipped firmware**. It only reads `/state.json` and
(optionally) `/admin/api/monitor` over HTTP — it never writes to a device. Use it
on a bench or a staging tailnet to prove a build stays bonded over hours, or to
capture the fingerprint of a disconnect you are chasing.

It grew out of the bench harness that found and validated the recent firmware
disconnect bugs; this version is generalized (targets, creds, cadences, and
duration are all arguments) and stdlib-only (`urllib`, `json`, `argparse`), so it
runs anywhere Python 3 does with no install step.

## Running it

Watch two remotes and one machine for 12 h, capturing the DUT's USB console, and
declare success after 8 h with no events:

```bash
python3 tools/soak_disconnect_monitor.py \
    --remote PSTOP54=http://192.168.107.70 \
    --remote DUT=http://pstop-01d7f344 \
    --machine machn=http://100.84.155.111 \
    --admin-user admin --admin-pass "$ML_ADMIN_PASSWORD" \
    --machine-admin-pass microlink \
    --duration 43200 --out ./soak_run \
    --serial /dev/ttyACM1 --clean-target 28800
```

- `--remote NAME=URL` / `--machine NAME=URL` are repeatable; `NAME` is just the
  label used in the logs. The machine URLs are only needed for the
  `bonded_remotes` context attached to each event — you can run with remotes
  only.
- Admin passwords are read from `--admin-pass` / `--machine-admin-pass` or the
  env vars `ML_ADMIN_PASSWORD` / `ML_MACHINE_ADMIN_PASSWORD`. Without them the
  DERP-rehome telemetry columns are simply left blank; disconnect detection does
  not depend on them.
- `--machine-id 0x01020304` selects the machine slot on each remote by id when a
  remote is bonded to several machines. Default is the first configured slot.
- `--poll-sec` (default 5) is the disconnect-detection cadence; `--telem-sec`
  (default 30) is the finer telemetry cadence.
- `--serial DEV` captures a single USB console (typically the DUT) and tails it
  into every event record.
- `--on-disconnect CMD` runs a shell command once per detected event — a generic
  replacement for the old bench-only HIL button reset (see *Left out*, below).

It never crashes on a network error: any failed poll is swallowed and the loop
keeps going, so the monitor outlives the disruptions it is watching. `Ctrl-C`
stops it and leaves the partial logs in `--out`.

## Output files (in `--out`)

| File | Contents |
|------|----------|
| `events.log` | One timestamped block per disruption: the trigger(s), the remote's slot line (`state`, `sent`, `replies`, `send_fail`, `rebonds`, `rtt_ms`, `wg_direct`, `last_reply_ms`), its net line (`ml_state`, `ml_reconnects`, `eth_link`, `eth_recoveries`/reason, `pstop_sf_route`/`sf_txdrv`/`sf_errno`, `wg_pbuf_fails`, `gw_rtt_ms`), every machine's `bonded_remotes` view, and the console tail if `--serial` is set. This is the primary artifact. |
| `samples.csv` | One wide row per remote per poll: slot state/sent/replies/send_fail/rebonds/rtt/wg_direct plus ml_state/ml_reconnects/eth_link/eth_recoveries. The continuous time series behind the events. |
| `telemetry.csv` | One row per unit per `--telem-sec`: `ml_reconnects`, `ml_state`, per-slot `rtt_ms`/`wg_direct`, `rebonds`, `replies`, `send_fail`, and (with admin creds) `derp_home_region` + `pp_has_direct`. The fine-grained signals that validate the re-sync / DERP-rehome behaviour. |
| `status.txt` | Rewritten every poll: elapsed, per-remote event counts, the current clean streak vs `--clean-target`, and each remote's latest state. Good for a `watch cat`. |

## What the tool detects

Each is evaluated per remote, sample over sample, against the real `/state.json`
shape (`pstop_machines[]`, `bonded_remotes[]`, the `pstop_sf_*` / `ml_reconnects`
/ `eth_rec_*` top-level counters):

| Trigger | Meaning |
|---------|---------|
| `peer bond DROPPED 2->x` | The remote's machine slot `state` fell from 2 (bonded). |
| `rebonds a->b` | The remote re-established a bond — a transient loss that self-healed. |
| `ml_reconnects a->b` | A control-plane (Tailscale/DERP) reconnect. |
| `eth_recovery reason=...` | The W5500 Ethernet watchdog recovered the link. |
| `reply STALL Ns` | The slot keeps `sent` advancing but `replies` is frozen — the far side went quiet while we kept transmitting. |
| `state.json UNREACHABLE` | The unit's own HTTP endpoint stopped answering (unit reboot, uplink down, or host unreachable). |

## Reading a disconnect event — which cause?

The point of the multi-unit snapshot is to localize a disconnect. Cross-reference
[`docs/MULTI_REMOTE_MULTI_MACHINE.md`](MULTI_REMOTE_MULTI_MACHINE.md) §7
(*Troubleshooting: "remote won't bond to the machine"*):

- **`pstop_sf_route` &ne; 0 / `sf_errno` 118 (ENETUNREACH)** &rarr; no route: the
  machine peer is not in the remote's netmap or WG is down. Check the remote's
  `derp_home_region` in `telemetry.csv` and its `/admin/api/monitor`
  (`wg_paused`, `ts_boot_en`). This is the netmap / boot-safety-pause class.
- **`sf_errno` 128 (ENOTCONN) with slot `state=1`, `sent` climbing, `send_fail`
  climbing** &rarr; **disambiguate — errno 128 has two causes.** FIRST check the
  tailnet size. **(a) Small tailnet (< 128 nodes):** it is NOT a peer-cap trim — it
  is an **outbound cold-bond / DERP region-home** failure (the chip can't relay its
  handshake to a machine on a different DERP region). Check `/admin/api/monitor`
  `derp_effective_home_region` + `derp_pool[]` vs the machine's region; the
  multi-region relay (2026-08-04) should open an aux conn on the machine's region.
  See **`docs/OUTBOUND_COLD_BOND.md`**. **(b) Large tailnet (≥ 128 nodes):** the
  **peer-cap trim** — the machine dropped the remote from its 128-entry netmap
  (its `bonded_remotes` line will *not* list this remote). Fix per §5 / §7: make
  the remote an operator (pins it), prune below 128, or co-locate on one LAN.
- **`ml_reconnects` incrementing while `rebonds` stays flat and the bond returns
  quickly** &rarr; a control-plane flap that the transport rode through; the
  heartbeat likely never gapped. Corroborate with `wg_direct` and
  `derp_home_region` in `telemetry.csv` (a `wg_direct` 1&rarr;0 flip means the
  path fell back from direct WireGuard to DERP relay).
- **`eth_recoveries` incrementing with `eth_rec_reason` set** &rarr; a physical /
  link-layer Ethernet event that the W5500 watchdog handled; unrelated to the
  tunnel.
- **`reply STALL` with `send_fail` flat** &rarr; we are transmitting fine but the
  machine stopped replying: look at that machine's `bonded_remotes` `age_ms`
  (stale = the machine also thinks the remote went silent) and its
  `relay_stop`/state.

`rtt_ms` is a coarse health hint, not a path signal — `wg_direct` and
`derp_home_region` are what actually distinguish a direct WireGuard path from a
DERP-relayed one.

## The "N-hours-clean" acceptance pattern

For a release/acceptance soak, set `--clean-target` to the number of seconds you
require with **zero** new events. The tool exits **0** the moment that clean
streak is reached (any event resets the streak), so it slots straight into CI or
a scripted gate:

```bash
python3 tools/soak_disconnect_monitor.py \
    --remote R1=http://... --machine M=http://... \
    --duration 90000 --clean-target 28800 --out ./accept_run
echo "exit=$?"   # 0 = 8h clean streak achieved before --duration ran out
```

`--duration` is the hard ceiling; make it comfortably larger than
`--clean-target` so there is room for the streak to accumulate. If the run ends
by hitting `--duration` instead, it still exits 0 — `events.log` and the per-run
event counts in `status.txt` are the verdict in that case, not the exit code.

## Left out on purpose

The originating bench harness pressed a HIL relay button to auto-recover a
device-under-test after a disconnect. That is fixture-specific (relay URLs,
channel wiring, open=pressed polarity) and is deliberately **not** shipped here.
Use the generic `--on-disconnect CMD` hook if you need equivalent behaviour on
your own rig, e.g. `--on-disconnect 'python3 tools/usb_relay4.py pulse 1 2'`.
The multi-hour checkpoint files the bench version wrote are also dropped;
`status.txt` plus the CSVs cover the same need without hardcoded checkpoint
offsets.
