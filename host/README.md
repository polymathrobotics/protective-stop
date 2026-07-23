# machine_app_runner — robot-side pstop machine

Companion to the remote firmware in `firmware/`. The remote runs as a
**pstop client** (lockstep verdict from cores 0 + 1, byte-compared, sent
over UDP at 10 Hz); this runs the **pstop machine** that accepts those
heartbeats and tells the robot to STOP when they say so — or when they
stop coming.

The upstream pstop_c library (`pstop_c/`, submodule,
certification track) is linked unmodified. This wrapper adds:

- transport: binds `0.0.0.0` (configurable) instead of `127.0.0.1`, so
  the remote can reach it over Tailscale, USB-NCM, or plain LAN;
- config: every library knob + wrapper policy in `machine.toml`
  (documented inline);
- **min-STOP-duration arming policy** (`min_stop_ms`, default 500 ms): a
  STOP episode shorter than this cannot arm the robot on release —
  vetoed via the library's public `machine_stop_robot()`. Defends
  against EMC-induced loop blips performing the arming gesture. See
  `docs/FAILOVER_AND_ARMING_DESIGN_2026-07-21.md`;
- a status latch: OK is only propagated after the policy approves the
  arming — **any real actuation must hang off the latched status, never
  the library's raw callback**;
- sequence-anomaly logging (counter gaps/regressions, non-monotonic
  stamps, pstop rejections with direction disambiguation).

## Build

```sh
make
```

No ESP-IDF needed — just `cc` (gcc or clang) and the vendored pstop_c
sources. Builds to `./machine_app_runner`.

## Run

```sh
./machine_app_runner                  # ./machine.toml, 0.0.0.0:8890
./machine_app_runner my.toml          # different config
./machine_app_runner 9999 -v          # port override + verbose RX/TX trace
```

Stderr prints per-pstop command changes and state transitions by default —
`pstop 0x<id> -> STOP/OK/BOND/...`, `Robot Status = OK` / `STOP`,
`ARMED by 0x<id> ...`, `ANOMALY: ...` — each line naming the remote by its
device id, so with several remotes bonded you can see exactly which one sent
what (and which one caused a stop or owns the arming cycle).

## Point the remote at this host

USB-NCM (host side of the tether is `10.42.0.1` under the default
NetworkManager share):

```sh
curl -X POST "http://$CHIP/api/pstop_peer?ip=10.42.0.1&port=8890"
```

Tailscale (the normal deployment path):

```sh
HOST_TS_IP=$(tailscale ip -4)
curl -X POST "http://$CHIP/api/pstop_peer?ip=$HOST_TS_IP&port=8890"
```

The change persists to NVS and the comparator picks it up within one
tick (~100 ms), no reboot. For a Tailscale peer the chip source-binds
its socket to the VPN IP — if the tunnel is down it holds the bond and
the machine safely sees silence.

## What you should see

```
machine_app_runner listening on 0.0.0.0:8890
Robot Status = STOP
pstop 0x01D7791C -> BOND
  STATE robot=STOPPED restart=NEED_STOP owner=0x00000000 active=1  (from pstop 0x01D7791C BOND; ...)
pstop 0x01D7791C -> STOP
ARMED by 0x01D7791C: STOP held 804 ms (policy minimum 500 ms)
Robot Status = OK
pstop 0x01D778B4 -> STOP
  STATE robot=STOPPED restart=NEED_STOP owner=0x01D7791C active=2  (from pstop 0x01D778B4 STOP; was robot=OK ...)
```

The machine starts disarmed (NEED_STOP). Arm it: press and hold the
E-stop switch ≥ `min_stop_ms` (500 ms), then release →
`ARMED by 0x01020380`, `Robot Status = OK`, remote's ring turns green.

On the remote, `/state.json` should show `pstop_sent` advancing at 10 Hz
with `pstop_mismatch` flat:

```sh
curl -s http://$CHIP/state.json | jq '.pstop_sent, .pstop_replies, .pstop_mismatch'
```

If `pstop_mismatch` climbs, the two cores are producing different
encodings — see `firmware/main/main.c::compute_verdict` and the loop
diagnostics (`e_hi*/e_lo*`) in `/state.json`.

## Testing the policy without a chip

`tools/pstop_test_remote.py` bonds over the real wire protocol and runs
timed STOP/OK sequences against a dedicated runner instance — see
`docs/TESTING.md`.

## Stopping

`Ctrl-C`. SIGINT/SIGTERM are handled cleanly and the socket is closed.
