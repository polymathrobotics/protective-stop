# pstop 10 Hz heartbeat client

This is the safety-payload running on the chip. It produces a 40-byte
pstop_msg OK at exactly 10 Hz, sends it via UDP to a runtime-configurable
destination, and parses replies (also 40-byte pstop_msg) to track RTT and
liveness.

## Design

`pstop_client_task` in `examples/dual_core_safety/main/main.c`:

- Plain `socket(AF_INET, SOCK_DGRAM, 0)` — **not** `microlink_udp_send()`.
  This was a deliberate choice; see "Why plain BSD UDP, not microlink_udp"
  below.
- `setsockopt SO_RCVTIMEO 50 ms` so the recv doesn't block the 10 Hz tick.
- Builds pstop_msg with fields: `version=1`, `device_id=0xCAFEBABE` (chip),
  `seq` incrementing, `state=OK`, `timestamp_ms`, `padding[24]` zero.
- CRC-16-CCITT (poly `0x1021`, init `0xFFFF`) over bytes [0..37], stored
  little-endian at [38..39].
- After each send: `recvfrom` (50 ms timeout), validate CRC of reply,
  compute RTT from `timestamp_ms` round-trip.
- Atomics update `pstop_sent`, `pstop_replies`, `pstop_send_fail`,
  `pstop_rtt_ms` for `/state.json` exposure.
- Sleeps `100 ms - elapsed` to maintain exact 10 Hz cadence.

## Configuration

Peer IP and port are persisted in NVS (`USR_NVS_NS "dcs_app"`, keys
`pstop_ip` and `pstop_port`). Defaults to `10.42.0.1:8890` (host over
USB-NCM).

To repoint at runtime, no reboot needed:

```sh
curl -u admin:microlink -X POST \
    "http://10.42.0.80/api/pstop_peer?ip=100.97.180.50&port=8890"
```

Response includes the new values; task picks them up at next tick.

## Telemetry in /state.json

Fields owned by the pstop task:

| key | meaning |
|---|---|
| `pstop_peer_ip` | current dest IP |
| `pstop_peer_port` | current dest port |
| `pstop_sent` | total sends attempted |
| `pstop_replies` | total valid replies received |
| `pstop_send_fail` | `sendto` errors (typically `EAGAIN`/`ENETUNREACH`) |
| `pstop_rtt_ms` | last reply RTT |

Healthy state: `pstop_sent ≈ pstop_replies`, both grow at 10/s, `fail=0`.

`fail` growing while `sent` stays flat usually means the destination is
unreachable on the routing table — check `pstop_peer_ip` is on a netif
that's up. `sent` growing without matching `replies` means responder is
down or firewalled.

## Bench-verified results

Over USB-NCM (`10.42.0.1:8890`), 60 s steady-state:
- `pstop_sent`: 600
- `pstop_replies`: 599
- `pstop_send_fail`: 0
- mean RTT: ~3 ms

99.8% reply rate. The one dropped reply was at boot before the responder
binding settled.

Over Tailscale (chip→VPN→host_tailscaled→responder), the chip *can*
deliver heartbeats during the alive window but the microlink wedge
takes everything down within 60–90 s. See `MICROLINK_WEDGE.md`.

## Why plain BSD UDP, not microlink_udp_send

We initially tried `microlink_udp_send()` which routes through the WG
tunnel. Two problems:

1. With `disco/stun = false` (which we need, because turning them on
   broke registration), the chip can't complete a `CallMeMaybe`
   round-trip, so the peer table never gets a usable endpoint for the
   robot. `microlink_udp_send` returns failure for every peer not yet
   handshake-complete.
2. We bumped `max_peers` from 2 to 16, but that just exposes more peers
   waiting on disco; doesn't unblock the path to the one peer we care
   about.

The plain BSD socket path uses lwIP's routing table directly. lwIP picks
the netif by destination IP:
- `10.42.0.1` → USB-NCM netif (the host) → host can forward to robot via
  its full Tailscale daemon. This works.
- `100.97.180.X` → WG netif → microlink layer → handshake → robot. This
  is what we want long-term; blocked by the wedge.

Plain BSD UDP also gives us the `pstop_send_fail` counter via `errno`,
which we couldn't easily get from `microlink_udp_send`.

The path-choice trick: change `pstop_peer_ip`, lwIP does the right thing
automatically. No code change to switch between USB-NCM and Tailscale
routing.

## Recommended deployment topology

While the chip-side wedge is unresolved, the recommended topology for
real-world safety use is:

```
robot ─[Tailscale]→ host PC ─[USB-NCM]→ chip
                       ↓
                       runs full Linux tailscaled
                       runs a small forwarder script
                          (rx on TS interface, tx on usb0 to 10.42.0.80)
```

This pushes the Tailscale stack onto Linux where it's been battle-tested.
The chip only needs USB-NCM, which has been rock-solid in soak tests.
The forwarder is ~30 lines of Python.

Once the microlink wedge is fixed (see `MICROLINK_WEDGE.md`) we can drop
the host PC from the topology and have the chip do Tailscale directly.

## Test responder

See `pstop_responder.py` in this docs/ dir. Run it on the host (or on
the robot itself) to echo pstop messages back:

```sh
python3 examples/dual_core_safety/docs/pstop_responder.py \
    --bind 0.0.0.0 --port 8890
```

It logs each received message (seq, state, RTT-as-seen-from-here) and
echoes a pstop_msg OK back to the sender with the same `seq` and a
`device_id=0x01020304` so the chip can tell its own packets from the
reply.
