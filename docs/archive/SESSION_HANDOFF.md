> ⚠️ **Historical (2026-05-25 handoff).** The wedge described here as
> "likely fixed in v15.1" was only partially mitigated. The full root
> cause (task-stack overflows in `bc_clear` at 2 min and IDF `ping` at
> 33 min, plus an lwIP cross-thread-safety bug) was caught and fixed
> in v15.20–v15.26. **Current canonical doc:**
> [`TAILSCALE_FINAL_2026-05-26.md`](TAILSCALE_FINAL_2026-05-26.md).
> **Troubleshooting:** [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md). Kept
> below for archaeological context.

# Session handoff — dual_core_safety with pstop heartbeat

Last touched 2026-05-25. Read this first; the other docs in this folder go
deeper on specific topics.

> **Update 2026-05-25 (evening session):** the lwIP wedge described
> below is **likely fixed** — see `SESSION_REPORT_2026-05-25.md` for
> the root cause (dead `enable_disco` flag) and the v15.1 bench results
> (2+ min soak, pstop 10 Hz at 99.9 %, `wg_pbuf_fails=0`). The "What's
> broken / not done" section below is stale; pick up from
> SESSION_REPORT_2026-05-25.md for current state.

## Where things stand

The chip is a Seeed XIAO ESP32-S3 wired to a host PC over USB-C. On the host
side it shows up as a USB-CDC-NCM network device at `enx<mac>` with the host
at `10.42.0.1` and the chip at `10.42.0.80`. The chip also brings up
Tailscale via the microlink library on a separate WireGuard netif
(VPN IP `100.97.180.43`).

What works right now, deployed on the chip:

- USB-NCM tether comes up reliably; admin UI at `http://10.42.0.80/`,
  password `admin:microlink`.
- Tailscale registers and connects briefly, but the chip's lwIP silently
  wedges 60–90 s into sustained Tailscale activity. See `MICROLINK_WEDGE.md`.
- A 4-layer safety chain catches every wedge and rolls back automatically:
  liveness watchdog → `boot_count` → bootloader rollback. See
  `SAFETY_CHAIN.md`.
- pstop client task sends 40-byte pstop_msg OK heartbeats at exactly 10 Hz
  via a runtime-configurable destination. Bench-verified at 99.8% reply
  success over `10.42.0.1` (USB-NCM). See `PSTOP_HEARTBEAT.md`.

What's broken / not done:

- Direct chip-over-Tailscale pstop heartbeats are not yet reliable enough
  for safety-critical use. The microlink wedge gives ~60 s alive windows
  with ~45 s outages.
- We never ran the upstream `microlink-polymath` v2.1.0 fix set (the user
  has it at `~/microlink-polymath/`); we're on an older internal fork that
  has *some* of those fixes but not all. See `MICROLINK_WEDGE.md` §
  "Three small fixes from upstream we haven't pulled".

## Next steps, ranked by expected impact and effort

These are the concrete things to try in the next session, in order. Each
one is small and reversible (the safety chain catches every regression).

### 1. Bump the lwIP pbuf pool — the single highest-EV fix

Edit `examples/dual_core_safety/sdkconfig.defaults`. Add these lines next
to the existing `# lwIP` block:

```
CONFIG_LWIP_PBUF_POOL_SIZE=64
CONFIG_LWIP_IRAM_OPTIMIZATION=y
```

Default `CONFIG_LWIP_PBUF_POOL_SIZE` is 16. With three netifs active
(USB-NCM + WiFi STA shell + WireGuard) and 16 Tailscale peers' worth of
DISCO/keepalive churn, 16 entries is the most likely thing the chip is
running out of. See `MICROLINK_WEDGE.md` for the full hypothesis.

Build, OTA, then run a 5-min uptime test (see `RECOVERY_PLAYBOOK.md`).
Heap, ticks, and `pstop_sent`/`pstop_replies` should all advance
monotonically with no reboot.

### 2. Add a pbuf-failure log to wireguardif

In `components/microlink/components/wireguard_lwip/src/wireguardif.c`, find
every `pbuf_alloc` call and add an `ESP_LOGW` on NULL return so we'd see
exhaustion in UART instead of having to guess. Small surgical patch, also
catches the symptom if hypothesis 1 is wrong.

### 3. Look at the upstream v2.1.0 microlink at `~/microlink-polymath`

That fork has commits we haven't pulled, specifically:
- `09c387e v2.1.0 — TCP API, dual-path WG handshake, EarlyNoise slow-path fix`
- `502d0b1 Fix coord_recv and noise_recv partial read bug for large tailnets` (this one IS in our tree)
- `f4558d4 MapResponse 60s timeout fix` (also in our tree)
- `1c49194 Fix ml_socket to route AF_INET6 through AT socket bridge` (may not apply)

The `09c387e` set adds dual-path WG handshake (sends INIT via DERP +
direct UDP simultaneously). That should make the disco/stun=false WG
handshake setup work without the DISCO `CallMeMaybe` round-trip we
currently can't complete. Worth porting in carefully if step 1 doesn't
fully fix things.

### 4. If wedge still happens after step 1, cap concurrent DISCO probes

In `components/microlink/src/ml_wg_mgr.c`, find `disco_periodic_probes()`
and rate-limit to ≤ 2 in-flight probes regardless of peer count. The
chip's TCPIP thread is the bottleneck — blasting 16 peers at once
saturates it even with a bigger pool.

### 5. Once the wedge is gone, point pstop at the Tailscale IP directly

`curl -u admin:microlink -X POST "http://10.42.0.80/api/pstop_peer?ip=100.A.B.C&port=8890"`

(replace with robot's Tailscale IP — `100.A.B.C` is whatever
`tailscale status` shows for the robot). Persisted to NVS, takes effect
without reboot. Then run a multi-hour soak with the pstop responder on
the robot — `pstop_replies` and `pstop_sent` should grow together at
exactly 10 Hz, `pstop_send_fail` should stay at 0.

## How to deploy a new build (no buttons)

The chip is in production firmware mode (USB-OTG owned by TinyUSB), so
the ROM USB-JTAG isn't visible to host. **OTA is the only reach.**

Two-step deploy that's been working reliably:

```sh
# 1. Pause Tailscale so the OTA upload isn't competing with peer traffic.
curl -m 5 -X POST http://10.42.0.80/api/derp
curl -m 5 -X POST http://10.42.0.80/api/wg

# 2. Upload the new binary. With Tailscale paused the upload finishes
#    cleanly in ~15 s; without the pause, lwIP starvation aborts it.
curl -m 300 -u admin:microlink \
     -H "Content-Type: application/octet-stream" \
     --data-binary "@build/microlink_dual_core_safety.bin" \
     http://10.42.0.80/admin/api/ota
```

Returns `{"ok":true,"message":"Firmware updated. Rebooting..."}` on
success. Chip auto-reboots into the new image.

If the chip is mid-wedge when you try, the OTA POST returns `HTTP 000`
(timeout). Wait for the safety chain to fire and try again — see
`RECOVERY_PLAYBOOK.md`.

## Bench setup that gets you to a working state in <5 min

```sh
# On the host:
cd ~/Nextcloud/PROWORK/Polymath\ Robotics/Claude/microlink-polymath-main
git log --oneline -8                       # confirm you have the commits

# Build (IDF 5.5 required — see reference-idf55-pstop-pollh memory):
cd examples/dual_core_safety
source ~/esp-idf-5.5/export.sh
idf.py build

# Start the pstop responder (host-side simulator of the robot):
python3 docs/pstop_responder.py --bind 0.0.0.0 --port 8890 &

# OTA the build (chip must already have a microlink firmware on it):
# … see "How to deploy a new build" above …

# Verify pstop is flowing:
curl -s http://10.42.0.80/state.json | jq '.pstop_sent, .pstop_replies, .pstop_send_fail'
```

Inside the chip's first 50 s of uptime you should see `pstop_sent` and
`pstop_replies` both climbing in lockstep at 10/s.

## Where the artifacts live

- Source root: this repo, `examples/dual_core_safety/`
- Built bin: `examples/dual_core_safety/build/microlink_dual_core_safety.bin`
- Build elf for addr2line: `examples/dual_core_safety/build/microlink_dual_core_safety.elf`
- Test responder: `examples/dual_core_safety/docs/pstop_responder.py`
- Last-known-good binary on the bench: was at `/tmp/v22.bin` on previous
  host — not in git, rebuild from source.

## Commits to be aware of (this session's work)

```
8f2f57e dual_core_safety: pstop 10Hz heartbeat client over UDP
bf499bd Tailscale: fix WiFi-scan-storm + DERP starvation under load
8bb88e9 ml_derp: verbose [TIMING-DERP] logs to localise the HTTP-upgrade hang
0471d8c ml_derp + dual_core_safety: leak fix + network liveness watchdog
e2a7d4d dual_core_safety: README — document USB toggle, safety design, …
bb49f0d dual_core_safety: USB toggle, safety-grade OTA rollback, IDF 5.5
```

…plus a `components/pstop/upstream` submodule bump for the IDF 5.5
`<sys/poll.h>` fix from yesterday.
