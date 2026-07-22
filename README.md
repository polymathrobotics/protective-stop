# pstop — networked protective stop

A protective-stop (pstop) system: a battery-powerable ESP32-S3 **remote**
heartbeats operator intent at 10 Hz over an encrypted tunnel to a
**machine** process on the robot; if the heartbeats say STOP — or stop
arriving at all — the machine stops the robot. The wire protocol and the
machine-side state logic are the certification-track
[`pstop_c`](https://github.com/polymathrobotics/protective-stop) library
(pinned submodule, never modified); everything in this repo is the
transport, sensing, and supervision shell around it.

The remote (Waveshare ESP32-S3-ETH, W5500 PoE Ethernet) senses a physical
DPST normally-closed E-stop switch through two independent loopback
channels — one per CPU core. The two cores run in lockstep: each reads
its own loop, builds the full 40-byte pstop message independently, and a
comparator transmits only when the two encodings are byte-identical
(2oo2-to-run, 1oo2-to-stop). Any disagreement, loop fault, or task stall
sends nothing, and the machine's native heartbeat timeout stops the
robot: fail-safe by silence. Uplinks fail over Ethernet → USB-NCM →
WiFi automatically; the pstop traffic rides Tailscale/WireGuard via the
vendored microlink library, with egress pinned to the tunnel so a downed
VPN fails safe, never falls back to plaintext.

The machine side (`host/machine_app_runner`) wraps the unmodified pstop_c
machine library with transport, config (`machine.toml`), logging, and a
wrapper-owned arming policy: a STOP episode must be held ≥ `min_stop_ms`
(default 500 ms) for its release to arm the robot, so an EMC blip can
never perform the arming gesture. All wrapper policies are documented in
`docs/FAILOVER_AND_ARMING_DESIGN_2026-07-21.md`.

## Architecture

```
            REMOTE (ESP32-S3, firmware/)                        MACHINE (robot host, host/)
 ┌────────────────────────────────────────────────┐    ┌──────────────────────────────────────┐
 │  DPST E-stop switch (physical part)            │    │  machine_app_runner (wrapper)        │
 │   pole1: GPIO39 ─▶ loop A ─▶ GPIO40  core 0    │    │   ├─ machine.toml config             │
 │   pole2: GPIO41 ─▶ loop B ─▶ GPIO42  core 1    │    │   ├─ min_stop_ms arming policy       │
 │        │verdict+encode      │verdict+encode    │    │   ├─ status latch (actuation hook)   │
 │        ▼                    ▼                  │    │   └─ anomaly logging                 │
 │   ┌─────────────────────────────┐              │    │  pstop_c machine library (submodule, │
 │   │ comparator: memcmp 40-byte  │  10 Hz UDP   │    │  certification track, UNMODIFIED):   │
 │   │ encodings, tx only on match ├──────────────┼────┼─▶ bond/counters/CRC/heartbeat        │
 │   └─────────────────────────────┘  pstop msgs  │    │   timeout → robot STOP               │
 │  WS2812 ring (GPIO17): link state              │    └──────────────────────────────────────┘
 │  dcs_support: admin UI, OTA+rollback chain,    │            ▲
 │  net supervisor (Eth>USB>WiFi), liveness WDT   │            │ Tailscale / WireGuard
 │  microlink: Tailscale/WG, DERP fallback ───────┼────────────┘ (DERP relay fallback ≤10 s)
 └────────────────────────────────────────────────┘
```

## Repo layout

| Path | What |
|---|---|
| `firmware/` | ESP-IDF 5.5 project (`pstop_remote`): `main/main.c` is the auditable lockstep safety core; `components/dcs_support/` is everything else (boot, OTA chain, admin UI, failover, telemetry) |
| `components/pstop/` | `upstream/` = pstop_c submodule @ `d9f4970` (certification track, never modified) + ESP port glue |
If certification scoping later requires the arming-duration policy
inside the library, the specified change and its risk register are in
[`docs/PSTOP_C_MIN_STOP_OPTION.md`](docs/PSTOP_C_MIN_STOP_OPTION.md).
| `components/microlink/` | vendored Tailscale/WireGuard/DERP transport library |
| `components/ml_dev_tether/` | USB-CDC-NCM tether (TinyUSB) |
| `host/` | `machine_app_runner.c` + `machine.toml` — the robot-side machine wrapper |
| `tools/` | Python test tools: scripted wire-protocol remote, chaos proxy, soak harnesses |
| `test/` | bench test scripts (chaos/netem ladders, soak loops) |
| `hardware/` | enclosure CAD (STL/STEP/3MF), board schematic |
| `docs/` | current design docs, reports, playbooks; `docs/archive/` = historical records |

## Quickstart

### Build + first flash (remote)

```sh
cd firmware
source ~/esp-idf-5.5/export.sh
idf.py build
```

First flash must be over the wire: hold **BOOT**, tap **RST** to enter
ROM download mode (when the USB tether is enabled, TinyUSB owns USB-OTG,
so the ROM USB-JTAG is not visible otherwise), then:

```sh
idf.py -p /dev/ttyACM0 flash
```

Every subsequent update goes over the network (either interface):

```sh
curl -u admin:microlink --data-binary @build/pstop_remote.bin \
     http://$CHIP/admin/api/ota
```

**Backup recovery (fleet bypass):** this same direct upload is the way to
recover a unit that the fleet can't reach or update — e.g. one wedged with a
bad DERP home so its fleet check-in/OTA fails. As long as you can reach the
unit's admin server directly (LAN, USB tether `10.42.0.1`, or its Tailscale IP
from a same-region/direct peer), push a known-good `pstop_remote.bin` to
`http://$CHIP/admin/api/ota`; it flashes, reboots, and self-heals. Verified
2026-07-22 recovering a unit stuck advertising the wrong DERP region.

Copy `sdkconfig.credentials.example` → `sdkconfig.credentials` for WiFi
and Tailscale credentials before the first build.

### Machine (robot host)

```sh
cd host
make                                  # needs only cc + make
./machine_app_runner machine.toml     # listens on 0.0.0.0:8890
```

Point the remote at it (persisted to NVS, applied within one 100 ms tick):

```sh
curl -X POST "http://$CHIP/api/pstop_peer?ip=<machine-ip>&port=8890"
```

Arming: press and hold the E-stop switch ≥0.5 s, release — the runner
logs `ARMED`, the ring turns green. See `host/README.md`.

## Key runtime facts

- **10 Hz** heartbeat; machine stop-on-silence =
  `heartbeat_ms × (max_missed_heartbeats+1)` ≈ 1–2 s as configured.
- **Arming policy:** STOP held < `min_stop_ms` (500 ms) is vetoed by the
  machine wrapper; the chip additionally debounces loop re-close by
  3 ticks and holds all transmission until the loops settle at boot.
- **Fail-safe silence:** lockstep mismatch, loop fault, VPN-down
  (source-bound socket), or comparator stall all result in *no message*,
  which the machine treats as STOP.
- **Interfaces:** Ethernet > USB-NCM > WiFi, supervised at 1 Hz with
  automatic default-route failover. Bench IPs are environment-specific —
  nothing in this repo assumes a particular LAN.
- **Status ring:** yellow = no machine, blue = bonded, green = armed,
  red = STOP, purple = lockstep mismatch (see `docs/TROUBLESHOOTING.md`).
- **Recovery chain:** TWDT → network-liveness watchdog → crash counter →
  OTA rollback, all button-free (`docs/SAFETY_CHAIN.md`,
  `docs/RECOVERY_PLAYBOOK.md`).
- **HTTP API:** diagnostic/config (port 80) + password-protected `/admin/*`
  routes — full reference in [`docs/API.md`](docs/API.md).

## Testing

See `docs/TESTING.md` for the full harness: `tools/pstop_test_remote.py`
(arming-policy suite over the real wire protocol),
`tools/pstop_chaos_proxy.py` + `test/chaos_ladder.sh` (protocol-level
impairment ladder), `test/netem_ladder.sh` (WG-underlay chaos), and the
soak methodology with measured baselines. The simulated-press endpoint
(`pstop_sim`) was removed from production firmware — tests exercise the
machine policy via `tools/pstop_test_remote.py` instead, and arming a
real unit requires a physical press.

Static analysis: `./tools/misra_check.sh` runs a free-cppcheck MISRA
C:2012 pass over the code we own (the certified `pstop_c` is excluded).
See `docs/MISRA_COMPLIANCE_2026-07-21.md` for results and the deviation
register.

## Current status + open items (2026-07-21)

Validated across all three transports (soaks + impairment ladders, zero
false stops, zero device crashes — `docs/TRANSPORT_TEST_REPORT_2026-07-20.md`,
`docs/CHAOS_RESULTS_2026-07-20.md`). DERP failover ≤10 s and the arming
policy are bench-verified (`docs/FAILOVER_AND_ARMING_DESIGN_2026-07-21.md`).
MISRA C sweep of the owned firmware done (`docs/MISRA_COMPLIANCE_2026-07-21.md`).

Open items:
- **WiFi is the weakest transport** — ~98 % soak reply rate with a
  fail-safe stop/self-heal burst every 10–15 min. Deployment policy:
  Ethernet primary, WiFi fallback, USB-NCM for bench/service.
- Retune machine `max_lost_messages` (10 → ≈20) so the heartbeat
  timeout, not the counter gap, is the stop authority on WiFi.
- **Tailscale subnet-route caveat:** a subnet router advertising the
  robot's LAN hijacks operator-laptop traffic to the chip's LAN IP;
  needs an `ip rule` workaround — document for the fleet.
- Long-duration (24 h+) per-transport soaks before certification runs.
- Re-run the WiFi soak to prove zero unscheduled arms under the new
  debounce + arming policy.

## Certification note

`components/pstop/upstream` (pstop_c) is pinned at `d9f4970` and is
**never modified** — it is on its own certification track. Every policy
this repo adds (arming veto, status latch, transport binding, debounce)
lives in the wrapper or the firmware shell, uses only public library
API, and is designed so a wrapper bug can cost availability but never
cause a spurious arm. Wrapper-owned policies and the option of moving
them into the library are documented in
`docs/FAILOVER_AND_ARMING_DESIGN_2026-07-21.md`; the SIL2 design record
is `docs/PSTOP_SAFETY_DESIGN.md`. Deployment rule: a pstop_c bump that
changes the CRC is a wire break — update chip and machine together.
