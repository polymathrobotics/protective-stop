# Protective Stop 🛑🛡

A low-cost, reliable **protective stop** for robots and automation: a
battery-powerable ESP32-S3 **remote** heartbeats operator intent at 10 Hz
over an encrypted tunnel to a **machine** process on the robot. If the
heartbeats say STOP — or stop arriving at all — the machine brings the
robot to a controlled stop.

**Safety scope.** This is a *protective* stop (a controlled, preventative
shutdown), not an emergency stop. It is not a substitute for an E-stop
where there is an immediate threat to people or equipment.

The wire protocol and the machine-side safety state logic are the
[`pstop_c`](pstop_c/) library, kept on its own certification track and
**never modified elsewhere in this repo**. Everything around it —
firmware, transport, sensing, supervision, and the machine wrapper — is
the open engineering shell that turns `pstop_c` into a deployable device.

## How it works

The remote (Waveshare ESP32-S3-ETH) senses a physical DPST
normally-closed E-stop switch through two independent loopback channels,
one per CPU core. The two cores run in **lockstep**: each reads its own
loop, independently builds the 40-byte pstop message, and a comparator
transmits only when the two encodings are byte-identical (2-out-of-2 to
run, 1-out-of-2 to stop). Any disagreement, loop fault, or stalled task
sends nothing — and the machine's heartbeat timeout stops the robot.
**Fail-safe by silence** is the core principle: every failure mode
degrades to "no message," which is a stop.

pstop traffic rides Tailscale/WireGuard with automatic uplink failover
(Ethernet → USB-NCM → WiFi) and DERP relay fallback; egress is pinned to
the tunnel so a downed VPN fails safe rather than leaking plaintext. The
remote establishes the link itself — it homes its DERP connection on the
machine's region and initiates the handshake, so no manual `tailscale ping`
from the robot side is ever needed, and it re-establishes on its own after
either end reboots. The machine wrapper adds a minimum-hold arming policy
so an electrical blip can never perform the arming gesture.

```mermaid
flowchart LR
    subgraph REMOTE["REMOTE — ESP32-S3 · firmware/"]
        direction TB
        SW["DPST E-stop switch<br/>pole1 GPIO39▸loopA▸GPIO40<br/>pole2 GPIO41▸loopB▸GPIO42"]
        C0["core 0<br/>read loopA · build + encode"]
        C1["core 1<br/>read loopB · build + encode"]
        CMP{"comparator<br/>memcmp — transmit<br/>only on byte-match"}
        SW --> C0 --> CMP
        SW --> C1 --> CMP
    end

    NET(["Tailscale / WireGuard<br/>direct path · DERP relay fallback<br/>egress pinned to tunnel"])

    subgraph MACHINE["MACHINE — robot host · host/"]
        direction TB
        WRAP["machine_app_runner<br/>machine.toml · min-hold arming<br/>status latch + anomaly log"]
        LIB["pstop_c machine lib<br/>certified · UNMODIFIED<br/>bond · counters · CRC · timeout"]
        STOP["heartbeat says STOP<br/>— or stops arriving —<br/>▶ robot stops"]
        WRAP --> LIB --> STOP
    end

    CMP -->|"10 Hz pstop over UDP"| NET --> WRAP
```

Every failure mode — loop fault, core disagreement, stalled task, dropped
link — degrades to *no message*, and no message is a stop. **Fail-safe by
silence.**

## Repo layout

| Path | What |
|---|---|
| [`pstop_c/`](pstop_c/) | Certified protocol + machine-safety C library. Its own track — **do not modify here**; contribute upstream. |
| `firmware/` | ESP-IDF 5.5 remote firmware (`pstop_remote`). `main/main.c` is the auditable lockstep core; `components/dcs_support/` is the shell (boot, OTA+rollback, admin UI, failover, telemetry). |
| `components/` | `microlink/` (Tailscale/WireGuard/DERP), `ml_dev_tether/` (USB-CDC-NCM), `pstop/` (ESP-IDF glue that compiles `pstop_c/`). |
| `host/` | `machine_app_runner` + `machine.toml` — the robot-side machine wrapper. |
| `tools/` · `test/` | Wire-accurate test remote, chaos proxy, MISRA check; bench chaos/soak/netem ladders. |
| `docs/` | Design records, test reports, safety/recovery playbooks (`docs/archive/` = historical). |
| `hardware/` | Enclosure CAD + board schematic (work in progress — see `hardware/README.md`). |
| `archive/` | Deprecated ROS 2 / Foxglove packages, kept for reference pending removal. |

## Quickstart

**Remote firmware**

```sh
cp firmware/sdkconfig.credentials.example firmware/sdkconfig.credentials   # WiFi + Tailscale creds
cd firmware && source ~/esp-idf-5.5/export.sh && idf.py build
```

First flash is over the wire (hold **BOOT**, tap **RST** for ROM download
mode, since TinyUSB owns USB-OTG at runtime):

```sh
idf.py -p /dev/ttyACM0 flash
```

Every update after that is over the network:

```sh
curl -u admin:<pw> --data-binary @build/pstop_remote.bin http://<chip>/admin/api/ota
```

**Machine (robot host)** — needs only `cc` + `make`, no ESP-IDF:

```sh
cd host && make && ./machine_app_runner machine.toml     # listens on 0.0.0.0:8890
curl -X POST "http://<chip>/api/pstop_peer?ip=<machine-ip>&port=8890"
```

Arm by pressing and holding the switch ≥0.5 s, then releasing.

## Key facts

- **10 Hz** heartbeat; stop-on-silence in ≈1–2 s (configurable).
- **Arming policy:** a STOP shorter than `min_stop_ms` (default 500 ms) is
  vetoed machine-side; the remote also debounces loop re-close and holds
  transmission until the loops settle at boot — an EMC blip can't arm.
- **Interfaces:** Ethernet > USB-NCM > WiFi, 1 Hz supervised failover.
  128-peer Tailscale capacity, priority-peer latency isolation.
- **Identity:** each remote derives a stable per-unit ID from its MAC —
  Tailscale node `pstop-01xxxxxx` matches its pstop device ID `0x01xxxxxx`;
  the node record and VPN IP persist across reboot/OTA.
- **Recovery:** task watchdog → network-liveness watchdog → crash counter
  → OTA rollback, all button-free. Transport self-heals a lost link with
  no operator action.
- **Status ring:** yellow = no machine · blue = bonded · green = armed ·
  red = STOP · purple = lockstep mismatch.

## Testing & static analysis

`docs/TESTING.md` documents the full harness. Highlights:
`tools/pstop_test_remote.py` (arming-policy suite over the real wire
protocol), `tools/pstop_chaos_proxy.py` + `test/chaos_ladder.sh`
(loss/delay/dup/corrupt ladder), `test/netem_ladder.sh` (WireGuard-underlay
chaos), and `./tools/misra_check.sh` (free-cppcheck MISRA C:2012 over the
code we own; `pstop_c` excluded — see `docs/MISRA_COMPLIANCE_2026-07-21.md`).

Validated across USB / Ethernet / WiFi (soaks + impairment ladders, zero
false stops, zero crashes), on a live 128-peer tailnet, and on a live
two-site rig against a geographically remote machine. In that run a full
blackhole of the direct WireGuard path was carried by DERP with **zero
gap and zero rebonds**, any single-path failure left the heartbeat intact,
and a total link loss degraded to STOP with **~3–6 s autonomous recovery**.
See `docs/TWO_SITE_FAILOVER_2026-07-21.md`,
`docs/TRANSPORT_TEST_REPORT_2026-07-20.md`,
`docs/CHAOS_RESULTS_2026-07-20.md`, and
`docs/FAILOVER_AND_ARMING_DESIGN_2026-07-21.md`.

## Certification & licensing

`pstop_c/` is on its own certification track and is never modified from
this repo. Every policy the shell adds (arming veto, status latch,
transport binding, loop debounce) uses only public library API and is
designed so a shell bug can cost availability but never cause a spurious
arm. The SIL2 design record is `docs/PSTOP_SAFETY_DESIGN.md`; the option
of moving the arming policy into the library is analyzed in
`docs/PSTOP_C_MIN_STOP_OPTION.md`.

Software is Apache-2.0 (see [`LICENSE`](LICENSE)). Open-hardware
certification progress and the intended hardware/docs license split are
tracked in `docs/OSHWA_COMPLIANCE.md`.

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md). In short: CI must be green,
`pre-commit` clean, and changes to protocol/safety behavior belong in
`pstop_c` upstream, not here. Security reports: [`SECURITY.md`](SECURITY.md).
