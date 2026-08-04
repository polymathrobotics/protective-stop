<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Protective-Stop — System Definition & Function Decomposition

**Status:** authoritative backbone for the safety analysis. HARA, FMEA,
safety requirements, and the traceability/coverage reports all reference the
function IDs defined here. Keep IDs **stable** — downstream tables cite them.

**Standard frame:** IEC 61508 (SIL, systematic capability) **+** ISO 13849
(Performance Level for the actuation/machinery side). Target integrity:
**SIL 3 / PL e** for the on-demand stop function (see HARA for the derivation;
this is the design target, not yet a claim).

**Coverage regime:** structural coverage to the SIL 3 bar (statement + branch,
with MC/DC on the safety decision logic) **and** requirements-driven coverage
(every safety-relevant line traced to ≥1 requirement; every requirement verified
by ≥1 test).

> **Reconciliation note (doc vs code, 2026-08).** `PSTOP_SAFETY_DESIGN.md` is
> SIL2-framed and predates the current firmware. Two of its "proposed" measures
> are now **implemented** and one is **dropped** — the analysis below reflects
> the *code*, not that doc:
> - **Option A (implemented):** the OK verdict is derived from a *fresh
>   both-phase* physical sample every tick (not the old rolling `counter&1`).
> - **Option B (implemented):** the two cores form the verdict by *diverse
>   expressions* of the same reads (arithmetic-image vs boolean), so a
>   systematic interpretation bug on one core diverges the comparator.
> - **Option C (dropped, 2026-07):** the LFSR/PRNG challenge-response and the
>   stamp echo-signature (§5.1/§5.3 of the design doc) were assessed and
>   **not** adopted — Option A's both-phase already covers stuck-at, and a
>   deterministic challenge is reproducible by a same-firmware fault. Recorded
>   as a residual, not a planned control.

---

## 1. System boundary

The **protective-stop (pstop) system** spans a wireless-linked remote and a
machine controller. `pstop_c` (the certified protocol library) is a
**pre-qualified component (SEooC)** — referenced at its interface, never
modified, and out of scope for re-verification here.

```
  ┌─────────────────────────┐         black channel          ┌──────────────────────────┐
  │  REMOTE (ESP32-S3)       │   Tailscale / WireGuard over    │  MACHINE controller       │
  │  dual-core lockstep      │   WiFi / Ethernet / USB-NCM,     │  (one of two backends)    │
  │  DPST button on 2 loops  │═══ DERP relay fallback ═════════▶│  fail-safe relay output   │
  │  firmware/ (in scope)    │◀══ counter/stamp echo ══════════│  host/ or ros2/ (in scope)│
  └─────────────────────────┘                                  └──────────────────────────┘
        both ends link the pre-qualified pstop_c library (out of scope, referenced at interface)
```

**In scope (this effort):** remote firmware `firmware/`; host machine
`host/machine_app_runner.c`; ROS2 machine `ros2/protective_stop_machine/`.
**Referenced boundary (pre-qualified):** `pstop_c/` — has its own
requirement-traced tests + coverage + CI; we add only *interface/integration*
requirements at its edges.

## 2. Operating context

| Aspect | Value |
|---|---|
| Application | Mobile autonomous off-highway / industrial vehicle |
| Operator interface | Handheld or fixed remote with a DPST normally-closed stop button |
| Link | Tailscale/WireGuard mesh over WiFi / Ethernet / USB-NCM; DERP relay when no direct path |
| Safe state | **STOP = de-energized** (no valid message ⇒ machine heartbeat-times-out to STOP) |
| Architecture | Remote: 1oo2D to stop / 2oo2 to run, HFT=1. Machine relay: de-energize-to-safe |
| Tick rate | 10 Hz sensing/compare on the remote |
| Heartbeat | `heartbeat_ms` = 400 ms default; timeout = `heartbeat_ms × max_missed` (=5) ≈ **2.0 s** |
| Arming | STOP must be held ≥ `min_stop_ms` = 500 ms, then released, to count as the deliberate arming gesture |
| Topology | Many remotes → one machine (fail-safe OR); one remote → many machines |

> **Heartbeat-timeout tunable (2026-08-04).** `max_missed_heartbeats` was raised
> **3 → 5** (machn firmware, advertised to remotes), so the silent-remote
> STOP-detection timeout moved from `400 × 3 ≈ 1.2 s` to `400 × 5 ≈ 2.0 s`.
> Rationale: ~90-min Tailscale control-plane re-syncs caused transient ~1.4 s RTT
> spikes on the machine bond that exceeded the old 1.2 s timeout → nuisance
> rebonds / false STOPs; the 2.0 s margin rides through them. This is a
> deliberate trade of real-disconnect detection latency (1.2 s → 2.0 s) for far
> fewer false safety stops. `max_missed` is a runtime tunable inside the
> validated envelope `[1,5]` (SR-H-03); the new default sits at the envelope
> **ceiling**. Any FTTI / process-safety-time budget must now accommodate the
> ~2.0 s detection time (+ one 100 ms tick ⇒ ≈ 2.1 s internal budget — HARA A-05,
> SR-SYS-01).

## 3. Top-level safety functions

| ID | Safety function | Statement |
|---|---|---|
| **SF-1** | Stop-on-demand | A button press (or any detected fault) drives the machine to STOP within the process safety time. |
| **SF-2** | No spurious clear-to-run | The machine holds OK only while a sustained OK is *freshly and causally* derived from a correct live read of the switch loop on **both** channels, with a valid black-channel round-trip. |
| **SF-3** | Arming integrity | The machine may transition STOP→OK only via a deliberate arming gesture (STOP held ≥ `min_stop_ms`, then release) — never spontaneously, never on a transient. |

## 4. Function / item decomposition

Stable IDs. `[PQ]` = inside the pre-qualified `pstop_c` boundary (referenced,
not re-verified — only its interface contract is in scope).

### 4.1 Remote firmware — `firmware/` (`F-R-xx`)

| ID | Function | Primary location |
|---|---|---|
| F-R-01 | Dual-channel E-stop loop sensing (both-phase drive/echo, per-core channels A/B) | `main.c` `estop_channel_closed` |
| F-R-02 | Per-core verdict formation — Option A arithmetic-image + Option B diverse expressions | `main.c` `compute_verdict` |
| F-R-03 | Release debounce + boot warm-up hold | `main.c` (`LOOP_RECLOSE_DEBOUNCE_TICKS`, `LOOP_BOOT_OPEN_CONFIRM_TICKS`) |
| F-R-04 | Lockstep comparator (memcmp both cores' 40-byte encodings; transmit on agreement only) | `main.c` comparator/relay |
| F-R-05 | pstop message encode + transmit (via `pstop_c`) | `main.c` → `[PQ]` |
| F-R-06 | Black-channel round-trip verification (own counter/stamp echoed back) | `main.c` + `[PQ]` protocol checks |
| F-R-07 | Network transport mgmt (WG/Tailscale, iface selection, DERP, failover) | `dcs_net_*.c` |
| F-R-08 | Status indication (LED ring: OK/STOP/fault colours) | `dcs_rgb.c`, `dcs_pstop_ring.c` |
| F-R-09 | Boot, NVS config persistence, device identity | `dcs_boot.c`, `dcs_nvs.c`, `dcs_identity.c` |
| F-R-10 | Admin/web interface (config visibility, button/LED status) — **non-safety** | `dcs_admin_pages.c`, `dcs_admin_html.h` |

### 4.2 Host machine — `host/machine_app_runner.c` (`F-H-xx`)

| ID | Function | Notes |
|---|---|---|
| F-H-01 | Host the `pstop_c` machine role — receive & validate remote messages | wraps `[PQ]` |
| F-H-02 | Arming policy — enforce `min_stop_ms` gesture, veto short STOP→OK cycles | calls `machine_stop_robot()` |
| F-H-03 | Heartbeat/liveness monitoring → STOP on silence | `[PQ]` `check_heartbeats` on machine `CLOCK_MONOTONIC` |
| F-H-04 | Robot status output (OK/STOP) + operator-visible logging | |
| F-H-05 | Many-to-one aggregation — fail-safe OR across bonded remotes; arming ownership | |

### 4.3 ROS2 machine — `ros2/protective_stop_machine/` (`F-M-xx`)

| ID | Function | Location |
|---|---|---|
| F-M-01 | Lifecycle node mgmt (configure/activate/deactivate/cleanup/shutdown/error) | `machine_bridge_node.cpp` |
| F-M-02 | Backend abstraction (select software vs hardware) | `backend.hpp`, `build_backend` |
| F-M-03 | Software backend — host `pstop_c` machine on a UDP port | `software_backend.cpp` → `[PQ]` |
| F-M-04 | Hardware backend — poll ESP32 machine `/state.json` over libcurl | `hardware_backend.cpp` |
| F-M-05 | JSON parsing (depth-capped `json_lite`) | `json_lite.hpp` |
| F-M-06 | Publish status / relay / bonded-remotes on ROS2 topics | `machine_bridge_node.cpp` |
| F-M-07 | Parameter validation + dynamic (re)config | `on_set_parameters`, `validate_timing` |
| F-M-08 | Diagnostics (`diagnostic_updater`) | `diagnostics()` |

### 4.4 Pre-qualified `pstop_c` interface boundary (`F-P-xx`) — referenced

| ID | Function (referenced, pre-qualified) | Interface obligation on the app shell |
|---|---|---|
| F-P-01 | Message encode/decode + CRC-16 over bytes [0..37] | Shell must not alter the 40-byte layout or CRC span |
| F-P-02 | Machine dispatch — unknown/malformed ⇒ STOP (default→STOP) | Shell must preserve fail-safe default |
| F-P-03 | Heartbeat check on machine monotonic clock | Shell must supply a correct `get_time_cb`; never freeze it |
| F-P-04 | Counter/stamp black-channel echo + `MSG_LOST`/`OUT_OF_ORDER` validation | Shell must not spoof or bypass the echo |

## 5. Message-polarity & fail-safe invariants (analysis anchors)

- `OK == 0x00`, `STOP == 1U` — **inherited fail-danger polarity** (all-zeros
  message decodes toward OK). The live-sample derivation (F-R-02) is the
  compensating measure. This is a standing FMEA/requirements anchor.
- Sense inputs pulled **down**: an open loop reads 0 ⇒ STOP ⇒ fail-safe idle.
- Comparator is **transmit-on-agreement**: any divergence ⇒ send nothing ⇒
  machine heartbeat-timeout STOP. A single-channel fault is a stop, never a
  spoofable half-verdict on the wire.
- Machine default is **STOP**; OK is the exceptional, continuously-justified
  state.

## 6. Out of scope

- Re-verification of `pstop_c` internals (pre-qualified; own coverage/CI).
- The remote's admin/web UI beyond confirming it cannot influence the verdict
  path (F-R-10 is non-safety, but must be shown *non-interfering*).
- Physical/electrical HARA of the vehicle beyond the pstop function's boundary
  (the integrator's vehicle-level HARA consumes SF-1..SF-3 as inputs).
