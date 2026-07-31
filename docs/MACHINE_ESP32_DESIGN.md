# pstop machine on ESP32-S3 — design sketch (v0, for review)

**Status:** v0.1 — review answers folded in, 2026-07-31. Deliberately
short — structure over detail. Nothing implemented yet.

## Goal

Deploy the pstop_c **machine** role on the same ESP32-S3 platform as the
remote: a self-contained box that bonds remotes, runs the safety decision
dual-core, and actuates the robot's stop circuit through relays. The pure
software machine (`host/machine_app_runner`) remains fully supported — same
library, same wire protocol, same config semantics; this is a second
machine form factor, not a replacement.

## Reused from the remote (unchanged infrastructure)

- **Hardware**: Waveshare ESP32-S3-ETH (PoE), same enclosure family.
- **Connectivity**: microlink (Tailscale/WireGuard/DERP), eth → wifi
  supervised uplinks. Own tailnet identity derived from eFuse MAC, prefix
  distinguishes the role: **`machn-01<mac24>`** (remotes keep `pstop-`).
- **Platform services**: NVS-persisted settings + admin UI, boot-count
  safety ladder, direct OTA (`/admin/api/ota`), `/state.json` telemetry
  (with `fw_ver`/`fw_sha`), reset-history, panic log.
- **pstop_c untouched**, as always: the machine role is what the library
  already provides; everything below is the shell around it.

## Dual-core RX lockstep (mirror of the remote's TX lockstep)

The remote's pattern: two cores independently *encode*, byte-compare,
transmit only on agreement. The machine inverts it:

- Each core runs its **own pstop_c machine instance** (own state, own
  remotes table) and independently decodes + validates every received
  datagram and computes (verdict, reply bytes).
- A comparator (same 100 ms tick discipline as the remote) compares the
  two cores' verdicts AND reply bytes. **Agree** → actuate relays to the
  agreed state, send the reply. **Disagree** → treat as internal fault:
  both relay outputs commanded open (STOP), reply withheld (the remote
  sees silence and stops, exactly like a remote-side mismatch), mismatch
  counter incremented (purple-equivalent indication, persistent-only per
  the current ring policy).
- Liveness (`check_heartbeats`) runs per core on the shared monotonic
  clock; a timeout verdict must also agree before it opens the relays —
  with the caveat that *either* core alone opening its relay already
  stops the robot (see below), so disagreement is never less safe.

## Relay output + feedback (replaces the remote's button inputs)

Two opto-isolated relays, contacts **wired in series** in the robot's
stop circuit — either core alone can break the circuit:

- Core 0 drives relay A, core 1 drives relay B (one GPIO each through
  opto-isolated drivers, reusing the remote's E-stop loop pins).
  **Energized = circuit closed = robot may run; de-energized = STOP.**
  Power loss, brownout, reset, or a hung core all fail to STOP.
- **Feedback = output-voltage sensing**: a resistor divider on each
  relay's switched output, read as a digital input by BOTH cores. After
  any commanded transition the observed voltage must match within
  `RELAY_FEEDBACK_MS` (TBD from relay switching time) and is also
  checked continuously at every comparator tick.
- **Fault semantics — nothing latches.** Transient conditions (a
  momentary core disagreement, a momentary loss of remote signal)
  already self-recover exactly as on the remote: the comparator
  withholds/opens for the duration and resumes when agreement returns.
  A *feedback contradiction* (e.g. output still live after commanding
  open — welded contact / shorted driver) raises a persistent
  **relay-fault indication** for as long as the condition is observed:
  telemetry flag, fault counter, and the machine keeps both outputs
  commanded open. Safety does not depend on latching — the series
  partner relay breaks the circuit — but the indication persists so a
  single-channel box gets serviced; it clears by itself when feedback
  returns to sanity.

## Reporting to the ROS2 computer

The ESP32 machine replaces the runner's *decision* role but the robot
computer still needs live state. Two channels, both read-only:

**`/state.json`** gains machine-role fields: robot verdict, restart
state, arming owner, per-remote table (bonded/heartbeat age), relay
commanded/observed states, mismatch + fault counters. A thin **ROS2
node** (successor of the archived `protective_stop_node`) polls it at
5 Hz and republishes as topics — `robot_status`, `relay_state`,
`fault`. No new firmware surface; no UDP stream.

The stop circuit itself is the relay chain — the ROS2 reporting path
carries **no safety responsibility**.

## Fleet / OTA policy (config-gated, per decision)

- The machine runs the same fleet check-in code but registers with
  **`device_type: "machine"`** in the check-in payload (new field,
  remotes implicitly `"remote"`).
- **Server-side enforcement (required, fleet-console work):** the console
  must refuse to assign remote firmware to `device_type != remote` —
  machines get their own image lineage or nothing. This needs a handoff
  item to the console session before any machine unit ships.
- Belt-and-braces on the device: machine builds set `auto_update`
  default **off**, and the OTA-apply path verifies the incoming image's
  project name (`machn_machine` vs `pstop_remote` in the app
  descriptor) and rejects a mismatch regardless of what the server says.
- Direct OTA (`/admin/api/ota`) remains the recovery path, same as today.

## Config

The `machine.toml` knobs move to NVS + admin UI (same pattern as the
remote's settings): operator allowlist (`allow_unlisted`, per-remote
`stop_only`/`heartbeat_ms`), `max_missed_heartbeats`, `min_stop_ms`
(→ library `delay_between_stop_ms`), `machine_device_id`, listen port.
Persisted, live-applied where safe.

## Out of scope for v1

Certification argumentation; multi-machine coordination (remote side
already handles up to 4 machines — an ESP32 machine is just one of
them); LED ring on the machine box (onboard RGB only for v1, TBD);
console MACHINES-tab integration beyond the existing announce.

## Resolved in review (2026-07-31)

1. Relay: opto-isolated module; feedback via resistor divider on the
   switched output read as digital IO. Exact part TBD during hardware
   pass; `RELAY_FEEDBACK_MS` set from its switching time.
2. No fault latching: transients self-recover; a persistent feedback
   contradiction shows a persistent (self-clearing) fault indication —
   series redundancy carries the safety.
3. ROS2 reporting: poll `/state.json` at 5 Hz; no UDP stream.
4. Arming: remote gestures only — the machine box has no arm input.
5. Naming: remotes keep `pstop`; machines use `machn`.
