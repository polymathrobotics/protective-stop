# pstop machine on ESP32-S3 — design sketch (v0, for review)

**Status:** draft for comment, 2026-07-31. Deliberately short — structure
over detail. Nothing implemented yet.

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
  distinguishes the role: **`pmach-01<mac24>`**.
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

Two relays, contacts **wired in series** in the robot's stop circuit —
either core alone can break the circuit:

- Core 0 drives relay A, core 1 drives relay B (one GPIO each, reusing
  the remote's E-stop loop pins). **Energized = circuit closed = robot
  may run; de-energized = STOP.** Power loss, brownout, reset, or a hung
  core all fail to STOP.
- Each relay provides an aux (ideally force-guided) **feedback contact**;
  BOTH cores read BOTH feedbacks. After any commanded transition, the
  observed state must match within `RELAY_FEEDBACK_MS` (~50 ms, TBD by
  relay datasheet) or the machine declares a **relay fault**: both
  outputs open, fault latched (requires reboot or explicit admin clear —
  TBD), fault visible in telemetry and to the remotes (replies STOP).
- Feedback is also monitored continuously at the comparator tick, not
  just on transitions — a welded or externally-forced contact is caught
  within one tick.

## Reporting to the ROS2 computer

The ESP32 machine replaces the runner's *decision* role but the robot
computer still needs live state. Two channels, both read-only:

1. **`/state.json`** gains machine-role fields: robot verdict, restart
   state, arming owner, per-remote table (bonded/heartbeat age), relay
   commanded/observed states, mismatch + fault counters.
2. A small **UDP status stream** (JSON datagram, ~5 Hz, to a configured
   `status_host:port`) so the ROS2 side gets sub-second state without
   polling. A thin ROS2 node (successor of the archived
   `protective_stop_node`) subscribes and republishes as topics —
   `robot_status`, `relay_state`, `fault`. Schema TBD in v1.

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
  project name (`pmach_machine` vs `pstop_remote` in the app descriptor)
  and rejects a mismatch regardless of what the server says.
- Direct OTA (`/admin/api/ota`) remains the recovery path, same as today.

## Config

The `machine.toml` knobs move to NVS + admin UI (same pattern as the
remote's settings): operator allowlist (`allow_unlisted`, per-remote
`stop_only`/`heartbeat_ms`), `max_missed_heartbeats`, `min_stop_ms`
(→ library `delay_between_stop_ms`), `machine_device_id`, listen port,
status-stream target. Persisted, live-applied where safe.

## Out of scope for v1

Certification argumentation; multi-machine coordination (remote side
already handles up to 4 machines — an ESP32 machine is just one of
them); LED ring on the machine box (onboard RGB only for v1, TBD);
console MACHINES-tab integration beyond the existing announce.

## Open questions for review

1. Relay hardware: specific part (force-guided aux contacts, coil
   voltage from VBUS/5 V?) — drives the feedback-timing constant and
   enclosure work.
2. Fault latch semantics: reboot-to-clear, admin-API clear, or
   power-cycle only?
3. Status stream transport: raw UDP JSON (proposed) vs. the ROS2 node
   polling `/state.json` at 2–5 Hz (no new firmware surface)?
4. Should the machine box have a physical reset/arm input of its own,
   or is arming exclusively via remote gestures (proposed: remotes only)?
5. Project/image naming: `pmach` prefix OK?
