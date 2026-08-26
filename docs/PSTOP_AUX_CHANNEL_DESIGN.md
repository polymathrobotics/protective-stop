<!--
SPDX-FileCopyrightText: 2026 Polymath Robotics
SPDX-License-Identifier: Apache-2.0
-->

# pstop auxiliary data channel

Status: **IMPLEMENTED (wrapper-only)** — schema + remote role announcement.
Machine→remote feedback direction is reserved and plumbed, not yet populated.

## 1. What this is

`pstop_c` v2 (`#104`) grew the wire message from 40 to 48 bytes by adding two
spare `uint32` fields, `padding1` and `padding2`, to `pstop_msg_t`. They are
covered by the frame CRC-16 and, on the remote, by the dual-core lockstep
byte-compare — but the certified library leaves them inert (zero on TX, ignored
on RX).

This document defines a **wrapper-only** schema that uses those bytes as a small
bidirectional data channel between remotes and machines. It rides the same
authenticated black channel as the STOP/OK signal, so it inherits that signal's
integrity and identity properties for free.

**The certified `pstop_c` library is never modified.** All encode/decode lives
in the shared, non-certified header `common/pstop_aux_channel.h`, included
identically by every wrapper (remote firmware, `machn`, `host`, ROS 2 machine)
so the layout cannot drift between implementations.

## 2. Wire layout

Each direction owns one 32-bit padding field. A version byte leads each field so
new sub-fields can be added later without another size change.

```
padding1 = UPLINK   (remote -> machine)
  bits  0.. 7   uplink schema version   (PSTOP_AUX_UP_VERSION = 0x01)
  bits  8..15   role                     (pstop_aux_role_t: 0/1/2)
  bits 16..31   reserved (zero)

padding2 = DOWNLINK (machine -> remote)   -- RESERVED, zero today
  bits  0.. 7   downlink schema version   (0 = no feedback present)
  bits  8..31   reserved (zero)
```

Ownership is by sender: a remote-originated frame fills `padding1` and leaves
`padding2 = 0`; a machine reply fills `padding2` and leaves `padding1 = 0`. The
wrapper operates on the decoded `pstop_msg_t` (native-endian `uint32`s), so the
schema is endianness-agnostic — `pstop_c` owns the little-endian serialization.

## 3. Role semantics (uplink)

`role` values are wire-stable:

| value | name | meaning |
|---|---|---|
| 0 | `UNSPECIFIED` | no claim (unprovisioned / version mismatch) — treated as non-operator |
| 1 | `STOP_ONLY` | may only STOP, never re-arm (monotonic toward safety) |
| 2 | `OPERATOR` | claims re-arm privilege, subject to machine policy |

Safety rules enforced by the machine wrappers (never by `pstop_c`):

1. **AND-rule (Phase 1).** A remote may re-arm iff it announces `OPERATOR`
   **and** the machine's existing allowlist already permits it. Formally the
   machine's `remote_details_cb` returns
   `stop_only = allowlist_stop_only OR (claimed_role != OPERATOR)`. This is
   strictly more conservative than today: a provisioning gap surfaces as
   stop-only, never as an unexpected operator.
2. **Fail-safe default.** Absent, stale, or undecodable role ⇒ stop-only.
   `UNSPECIFIED` is never treated as `OPERATOR`; an unknown role byte under a
   recognized version decodes to `STOP_ONLY`.
3. **Asymmetry.** A `STOP_ONLY` announcement is accepted immediately; an
   `OPERATOR` announcement is a privilege claim gated by rule 1.
4. **Role change drops ownership.** When a bonded remote's announced role
   changes, the machine forces an unbond → rebond so any arming ownership held
   through that remote is invalidated and a fresh gesture is required. A
   promotion never retroactively legitimizes a gesture in flight. This is
   implemented in the wrapper using only public `pstop_c` API (a synthesized
   UNBOND), not by editing the library.
5. **No new arming path.** The channel can only ever *remove* arming capability
   relative to today's allowlist.
6. **Freshness.** The role rides every heartbeat, so while a remote is bonded
   the claim cannot go stale; an unbonded remote cannot arm anyway. No separate
   TTL machinery is required.

## 4. Provisioning & source of truth

The remote is the source of truth for its own role:

- Stored in remote NVS (key `role`, `u8`), **default `stop_only`** — a freshly
  provisioned or NVS-wiped remote can never arm until deliberately promoted.
- Set via the admin API `POST /api/role?role=stop_only|operator` and a toggle in
  the remote WebUI; exported in the remote's `state.json`.
- Machines expose each bonded remote's decoded `claimed_role` in their own
  `state.json` / ROS 2 `~/remotes` topic for observability (display only —
  never a policy input outside the AND-rule above).

## 5. Downlink (machine → remote) — reserved

`padding2` is reserved for future machine→remote feedback (e.g. relay/arm
state, stop reason). The plumbing exists today:

- Every machine wrapper calls `pstop_aux_encode_feedback_none()` on its reply
  (emits zero), keeping a single obvious home for the future encoder.
- The remote already reads the field via `pstop_aux_read_feedback_raw()` in its
  reply-drain path and ignores it while the downlink version is 0.

Adding real feedback later means defining a `PSTOP_AUX_DOWN_VERSION` layout in
the shared header and populating/consuming it — no wire size change and no
fleet rollout coordination beyond the normal flash cycle.

## 6. Rollout

`pstop_c` v2 is a hard wire cutover (40→48 bytes, different CRC; no v1/v2
interop). All remotes and all three machine implementations must be flashed to
v2 together. Because remotes default to `stop_only` and the machine AND-rule
requires both the claim and the existing allowlist, nothing gains privilege it
did not already have; provisioning gaps surface as stop-only.

## 7. Where it lives

| Piece | Location |
|---|---|
| Schema + encode/decode | `common/pstop_aux_channel.h` |
| Remote role NVS + API + WebUI + TX encode | `firmware/` |
| Machine decode + AND-rule + rebond-on-change | `machn/`, `host/`, `ros2/protective_stop_machine/` |
| `claimed_role` observability | machine `state.json`, ROS 2 `BondedRemote.msg` |

`pstop_c/` is untouched by all of the above.
