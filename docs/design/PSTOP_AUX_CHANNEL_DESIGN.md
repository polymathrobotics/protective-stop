<!--
SPDX-FileCopyrightText: 2026 Polymath Robotics
SPDX-License-Identifier: Apache-2.0
-->

# Remote role announcement

`pstop_c` v2 added two CRC-protected `uint32` padding fields to its 48-byte
message. Wrapper code uses `padding1` to let a remote announce whether it is an
operator or stop-only. The certified `pstop_c` library remains unchanged.

> **Forward-compatibility:** `padding1` is the remote-to-machine field and keeps
> 16 bits reserved for future remote announcements. `padding2` is reserved in
> full for a future machine-to-remote schema, such as machine state or stop
> reason feedback. The planned convention for `padding2` is an 8-bit schema
> version followed by 24 bits of direction-specific data. It remains zero and
> uninterpreted in this version.

## Wire layout

`padding1` is encoded by `common/pstop_aux_channel.h`:

```text
bits  0..7   schema version (1)
bits  8..15  role: 0 unspecified, 1 stop-only, 2 operator
bits 16..31  reserved for future remote-to-machine additions (zero today)
```

`padding2` is owned by machine replies but has no active schema yet:

```text
bits  0..7   reserved for a future machine-to-remote schema version (zero today)
bits  8..31  reserved for future machine-to-remote data (zero today)
```

Senders must zero every reserved bit. Receivers must ignore `padding2` until a
nonzero machine-to-remote schema version is defined and implemented.

Unknown versions decode as unspecified. Unknown role values under version 1
decode as stop-only. Only an explicit operator value can contribute operator
authority.

## Policy

Two independent decisions, deliberately kept apart.

### Authority: the remote alone decides

A remote may re-arm a machine when — and only when — the frame carrying its OK
announces `OPERATOR`. Machines do not consult any list for this: the machine
callback seeds `is_stop_only` from the BOND frame's announced role and every
later frame refreshes it.

```text
stop_only = announced_role != OPERATOR
```

`UNSPECIFIED` (pre-role firmware, unknown schema version) and unknown role
values decode as stop-only, so the failure mode of a provisioning gap is always
"cannot re-arm", never "unexpected operator".

The role is **live**. `POST /api/role` on the remote applies to its next frame
(no reboot) and the machine re-reads it per frame, giving these semantics:

| Situation | Machine behaviour |
|---|---|
| Armed; the remote that armed it demotes itself to `stop_only` | keeps **running**; that remote's arming-cycle ownership is released |
| ...then that remote presses STOP | machine **stops** and refuses to re-arm on its release (`NEED_STOP`) |
| ...and presses again | still refused — a stop-only remote never opens an arming cycle |
| some remote announcing `OPERATOR` performs STOP → OK | arms |
| the demoted remote promotes itself back | its next STOP → OK arms |

Why the machine needs two hooks around `machine_process_message()`
(`common/pstop_aux_channel.h`, `pstop_aux_apply_role_pre/post`): `pstop_c`
consults `is_stop_only` only when a STOP *acquires* ownership of the arming
cycle (`remote_stop_id == 0`). Once a remote has armed the machine it stays the
owner and its later STOP re-opens the cycle unconditionally; and from
`restart_state = OK` a stop-only STOP leaves `restart_state` untouched, so the
release would re-arm. The pre-hook releases ownership held by a stop-only remote
(voiding a half-open cycle to `NEED_STOP`); the post-hook forces `NEED_STOP`
after any accepted STOP from a stop-only remote. `pstop_c` itself is unmodified;
only public structs are touched, identically in all three machines.

Because the role is self-asserted (CRC only, no authentication), it must never
*grant* anything the machine would otherwise refuse — which is why admission is
a separate, machine-owned decision.

### Admission: optional, machine-owned, global

Whether a remote may **bond at all** is decided by two optional lists on the
machine, both empty by default (**open**: every remote is admitted). This is the
normal way to run; a fleet that wants a "paranoid" posture populates them.

| List | Effect |
|---|---|
| `allowlist` non-empty | **only** listed ids may bond |
| `denylist` | listed ids may **never** bond; wins over the allowlist |

Admission is evaluated once, at BOND (`remote_details_t.allowed`). A refused
BOND is answered with the `UNBOND` reply `pstop_c` already prepares
(`protocol.c`), addressed to the remote, instead of silence. On the remote a
BOND answered with `UNBOND` parks that session in `REJECTED` (`state: 3` in
`state.json`, red chip in the web UI, `last_msg = UNBOND`) with **no automatic
retries** — it waits for `POST /api/pstop_peers?slot=N&rebond=1` (the Rebond
button), a slot reconfigure, or a reboot. This keeps a banned or misconfigured
remote from knocking forever while making the refusal visible.

Where the lists live:

| Machine | Configuration |
|---|---|
| ESP32 `machn` | NVS via `GET/POST /api/admission` (`?allow=`, `?unallow=`, `?deny=`, `?undeny=`); web UI; `state.json` `allowlist`/`denylist` |
| ROS 2 node | `software.allowlist`, `software.denylist` (int arrays) |
| host runner | `[policy] allowlist = [...]`, `denylist = [...]` in `machine.toml` |

Admission never affects authority: an admitted remote still re-arms only if it
announces `OPERATOR`.

## Provisioning and lifecycle

The role is stored in remote NVS and defaults to stop-only. `POST /api/role`
persists a new value and applies it live; the next outbound frame announces it
and every bonded machine honours it on receipt (see the table above). No
re-bond is needed. The active role is exposed as `role` in the remote's
`state.json` and on its web UI.

Machine-side, each bonded remote's effective `stop_only` is observable (ROS 2
`/machine_bridge/remotes`, machn `state.json` `bonded_remotes`, host runner
`ROLE` log lines) and now tracks the announced role live.

## Rollout

The v2 message size is a hard wire cutover from 40 to 48 bytes. Update remotes
and machine implementations together. Existing remotes without the NVS role key
start as stop-only and must be explicitly promoted where re-arm authority is
required. A machine that previously carried an operator list does **not**
inherit it as an admission allowlist: the meanings differ ("may re-arm" vs
"may bond") and reinterpreting it would lock out every other remote after an
OTA. machn migrates the legacy `operators` ids once into its **pin list** (they
keep their WireGuard peer pinning, which is what made cross-site remotes bond
reliably) and comes up in open admission; add ids to the allowlist deliberately
if a paranoid posture is wanted.

One observed edge worth knowing: if a remote is promoted to `operator` *while
its button is held*, its continuing STOP frames are now operator STOPs and open
an arming cycle, so the release arms (min-STOP still counted from the first
operator STOP). Role is evaluated per frame; promotion is an authenticated
admin action, so this is by design.
