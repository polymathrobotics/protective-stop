<!--
SPDX-FileCopyrightText: 2026 Polymath Robotics
SPDX-License-Identifier: Apache-2.0
-->

# Remote role announcement

`pstop_c` v2 added two CRC-protected `uint32` padding fields to its 48-byte
message. Wrapper code uses `padding1` to let a remote announce whether it is an
operator or stop-only. The certified `pstop_c` library remains unchanged.

## Wire layout

`padding1` is encoded by `common/pstop_aux_channel.h`:

```text
bits  0..7   schema version (1)
bits  8..15  role: 0 unspecified, 1 stop-only, 2 operator
bits 16..31  zero
```

Unknown versions decode as unspecified. Unknown role values under version 1
decode as stop-only. Only an explicit operator value can contribute operator
authority.

## Policy

A remote may re-arm only when both conditions hold:

1. The BOND announces `OPERATOR`.
2. The machine's existing allowlist grants that remote operator authority.

The machine callback therefore applies:

```text
stop_only = allowlist_stop_only OR announced_role != OPERATOR
```

`pstop_c` latches the result into the bonded client. Later heartbeats do not
change authorization. This keeps policy changes on the existing BOND boundary
and prevents rejected or replayed heartbeat traffic from mutating role state.

## Provisioning and lifecycle

The role is stored in remote NVS and defaults to stop-only. `POST /api/role`
persists a new value and reboots the remote. The reboot interrupts heartbeats,
so every machine drops the old bond and forces STOP before accepting the new
BOND. Re-arming still requires a fresh STOP-to-OK gesture.

The active role is exposed as `role` in the remote's `state.json`. Machine-side
authorization remains observable through each bonded remote's effective
`stop_only` value.

## Rollout

The v2 message size is a hard wire cutover from 40 to 48 bytes. Update remotes
and machine implementations together. Existing remotes without the NVS role key
start as stop-only and must be explicitly promoted where re-arm authority is
required.
