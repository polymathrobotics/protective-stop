# Single remote, multiple machines

One remote can bond to up to 4 machines at once.
This page describes how the remote and each machine behave when machine M1 and machine M2 share remote R, and what R's one button does to all of them.
For many remotes on one machine, see [Multi-remote, single machine](MULTI_REMOTE_SINGLE_MACHINE.md).
The behavior comes from `firmware/main/main.c` and the machine logic in `pstop_c/pstop/src/pstop/machine.c`.
The configuration API is in [`API.md`](../guides/API.md), under "Multi-machine".

## Model

- **Sessions.** R keeps one session per configured peer slot (0 to 3).
  Each session has its own UDP socket (local ports 8891 to 8894, in slot order), bond state, message counters, and reply-loss watchdog.
- **One verdict.** R samples its stop loops once per tick and encodes one verdict, STOP or OK.
  Every bonded session carries that verdict in the same tick.
  Pressing the button stops every machine, and releasing it offers OK to every machine.
- **Shared comparator.** The two cores compare every session's encoded message byte for byte before anything is sent.
  Any mismatch sends nothing to any machine.
- **Independent machines.** A machine stops by its own heartbeat timeout (`heartbeat_ms x max_missed`, 2 s by default), whatever the other machines are doing.
  R never sends UNBOND, so silence is how a machine learns R has gone.
- **Per-machine rate.** Each machine advertises its `heartbeat_ms` in every reply.
  R sends to that session every half window, clamped to 100 to 1000 ms.
  The stop loops are sampled at 10 Hz regardless.
- **One role.** R announces a single role, operator or stop-only, and every machine receives the same one.
- **Reply-loss watchdog.** A bonded session with no reply for `max(2.5 s, 5 x heartbeat_ms + 0.5 s)` drops back to bonding.
  A bond attempt is retried every 5 s.
- **Slot identity.** Each slot has an IP, port, and optional machine id (`&id=`, default `0x01020304`).
  Give machines distinct ids, because R addresses each message to the slot's id.

## Scenarios

M1 and M2 each have only R bonded, and R is an operator, unless stated.
"Armed" means the machine is running.
"Gesture" means R's button pressed and released.

| # | Situation | M1 | M2 | To run again |
|---|-----------|----|----|--------------|
| 1 | R bonds both machines | `NEED_STOP` | `NEED_STOP` | One gesture arms both |
| 2 | Button pressed | Stops | Stops | Release |
| 3 | Button released after `min_stop_ms` | Arms | Arms | Nothing |
| 4 | Both armed, M2 unreachable | Keeps running | Stops at its timeout | R re-bonds M2, then a gesture |
| 5 | M2 returns | Keeps running | `NEED_STOP` | A gesture, which also stops M1 |
| 6 | Comparator mismatch | Stops at its timeout | Stops at its timeout | Fix the fault, then a gesture |
| 7 | R loses power or all network | Stops at its timeout | Stops at its timeout | R re-bonds both, then a gesture |
| 8 | Slot 1 is cleared | Unaffected | Stops at its timeout | Reconfigure, then a gesture |
| 9 | Slot 1 is reconfigured | Unaffected | Session resets and re-bonds | A gesture |
| 10 | R is stop-only | Stops on a press | Stops on a press | Never, from R |
| 11 | M1 and M2 request different heartbeats | R sends at half M1's window | R sends at half M2's window | Nothing |

### 1. Initial bond

R bonds each machine as an independent session.
A machine that has no other remote starts in `NEED_STOP`.
R's OK heartbeats are answered with STOP until the arming gesture.
Booting R does not arm anything, because R sends nothing until both stop loops have been sampled on both phases.

### 2. Button pressed

The STOP verdict goes to every bonded session in the same tick.
Both machines stop, and each treats R as the owner of the arming cycle.
The press does not depend on any machine's reply.
A machine that is unreachable stops on its own timeout.

### 3. Button released

R sends OK to every machine.
Each machine accepts the OK only after its own `min_stop_ms` (500 ms by default) since the STOP.
An earlier OK is answered with STOP, the cycle stays open, and the next OK is accepted.
One gesture arms both machines even when their `min_stop_ms` values differ.
The machines may arm at slightly different times.

### 4. A machine becomes unreachable

R keeps heartbeating the machines that reply.
M2 stops at its own timeout, and M1 is not affected.
After 2.5 s without a reply R drops M2's session and bonds again every 5 s.
R never stalls on one dead machine.

### 5. A machine comes back

M2 treats R as a new remote.
When R is M2's only remote, M2 starts in `NEED_STOP` and answers R's OK with STOP.
R must be pressed and released to arm M2.
That STOP also reaches M1, which stops and re-arms along with M2.
Re-arming one machine this way always briefly interrupts the others, because the verdict is shared.
When M2 has other bonded remotes and is still armed, R joins without changing M2's state.

### 6. Comparator mismatch

If the two cores disagree on any session's encoded bytes, or one core misses its publish deadline, R sends nothing to any machine that tick.
Every machine stops on its own timeout.
This is device-wide, not per link.
Each session then drops to bonding after its reply-loss threshold.

### 7. R loses power or all paths

Every machine times out and stops.
When R returns, each machine bonds it as a new remote.
A machine with no other remotes starts in `NEED_STOP`, so a gesture is needed.
If R restarts faster than a machine's timeout, that machine may still hold R's old slot.
R's counters restart, so the machine rejects those messages until R's session re-bonds.

### 8. Clearing a slot

The session goes idle and R stops sending to that machine.
The machine does not receive an UNBOND, so it stops at its own timeout.
That is the same as losing R.

### 9. Reconfiguring a slot

Changing a slot's IP, port, or id resets only that session.
Its counters restart, and it bonds again.
Other sessions keep their counters and state.

### 10. R is stop-only

A stop-only R can stop every machine but cannot re-arm any of them.
A machine that has no operator remote bonded stays stopped.

### 11. Different heartbeat settings

Each machine's heartbeat comes back in its replies, so R follows each machine's own rate.
R advances a session's counter only on ticks that actually transmit, so the machine sees contiguous counters.

## Practical consequences

- Pressing R's button stops every machine, and arming one machine re-arms all that are in `NEED_STOP`.
  Arming a late-joining machine interrupts the machines that are already running.
- A single unreachable machine never stops the others, but the machine itself will stop after its timeout.
  R's `/state.json` `pstop_machines[]` shows each slot's `state`, `sent`, `replies`, `send_fail`, `rebonds`, `hb_ms` and `last_reply_ms`.
- A session in state 2 (bonded) with `replies` climbing is healthy.
  State 1 means bonding, and state 0 means the slot is not configured.
- The LED ring is divided evenly among the configured slots, in slot order from LED 1, so each machine has its own segment.
- Hardware validation is in `tools/pstop_multi_machine_test.py`.
  It checks bonding, per-machine isolation, recovery, slot clearing, and that the mismatch counter stays flat.
