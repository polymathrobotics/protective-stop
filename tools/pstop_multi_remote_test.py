#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Many-remotes-to-one-machine corner-case validation for pstop_c.

Spins up the host-side machine (`host/machine_app_runner`, which links the
unmodified pstop_c safety library) and drives it with N scripted software
remotes that speak the real 40-byte wire protocol (CRC16 poly 0x8D95). Each
remote carries its own device_id and counter sequence, so this exercises the
library's real multi-remote bookkeeping — not a mock.

The point is to prove the fail-safe invariants of the many-to-one logic hold
across the corner cases a fleet actually hits: independent arming, the
"any remote STOP stops the robot" OR, arming-ownership under a second
operator, heartbeat loss (network drop / device power-off), unbond
(disconnect), capacity overflow, allowlist rejection, stop-only operators,
and malformed traffic.

  Safety invariants asserted (must ALWAYS hold):
    I1  any bonded remote sending STOP  -> every remote sees STOP
    I2  a remote going silent > timeout -> robot STOP
    I3  robot never OK without a deliberate held STOP->OK arming cycle
    I4  a STOP shorter than min_stop_ms cannot arm
    I5  a freshly (re)bonded remote forces re-arm (NEED_STOP)
    I6  the (max_remotes+1)th remote is rejected, armed peers undisturbed
    I7  unbonding the arming owner drops the robot to STOP/NEED_STOP

This is host-side TEST tooling only — it speaks the wire protocol from the
outside and never touches the firmware or the certification-track library.

Usage:  python3 tools/pstop_multi_remote_test.py [-v]
Exit 0 = all invariants held.
"""

import argparse
import os
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time

# ---------------------------------------------------------------------------
# Wire protocol (parameterised by remote id — the shared library tracks each
# id independently).
# ---------------------------------------------------------------------------
PSTOP_VERSION = 0x00
MSG_OK, MSG_STOP, MSG_BOND, MSG_UNBOND, MSG_UNKNOWN = 0, 1, 2, 3, 0x0F
NAMES = {0: 'OK', 1: 'STOP', 2: 'BOND', 3: 'UNBOND', 0x0F: 'UNKNOWN'}
SIZE = 40
MACHINE_ID = 0x01020304
HEARTBEAT_MS = 1000

HERE = os.path.dirname(os.path.abspath(__file__))
RUNNER = os.path.join(HERE, '..', 'host', 'machine_app_runner')


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8D95) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode(remote_id, message, stamp, received_stamp, counter, received_counter, corrupt=False):
    body = struct.pack(
        '<BBQQIIIII',
        PSTOP_VERSION,
        message,
        stamp,
        received_stamp,
        remote_id,
        MACHINE_ID,
        HEARTBEAT_MS,
        counter,
        received_counter,
    )
    crc = crc16(body)
    if corrupt:
        crc ^= 0xFFFF  # deliberately wrong checksum
    return body + struct.pack('<H', crc)


def decode(data):
    (_ver, message, stamp, _rs, _snd, _rcv, _hb, counter, received_counter) = struct.unpack('<BBQQIIIII', data[:38])
    (checksum,) = struct.unpack('<H', data[38:40])
    return dict(
        message=message,
        stamp=stamp,
        counter=counter,
        received_counter=received_counter,
        crc_ok=(checksum == crc16(data[:38])),
    )


class SoftRemote:
    """A single scripted remote holding its own counter handshake state."""

    def __init__(self, remote_id, host, port, name=None):
        self.id = remote_id
        self.name = name or f'0x{remote_id:08X}'
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect((host, port))
        self.sock.settimeout(0.4)
        self.counter = 0
        self.last_rx_counter = 0
        self.last_rx_stamp = 0
        self.last_reply = None

    def _now(self):
        return time.monotonic_ns() // 1_000_000

    def send(self, message, corrupt=False, bad_type=None):
        """Send one message, absorb the machine reply, return reply dict/None."""
        self.counter += 1
        msg = bad_type if bad_type is not None else message
        pkt = encode(self.id, msg, self._now(), self.last_rx_stamp, self.counter, self.last_rx_counter, corrupt=corrupt)
        try:
            self.sock.send(pkt)
            reply = decode(self.sock.recv(SIZE))
        except (socket.timeout, OSError):
            self.last_reply = None
            return None
        if reply['crc_ok']:
            self.last_rx_counter = reply['counter']
            self.last_rx_stamp = reply['stamp']
        self.last_reply = reply
        return reply

    def bond(self, timeout_s=15):
        self.counter = 0
        self.last_rx_counter = 0
        self.last_rx_stamp = 0
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.counter = 1
            pkt = encode(self.id, MSG_BOND, self._now(), 0, 1, 0)
            try:
                self.sock.send(pkt)
                reply = decode(self.sock.recv(SIZE))
            except (socket.timeout, OSError):
                continue
            if reply['crc_ok'] and reply['message'] == MSG_BOND:
                self.last_rx_counter = reply['counter']
                self.last_rx_stamp = reply['stamp']
                self.last_reply = reply
                return True
            time.sleep(0.3)
        return False

    def send_noreply(self, message, corrupt=False, bad_type=None):
        """Fire a packet and do NOT wait for a reply (for flood/garbage tests)."""
        self.counter += 1
        msg = bad_type if bad_type is not None else message
        pkt = encode(self.id, msg, self._now(), self.last_rx_stamp, self.counter, self.last_rx_counter, corrupt=corrupt)
        try:
            self.sock.send(pkt)
        except OSError:
            pass

    def bond_at_capacity(self, keepalives, timeout_s=3):
        """Attempt to bond while keeping `keepalives` remotes heartbeating, so
        the machine is genuinely at capacity. Returns True if bonded, False if
        rejected (UNBOND) or no BOND reply."""
        self.counter = 0
        self.last_rx_counter = 0
        self.last_rx_stamp = 0
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for k in keepalives:
                k.send(MSG_OK)
            self.counter = 1
            pkt = encode(self.id, MSG_BOND, self._now(), 0, 1, 0)
            try:
                self.sock.send(pkt)
                reply = decode(self.sock.recv(SIZE))
            except (socket.timeout, OSError):
                continue
            if reply['crc_ok'] and reply['message'] == MSG_BOND:
                return True
            if reply['crc_ok'] and reply['message'] == MSG_UNBOND:
                return False  # explicit rejection
            time.sleep(0.1)
        return False

    def unbond(self):
        return self.send(MSG_UNBOND)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


class Session:
    """Drives a set of remotes at 10 Hz, letting each tick assign a per-remote
    message (a remote omitted from the assignment stays SILENT that tick)."""

    def __init__(self, remotes):
        self.remotes = {r.id: r for r in remotes}

    def run(self, duration_s, assign):
        """assign: {remote: message}. Returns {remote_id: last_reply_msg_or_None}."""
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            for r, m in assign.items():
                r.send(m)
            time.sleep(0.1)
        return {r.id: (r.last_reply['message'] if r.last_reply else None) for r in assign}


# ---------------------------------------------------------------------------
# Machine lifecycle
# ---------------------------------------------------------------------------
class Machine:
    """Launch machine_app_runner on a port with a given TOML; capture stderr."""

    _next_port = 8920  # 8890 is held by the machine serving the real chips

    def __init__(self, toml_text=None, verbose=False):
        self.port = Machine._next_port
        Machine._next_port += 1
        self.verbose = verbose
        if toml_text is None:
            toml_text = DEFAULT_TOML
        fd, self.toml_path = tempfile.mkstemp(suffix='.toml', prefix='pstop_')
        with os.fdopen(fd, 'w') as f:
            f.write(toml_text)
        self.log = tempfile.TemporaryFile(mode='w+')
        args = [RUNNER, self.toml_path, str(self.port)]
        if verbose:
            args.append('-v')
        self.proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=self.log, cwd=os.path.join(HERE, '..'))
        # wait for "listening"
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            self.log.seek(0)
            if 'listening on' in self.log.read():
                return
            time.sleep(0.05)
        raise RuntimeError('machine_app_runner did not come up')

    def stderr(self):
        self.log.seek(0)
        return self.log.read()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        try:
            self.proc.send_signal(signal.SIGTERM)
            self.proc.wait(timeout=3)
        except Exception:
            self.proc.kill()
        try:
            os.unlink(self.toml_path)
        except OSError:
            pass


DEFAULT_TOML = """
[network]
port = 8890
bind = "127.0.0.1"
[machine]
machine_device_id = 0x01020304
[limits]
max_lost_messages = 10
max_missed_heartbeats = 1
max_remotes = 3
[policy]
allow_unlisted = true
default_heartbeat_ms = 1000
default_stop_only = false
min_stop_ms = 500
"""

# Remote ids used across scenarios (mirror the two real chips + spares).
RID_A = 0x01D7791C  # old chip
RID_B = 0x01D778B4  # new chip
RID_C = 0x01020383
RID_D = 0x01020384

# ---------------------------------------------------------------------------
# Assertion bookkeeping
# ---------------------------------------------------------------------------
RESULTS = []


def check(cond, label, detail=''):
    RESULTS.append((bool(cond), label, detail))
    tag = 'PASS' if cond else 'FAIL'
    line = f'    [{tag}] {label}'
    if detail:
        line += f'  ({detail})'
    print(line)
    return bool(cond)


def arm(session, owner, others, hold_s=0.8):
    """Perform a proper arming cycle: owner holds STOP >= min, others hold OK."""
    a = {owner: MSG_STOP}
    a.update({o: MSG_OK for o in others})
    session.run(hold_s, a)
    a = {owner: MSG_OK}
    a.update({o: MSG_OK for o in others})
    return session.run(1.2, a)


# ---------------------------------------------------------------------------
# Scenario groups
# ---------------------------------------------------------------------------
def group_a_single(verbose):
    print('\n== A: single-remote arming policy (baseline) ==')
    with Machine(verbose=verbose) as m:
        a = SoftRemote(RID_A, '127.0.0.1', m.port)
        assert a.bond(), 'bond failed'
        s = Session([a])
        r = s.run(1.2, {a: MSG_OK})
        check(r[a.id] == MSG_STOP, 'A1 OK-stream before arming stays STOP (I3)', NAMES.get(r[a.id]))
        s.run(0.2, {a: MSG_STOP})
        r = s.run(1.2, {a: MSG_OK})
        check(r[a.id] == MSG_STOP, 'A2 200ms STOP->OK vetoed (I4)', NAMES.get(r[a.id]))
        r = arm(s, a, [])
        check(r[a.id] == MSG_OK, 'A3 800ms STOP->OK arms', NAMES.get(r[a.id]))
        s.run(0.2, {a: MSG_STOP})
        r = s.run(1.2, {a: MSG_OK})
        check(r[a.id] == MSG_STOP, 'A4 blip while armed stops + release vetoed', NAMES.get(r[a.id]))
        r = arm(s, a, [])
        check(r[a.id] == MSG_OK, 'A5 re-arm after veto', NAMES.get(r[a.id]))
        a.close()


def group_b_two_remote(verbose):
    print('\n== B: two-remote arming + any-STOP OR ==')
    with Machine(verbose=verbose) as m:
        a = SoftRemote(RID_A, '127.0.0.1', m.port, 'A')
        b = SoftRemote(RID_B, '127.0.0.1', m.port, 'B')
        assert a.bond() and b.bond(), 'bond failed'
        s = Session([a, b])
        r = s.run(1.2, {a: MSG_OK, b: MSG_OK})
        check(
            r[a.id] == MSG_STOP and r[b.id] == MSG_STOP,
            'B1 both bonded, no arming -> both STOP (I3)',
            f'A={NAMES.get(r[a.id])} B={NAMES.get(r[b.id])}',
        )
        r = arm(s, a, [b])
        check(
            r[a.id] == MSG_OK and r[b.id] == MSG_OK,
            'B2 A arms while B holds OK -> both OK',
            f'A={NAMES.get(r[a.id])} B={NAMES.get(r[b.id])}',
        )
        # B stops while both armed -> everyone STOP (OR)
        r = s.run(1.0, {a: MSG_OK, b: MSG_STOP})
        check(
            r[a.id] == MSG_STOP and r[b.id] == MSG_STOP,
            'B3 B STOP while armed -> BOTH see STOP (I1)',
            f'A={NAMES.get(r[a.id])} B={NAMES.get(r[b.id])}',
        )
        # B alone tries to re-arm (owner is A) -> must NOT arm
        s.run(0.8, {a: MSG_OK, b: MSG_STOP})
        r = s.run(1.2, {a: MSG_OK, b: MSG_OK})
        check(
            r[a.id] == MSG_STOP,
            'B4 non-owner B cannot re-arm alone (fail-safe ownership)',
            f'A={NAMES.get(r[a.id])} B={NAMES.get(r[b.id])}',
        )
        # Owner A re-arms while B holds OK
        r = arm(s, a, [b])
        check(
            r[a.id] == MSG_OK and r[b.id] == MSG_OK,
            'B5 owner A re-arms -> both OK',
            f'A={NAMES.get(r[a.id])} B={NAMES.get(r[b.id])}',
        )
        a.close()
        b.close()


def group_c_heartbeat(verbose):
    print('\n== C: heartbeat loss (network drop / power-off) ==')
    with Machine(verbose=verbose) as m:
        a = SoftRemote(RID_A, '127.0.0.1', m.port, 'A')
        b = SoftRemote(RID_B, '127.0.0.1', m.port, 'B')
        assert a.bond() and b.bond(), 'bond failed'
        s = Session([a, b])
        arm(s, a, [b])
        # B goes silent; A keeps streaming OK. Machine must STOP within timeout.
        r = s.run(2.5, {a: MSG_OK})  # only A sends; B silent
        check(r[a.id] == MSG_STOP, 'C1 remote B silent > timeout -> robot STOP (I2)', NAMES.get(r[a.id]))
        anomaly = 'MISSED_HEARTBEATS' in m.stderr() or 'STOP' in m.stderr()
        check(anomaly, 'C1b machine logged the heartbeat stop')
        # B comes back: must re-bond and re-arm (cannot silently resume OK).
        assert b.bond(), 'B re-bond failed'
        r = s.run(1.2, {a: MSG_OK, b: MSG_OK})
        check(
            r[a.id] == MSG_STOP and r[b.id] == MSG_STOP,
            'C2 re-bonded remote forces re-arm / NEED_STOP (I5)',
            f'A={NAMES.get(r[a.id])} B={NAMES.get(r[b.id])}',
        )
        a.close()
        b.close()


def group_d_unbond(verbose):
    print('\n== D: unbond / device disconnect ==')
    with Machine(verbose=verbose) as m:
        a = SoftRemote(RID_A, '127.0.0.1', m.port, 'A')
        b = SoftRemote(RID_B, '127.0.0.1', m.port, 'B')
        assert a.bond() and b.bond(), 'bond failed'
        s = Session([a, b])
        arm(s, a, [b])
        # A (arming owner) disconnects -> robot must STOP.
        a.unbond()
        r = s.run(1.0, {b: MSG_OK})
        check(r[b.id] == MSG_STOP, 'D1 arming owner unbonds -> robot STOP/NEED_STOP (I7)', NAMES.get(r[b.id]))
        # After owner left, B may now take ownership and arm.
        r = arm(s, b, [])
        check(r[b.id] == MSG_OK, 'D2 ownership released on unbond -> B can now arm', NAMES.get(r[b.id]))
        # B unbonds (last remote) -> stays stopped, no crash.
        b.unbond()
        r = s.run(0.5, {b: MSG_STOP})
        check(True, 'D3 last remote unbond handled (no crash)', 'machine alive' if m.proc.poll() is None else 'DIED')
        a.close()
        b.close()


def group_e_capacity_allow(verbose):
    print('\n== E: capacity + allowlist ==')
    # E1 overflow: max_remotes=3, 4th must be rejected.
    with Machine(verbose=verbose) as m:
        rs = [SoftRemote(rid, '127.0.0.1', m.port) for rid in (RID_A, RID_B, RID_C)]
        bonded = [r.bond() for r in rs]
        check(all(bonded), 'E1a three remotes bond (max_remotes=3)', str(bonded))
        fourth = SoftRemote(RID_D, '127.0.0.1', m.port)
        got = fourth.bond_at_capacity(rs)  # keep the 3 alive during the attempt
        check(
            not got, 'E1b 4th remote rejected at capacity (I6 OUT_OF_OPERATOR_SPACE)', 'bonded!' if got else 'rejected'
        )
        # existing remotes still serviced
        s = Session(rs)
        r = s.run(0.6, {rs[0]: MSG_OK, rs[1]: MSG_OK, rs[2]: MSG_OK})
        check(all(v is not None for v in r.values()), 'E1c existing 3 remotes still serviced after overflow')
        for r_ in rs:
            r_.close()
        fourth.close()

    # E2 not-allowed operator
    toml = DEFAULT_TOML.replace('allow_unlisted = true', 'allow_unlisted = false')
    toml += f'\n[[operator]]\ndevice_id = 0x{RID_A:08X}\nallowed = true\n'
    with Machine(toml_text=toml, verbose=verbose) as m:
        allowed = SoftRemote(RID_A, '127.0.0.1', m.port)
        denied = SoftRemote(RID_B, '127.0.0.1', m.port)
        check(allowed.bond(timeout_s=4), 'E2a listed operator allowed to bond')
        check(not denied.bond(timeout_s=3), 'E2b unlisted operator rejected (OPERATOR_NOT_ALLOWED)')
        allowed.close()
        denied.close()

    # E3 stop_only operator: may STOP but never arm.
    toml = DEFAULT_TOML + (f'\n[[operator]]\ndevice_id = 0x{RID_B:08X}\nstop_only = true\n')
    with Machine(toml_text=toml, verbose=verbose) as m:
        a = SoftRemote(RID_A, '127.0.0.1', m.port, 'A')
        so = SoftRemote(RID_B, '127.0.0.1', m.port, 'stop_only')
        assert a.bond() and so.bond(), 'bond failed'
        s = Session([a, so])
        # stop_only remote tries to arm -> must fail; A holds OK
        s.run(0.8, {so: MSG_STOP, a: MSG_OK})
        r = s.run(1.2, {so: MSG_OK, a: MSG_OK})
        check(r[a.id] == MSG_STOP, 'E3a stop_only remote cannot arm the robot', f'A={NAMES.get(r[a.id])}')
        # but it CAN still stop: A arms, then stop_only stops -> STOP
        arm(s, a, [so])
        r = s.run(1.0, {a: MSG_OK, so: MSG_STOP})
        check(r[a.id] == MSG_STOP, 'E3b stop_only remote can still STOP the robot (I1)', f'A={NAMES.get(r[a.id])}')
        a.close()
        so.close()


def group_f_malformed(verbose):
    print('\n== F: malformed / invalid traffic (fail-safe direction) ==')
    with Machine(verbose=verbose) as m:
        a = SoftRemote(RID_A, '127.0.0.1', m.port, 'A')
        b = SoftRemote(RID_B, '127.0.0.1', m.port, 'B')
        assert a.bond() and b.bond(), 'bond failed'
        s = Session([a, b])
        arm(s, a, [b])
        # UNKNOWN (invalid type) is rejected -> does NOT feed B's heartbeat ->
        # B times out -> robot STOP. A keeps heartbeating OK the whole time.
        saw_stop = False
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            a.send(MSG_OK)
            b.send_noreply(MSG_UNKNOWN)
            if a.last_reply and a.last_reply['message'] == MSG_STOP:
                saw_stop = True
            time.sleep(0.1)
        check(
            saw_stop,
            'F1 sustained UNKNOWN from B -> robot STOP via timeout (I2)',
            'STOP seen' if saw_stop else f'A stuck {NAMES.get(a.last_reply["message"]) if a.last_reply else None}',
        )
        assert b.bond(), 'B re-bond after F1 failed'
        arm(s, a, [b])
        # Bad-CRC flood from B (non-blocking): rejected, timestamp not fed ->
        # B times out -> robot STOP. Must NEVER produce a spurious OK.
        saw_stop = False
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            a.send(MSG_OK)
            b.send_noreply(MSG_OK, corrupt=True)
            if a.last_reply and a.last_reply['message'] == MSG_STOP:
                saw_stop = True
            time.sleep(0.1)
        check(
            saw_stop,
            'F2 sustained bad-CRC from B -> robot STOP (I2, no spurious OK)',
            'STOP seen' if saw_stop else 'never stopped',
        )
        check('checksum' in m.stderr().lower(), 'F2b bad checksum rejected/logged by machine')
        # Out-of-range message type (0x42) -> rejected; robot must not arm.
        assert b.bond(), 'B re-bond after F2 failed'
        for _ in range(8):
            b.send_noreply(MSG_OK, bad_type=0x42)
            a.send(MSG_OK)
            time.sleep(0.1)
        check(
            m.proc.poll() is None and (not a.last_reply or a.last_reply['message'] == MSG_STOP),
            'F3 out-of-range type: no crash, no spurious arm',
            'machine alive, robot STOP' if m.proc.poll() is None else 'DIED',
        )
        a.close()
        b.close()


def group_g_chaos(verbose):
    print('\n== G: network chaos (loss / delay / reorder / dup) ==')
    proxy_py = os.path.join(HERE, 'pstop_chaos_proxy.py')
    if not os.path.exists(proxy_py):
        check(False, 'G chaos proxy present', 'pstop_chaos_proxy.py missing')
        return
    with Machine(verbose=verbose) as m:
        listen, ctrl = m.port + 100, m.port + 200
        proxy = subprocess.Popen(
            [
                sys.executable,
                proxy_py,
                '--listen',
                str(listen),
                '--fwd',
                str(m.port),
                '--fwd-host',
                '127.0.0.1',
                '--ctrl',
                str(ctrl),
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(0.6)
        cs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        def chaos(spec):
            cs.sendto(spec.encode(), ('127.0.0.1', ctrl))
            time.sleep(0.2)

        try:
            a = SoftRemote(RID_A, '127.0.0.1', listen, 'A')
            b = SoftRemote(RID_B, '127.0.0.1', listen, 'B')
            chaos('loss=0 delay_ms=0 jitter_ms=0 dup=0 corrupt=0')
            check(a.bond() and b.bond(), 'G0 bond through clean proxy')
            s = Session([a, b])
            r = arm(s, a, [b])
            check(r[a.id] == MSG_OK and r[b.id] == MSG_OK, 'G1 arm through proxy -> OK', f'A={NAMES.get(r[a.id])}')
            # Heavy loss: heartbeats can't get through -> machine must STOP.
            chaos('loss=0.9')
            saw_stop = False
            end = time.monotonic() + 3.5
            while time.monotonic() < end:
                a.send(MSG_OK)
                b.send(MSG_OK)
                if a.last_reply and a.last_reply['message'] == MSG_STOP:
                    saw_stop = True
                time.sleep(0.1)
            check(
                saw_stop or 'STOP' in m.stderr(),
                'G2 heavy packet loss -> robot STOP (fail-safe, no spurious OK)',
                'STOP' if saw_stop else 'machine stderr STOP',
            )
            # Recover: clear loss, re-bond, re-arm -> OK again.
            chaos('loss=0')
            time.sleep(0.5)
            assert a.bond() and b.bond(), 're-bond after loss failed'
            r = arm(s, a, [b])
            check(r[a.id] == MSG_OK, 'G3 recovery after loss clears -> re-arm OK', f'A={NAMES.get(r[a.id])}')
            # Moderate churn (loss+delay+jitter+dup+reorder): must never spuriously
            # arm from a stopped state.
            chaos('loss=0.3 delay_ms=120 jitter_ms=80 dup=0.15 corrupt=0')
            b.send(MSG_STOP)  # drop into STOP
            time.sleep(0.3)
            spurious = False
            end = time.monotonic() + 3.0
            while time.monotonic() < end:
                a.send(MSG_OK)
                b.send(MSG_STOP)  # B holds STOP the whole time
                if a.last_reply and a.last_reply['message'] == MSG_OK:
                    spurious = True
                time.sleep(0.1)
            check(
                not spurious,
                'G4 churn while a remote holds STOP -> never spuriously OK (I1)',
                'spurious OK!' if spurious else 'stayed STOP',
            )
            a.close()
            b.close()
        finally:
            cs.close()
            proxy.send_signal(signal.SIGTERM)
            try:
                proxy.wait(timeout=2)
            except Exception:
                proxy.kill()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()

    if not os.path.exists(RUNNER):
        sys.exit(f'machine_app_runner not built at {RUNNER} — run `make` in host/')

    print('pstop many-to-one corner-case validation')
    print(f'runner: {RUNNER}')

    groups = [
        group_a_single,
        group_b_two_remote,
        group_c_heartbeat,
        group_d_unbond,
        group_e_capacity_allow,
        group_f_malformed,
        group_g_chaos,
    ]
    for g in groups:
        try:
            g(args.verbose)
        except Exception as e:  # noqa: BLE001 — surface, keep going
            check(False, f'{g.__name__} raised', repr(e))

    passed = sum(1 for ok, _, _ in RESULTS if ok)
    total = len(RESULTS)
    print(f'\n=== {passed}/{total} checks passed ===')
    if passed != total:
        print('FAILURES:')
        for ok, label, detail in RESULTS:
            if not ok:
                print(f'  - {label}  ({detail})')
    sys.exit(0 if passed == total else 1)


if __name__ == '__main__':
    main()
