#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Scripted pstop remote for machine-side policy testing.

Speaks the pstop_c v2 wire protocol (48-byte messages, CRC16 poly 0x8D95,
little-endian fields, far-Hamming message codewords, aux channel in
padding1) well enough to bond and run timed STOP/OK sequences against a
machine — used to verify the min-STOP-duration arming policy without
touching a chip. Works against host/machine_app_runner and the ROS 2
machine_bridge_node alike (same pstop_c, same UDP port):

    ./machine_app_runner test.toml &          # dedicated instance
    python3 pstop_test_remote.py --port 8893  # runs the test script

Arming needs BOTH gates open: this script announces role OPERATOR in its
aux channel (--role stop_only to test the refusal), and the machine must
list REMOTE_ID (0x01020381 = 16909185) as an operator — or run with
default_stop_only = false, as host/machine.toml does for the bench.

Test script (asserts on the machine's replies; library-native min-delay
semantics from pstop_c #59 — an OK sooner than delay_between_stop_ms after
the cycle-opening STOP is refused, and the SAME episode completes once the
delay elapses):
    1. bond, stream OK           -> machine must reply STOP (NEED_STOP)
    2. blip: STOP 200 ms, OK     -> immediate OK refused; steady OKs arm
                                    after the min delay
    3. press: STOP 800 ms, OK    -> arms immediately on release
    4. blip while armed          -> robot stops; immediate OK refused;
                                    steady OKs self-re-arm after the delay
    5. press-and-hold, release   -> re-arms

Exit code 0 = all assertions passed.
"""

import argparse
import socket
import struct
import sys
import time

# Wire protocol — MUST track pstop_c (config.h PSTOP_VERSION / PSTOP_MESSAGE_SIZE,
# pstop_msg.h codewords) and common/pstop_aux_channel.h (padding1 layout).
PSTOP_VERSION = 0x02
MSG_OK, MSG_STOP, MSG_BOND, MSG_UNBOND = 0x55, 0x92, 0xAD, 0x6A
NAMES = {0x55: 'OK', 0x92: 'STOP', 0xAD: 'BOND', 0x6A: 'UNBOND', 0x0F: 'UNKNOWN'}
SIZE = 48
FMT = '<BBQQIIIIIII'  # 46 bytes + u16 CRC
AUX_VERSION = 0x01
ROLE_STOP_ONLY, ROLE_OPERATOR = 1, 2
ROLES = {'stop_only': ROLE_STOP_ONLY, 'operator': ROLE_OPERATOR}
ROLE = ROLE_OPERATOR  # set from --role

REMOTE_ID = 0x01020381  # distinct from the chip's 0x01020380
MACHINE_ID = 0x01020304
HEARTBEAT_MS = 1000


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8D95) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode(message, stamp, received_stamp, counter, received_counter):
    body = struct.pack(
        FMT,
        PSTOP_VERSION,
        message,
        stamp,
        received_stamp,
        REMOTE_ID,
        MACHINE_ID,
        HEARTBEAT_MS,
        counter,
        received_counter,
        AUX_VERSION | (ROLE << 8),  # padding1 = aux channel: version + self-role
        0,  # padding2
    )
    return body + struct.pack('<H', crc16(body))


def decode(data):
    if len(data) != SIZE:
        return dict(message=None, stamp=0, counter=0, received_counter=0, crc_ok=False)
    (
        version,
        message,
        stamp,
        received_stamp,
        sender,
        receiver,
        hb,
        counter,
        received_counter,
        _padding1,
        _padding2,
    ) = struct.unpack(FMT, data[:46])
    (checksum,) = struct.unpack('<H', data[46:48])
    ok = checksum == crc16(data[:46])
    return dict(
        message=message,
        stamp=stamp,
        counter=counter,
        received_counter=received_counter,
        crc_ok=ok,
    )


class Remote:
    def __init__(self, host, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect((host, port))
        self.sock.settimeout(0.5)
        self.counter = 0
        self.last_rx_counter = 0
        self.last_rx_stamp = 0

    def now_ms(self):
        return time.monotonic_ns() // 1_000_000

    def xfer(self, message):
        """Send one message, return the machine's reply dict or None."""
        self.counter += 1
        pkt = encode(
            message,
            self.now_ms(),
            self.last_rx_stamp,
            self.counter,
            self.last_rx_counter,
        )
        self.sock.send(pkt)
        try:
            reply = decode(self.sock.recv(SIZE))
        except socket.timeout:
            return None
        if reply['crc_ok']:
            self.last_rx_counter = reply['counter']
            self.last_rx_stamp = reply['stamp']
        return reply

    def bond(self):
        self.counter = 0
        self.last_rx_counter = 0
        self.last_rx_stamp = 0
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            self.counter = 0
            pkt = encode(MSG_BOND, self.now_ms(), 0, 1, 0)
            self.counter = 1
            self.sock.send(pkt)
            try:
                reply = decode(self.sock.recv(SIZE))
            except socket.timeout:
                continue
            if reply['crc_ok'] and reply['message'] == MSG_BOND:
                self.last_rx_counter = reply['counter']
                self.last_rx_stamp = reply['stamp']
                return True
            time.sleep(2.5)  # let the machine age out stale state
        return False

    def stream(self, message, duration_s):
        """Send `message` at 10 Hz for duration; return last reply."""
        last = None
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            r = self.xfer(message)
            if r is not None:
                last = r
            time.sleep(0.1)
        return last


def expect(cond, label):
    print(f'  {"PASS" if cond else "FAIL"}: {label}')
    return bool(cond)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=8893)
    ap.add_argument(
        '--role',
        choices=sorted(ROLES),
        default='operator',
        help='self-role announced in the aux channel (default operator; stop_only must never arm)',
    )
    args = ap.parse_args()
    global ROLE
    ROLE = ROLES[args.role]

    r = Remote(args.host, args.port)
    ok = True

    print('1. bond + OK stream (machine must stay STOP / NEED_STOP)')
    if not r.bond():
        sys.exit('bond failed — is machine_app_runner listening?')
    last = r.stream(MSG_OK, 1.5)
    ok &= expect(
        last and last['message'] == MSG_STOP,
        'OK stream answered with STOP before any arming cycle',
    )

    if ROLE == ROLE_STOP_ONLY:
        print('2s. stop-only remote: a full press-and-release must NOT arm')
        r.stream(MSG_STOP, 0.8)
        last = r.stream(MSG_OK, 1.5)
        ok &= expect(
            last and last['message'] == MSG_STOP,
            'held STOP->OK from a stop_only remote stays STOP (re-arm refused)',
        )
        print('ALL PASS' if ok else 'FAILURES PRESENT')
        sys.exit(0 if ok else 1)

    print('2. blip: STOP 200 ms then OK — early OK refused, arms after min delay')
    r.stream(MSG_STOP, 0.2)
    first = r.xfer(MSG_OK)
    ok &= expect(
        first and first['message'] == MSG_STOP,
        'OK 200 ms after STOP refused (library min delay)',
    )
    last = r.stream(MSG_OK, 1.5)
    ok &= expect(
        last and last['message'] == MSG_OK,
        'steady OK stream arms once the min delay elapses',
    )

    print('3. press: STOP 800 ms then OK (must ARM immediately)')
    r.stream(MSG_STOP, 0.8)
    last = r.stream(MSG_OK, 1.5)
    ok &= expect(last and last['message'] == MSG_OK, 'held STOP->OK armed (replies turn OK)')

    print('4. blip while armed: robot stops; early release refused, self-re-arms')
    r.stream(MSG_STOP, 0.2)
    first = r.xfer(MSG_OK)
    ok &= expect(
        first and first['message'] == MSG_STOP,
        'blip stopped the robot; immediate OK refused',
    )
    last = r.stream(MSG_OK, 1.5)
    ok &= expect(last and last['message'] == MSG_OK, 'steady OK re-arms after the min delay')

    print('5. press-and-hold then release (normal gesture must still arm)')
    r.stream(MSG_STOP, 0.8)
    last = r.stream(MSG_OK, 1.5)
    ok &= expect(last and last['message'] == MSG_OK, 're-armed after a held press')

    print('ALL PASS' if ok else 'FAILURES PRESENT')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
