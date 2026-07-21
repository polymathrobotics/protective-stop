#!/usr/bin/env python3
"""Scripted pstop remote for machine-side policy testing.

Speaks the pstop_c wire protocol (40-byte messages, CRC16 poly 0x8D95,
little-endian fields, protocol @ d9f4970) well enough to bond and run
timed STOP/OK sequences against machine_app_runner — used to verify the
wrapper's min-STOP-duration arming policy without touching a chip:

    ./machine_app_runner test.toml &          # dedicated instance
    python3 pstop_test_remote.py --port 8893  # runs the test script

Test script (asserts on the machine's replies):
    1. bond, stream OK           -> machine must reply STOP (NEED_STOP)
    2. blip: STOP 200 ms, OK     -> arming must be VETOED (replies stay STOP)
    3. press: STOP 800 ms, OK    -> arming must complete (replies turn OK)
    4. blip while armed          -> robot stops; short release must NOT re-arm
    5. press again               -> re-arms

Exit code 0 = all assertions passed.
"""

import argparse
import socket
import struct
import sys
import time

PSTOP_VERSION = 0x00
MSG_OK, MSG_STOP, MSG_BOND, MSG_UNBOND = 0, 1, 2, 3
NAMES = {0: "OK", 1: "STOP", 2: "BOND", 3: "UNBOND"}
SIZE = 40

REMOTE_ID = 0x01020381          # distinct from the chip's 0x01020380
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
        "<BBQQIIIII",
        PSTOP_VERSION, message, stamp, received_stamp,
        REMOTE_ID, MACHINE_ID, HEARTBEAT_MS, counter, received_counter)
    return body + struct.pack("<H", crc16(body))


def decode(data):
    (version, message, stamp, received_stamp, sender, receiver,
     hb, counter, received_counter) = struct.unpack("<BBQQIIIII", data[:38])
    (checksum,) = struct.unpack("<H", data[38:40])
    ok = checksum == crc16(data[:38])
    return dict(message=message, stamp=stamp, counter=counter,
                received_counter=received_counter, crc_ok=ok)


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
        pkt = encode(message, self.now_ms(), self.last_rx_stamp,
                     self.counter, self.last_rx_counter)
        self.sock.send(pkt)
        try:
            reply = decode(self.sock.recv(SIZE))
        except socket.timeout:
            return None
        if reply["crc_ok"]:
            self.last_rx_counter = reply["counter"]
            self.last_rx_stamp = reply["stamp"]
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
            if reply["crc_ok"] and reply["message"] == MSG_BOND:
                self.last_rx_counter = reply["counter"]
                self.last_rx_stamp = reply["stamp"]
                return True
            time.sleep(2.5)   # let the machine age out stale state
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
    print(f"  {'PASS' if cond else 'FAIL'}: {label}")
    return bool(cond)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8893)
    args = ap.parse_args()

    r = Remote(args.host, args.port)
    ok = True

    print("1. bond + OK stream (machine must stay STOP / NEED_STOP)")
    if not r.bond():
        sys.exit("bond failed — is machine_app_runner listening?")
    last = r.stream(MSG_OK, 1.5)
    ok &= expect(last and last["message"] == MSG_STOP,
                 "OK stream answered with STOP before any arming cycle")

    print("2. blip: STOP 200 ms then OK (must be VETOED)")
    r.stream(MSG_STOP, 0.2)
    last = r.stream(MSG_OK, 1.5)
    ok &= expect(last and last["message"] == MSG_STOP,
                 "short STOP->OK did not arm (replies stay STOP)")

    print("3. press: STOP 800 ms then OK (must ARM)")
    r.stream(MSG_STOP, 0.8)
    last = r.stream(MSG_OK, 1.5)
    ok &= expect(last and last["message"] == MSG_OK,
                 "held STOP->OK armed (replies turn OK)")

    print("4. blip while armed: robot stops, short release must NOT re-arm")
    r.stream(MSG_STOP, 0.2)
    last = r.stream(MSG_OK, 1.5)
    ok &= expect(last and last["message"] == MSG_STOP,
                 "blip stopped the robot and the release was vetoed")

    print("5. press again (must re-arm)")
    r.stream(MSG_STOP, 0.8)
    last = r.stream(MSG_OK, 1.5)
    ok &= expect(last and last["message"] == MSG_OK, "re-armed after veto")

    print("ALL PASS" if ok else "FAILURES PRESENT")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
