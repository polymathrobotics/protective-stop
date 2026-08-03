#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Exhaustive STOP / RESET / RECONNECT battery for the pstop machine.

Drives the wire protocol against a dedicated machine_app_runner instance and
asserts the machine's reply at every step. Covers the arming state machine
(pstop_c #59 min-delay + ownership), heartbeat-timeout STOP, and — the focus
of this run — the reconnect-after-loss paths that govern "reset not
connecting". Every scenario is independent (fresh bond) so one failure can't
cascade.

Exit 0 = all scenarios pass.
"""

import atexit
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time

PSTOP_VERSION = 0x00
MSG_OK, MSG_STOP, MSG_BOND, MSG_UNBOND = 0, 1, 2, 3
NAMES = {0: "OK", 1: "STOP", 2: "BOND", 3: "UNBOND"}
SIZE = 40
MACHINE_ID = 0x01020304
HEARTBEAT_MS = 400  # matches production machine.toml
MIN_STOP_MS = 500   # delay_between_stop_ms
MAX_MISSED = 3      # timeout = hb * max_missed = 1.2 s

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8899

RUNNER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "host", "machine_app_runner")


def spawn_runner(port):
    """Spawn our OWN machine_app_runner on `port` with the machine_device_id +
    timing this battery targets, and tear it down on exit. Self-contained: the
    battery no longer silently fails when no external runner happens to be up
    on the right port with the right id."""
    if not os.path.exists(RUNNER):
        sys.exit(f"machine_app_runner not built at {RUNNER} — run `make -C host` first")
    cfg = tempfile.NamedTemporaryFile("w", suffix=".toml", delete=False)
    cfg.write(
        f"[machine]\nmachine_device_id = 0x{MACHINE_ID:08X}\n"
        f"[limits]\nmax_missed_heartbeats = {MAX_MISSED}\n"
        f"[policy]\nallow_unlisted = true\n"
        f"default_heartbeat_ms = {HEARTBEAT_MS}\nmin_stop_ms = {MIN_STOP_MS}\n")
    cfg.close()
    p = subprocess.Popen([RUNNER, cfg.name, str(port)],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _cleanup():
        p.terminate()
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()
        try:
            os.unlink(cfg.name)
        except OSError:
            pass
    atexit.register(_cleanup)
    time.sleep(1.0)  # let it bind
    if p.poll() is not None:
        sys.exit(f"machine_app_runner exited immediately — is port {port} already in use? "
                 f"(kill stray runners, or pass a free port)")
    return p


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8D95) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


class Remote:
    """One emulated pstop remote. Distinct id so scenarios don't collide."""

    _next_id = 0x01A00001

    def __init__(self, host="127.0.0.1", port=PORT, rid=None):
        if rid is None:
            rid = Remote._next_id
            Remote._next_id += 1
        self.id = rid
        self.host, self.port = host, port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect((host, port))
        self.sock.settimeout(0.5)
        self.counter = 0
        self.last_rx_counter = 0
        self.last_rx_stamp = 0
        self.last_tx_stamp = 0

    def now_ms(self):
        # Strictly monotonic per-remote send stamp. now_ms() is ms-resolution,
        # so two messages emitted in the same millisecond (e.g. BOND then the
        # first STOP of an immediate arming gesture) would carry an identical
        # stamp and pstop_c would REJECT the second as out-of-order/replay
        # (MSG_OUT_OF_ORDER) — correctly, since real 10 Hz hardware never emits
        # two frames inside one ms. Bump to last+1 to mirror that guarantee.
        stamp = time.monotonic_ns() // 1_000_000
        if stamp <= self.last_tx_stamp:
            stamp = self.last_tx_stamp + 1
        self.last_tx_stamp = stamp
        return stamp

    def _encode(self, message):
        self.counter += 1
        body = struct.pack(
            "<BBQQIIIII", PSTOP_VERSION, message, self.now_ms(),
            self.last_rx_stamp, self.id, MACHINE_ID, HEARTBEAT_MS,
            self.counter, self.last_rx_counter)
        return body + struct.pack("<H", crc16(body))

    def _decode(self, data):
        (ver, message, stamp, rstamp, sender, receiver, hb, counter,
         rcounter) = struct.unpack("<BBQQIIIII", data[:38])
        (chk,) = struct.unpack("<H", data[38:40])
        return dict(message=message, stamp=stamp, counter=counter,
                    crc_ok=(chk == crc16(data[:38])))

    def xfer(self, message):
        try:
            self.sock.send(self._encode(message))
            reply = self._decode(self.sock.recv(SIZE))
        except (socket.timeout, OSError):
            return None
        if reply["crc_ok"]:
            self.last_rx_counter = reply["counter"]
            self.last_rx_stamp = reply["stamp"]  # MUST echo back or msgs read stale
        return reply

    def bond(self, timeout=15):
        self.counter = self.last_rx_counter = self.last_rx_stamp = 0
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.counter = 1
            try:
                self.sock.send(self._encode(MSG_BOND))
                r = self._decode(self.sock.recv(SIZE))
            except (socket.timeout, OSError):
                continue
            if r["crc_ok"] and r["message"] == MSG_BOND:
                self.last_rx_counter = r["counter"]
                self.last_rx_stamp = r["stamp"]
                return True
            time.sleep(2.5)
        return False

    def stream(self, message, seconds, hz=10):
        """Send `message` at hz for `seconds`; return the last reply msg."""
        last = None
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            r = self.xfer(message)
            if r is not None:
                last = r["message"]
            time.sleep(1.0 / hz)
        return last

    def close(self):
        # UNBOND so the machine drops us immediately — otherwise our
        # heartbeat times out ~1.2 s later and, if that lands during the
        # NEXT scenario's arming gesture, correctly resets its cycle
        # (real many-remote fail-safe behavior; see scenario 12).
        try:
            self.xfer(MSG_UNBOND)
        except Exception:
            pass
        self.sock.close()
        time.sleep(0.1)


PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}: {label}")
    return bool(cond)


def arm(r, keepalive=None):
    """Valid arming gesture: hold STOP > min, release OK, confirm OK.
    keepalive: another Remote whose heartbeat must be kept fresh (its current
    message) so it doesn't time out and reset the cycle mid-gesture."""
    end = time.monotonic() + (MIN_STOP_MS + 300) / 1000.0
    while time.monotonic() < end:
        r.xfer(MSG_STOP)
        if keepalive:
            keepalive.xfer(keepalive_msg[0])
        time.sleep(0.1)
    last = None
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        rp = r.xfer(MSG_OK)
        if rp is not None:
            last = rp["message"]
        if keepalive:
            keepalive.xfer(keepalive_msg[0])
        time.sleep(0.1)
    return last == MSG_OK


keepalive_msg = [MSG_OK]  # what the keepalive remote should send


def scenario(name):
    print(f"\n== {name}")


def main():
    print(f"pstop stop/reset battery vs machine on 127.0.0.1:{PORT}\n"
          f"(hb={HEARTBEAT_MS}ms min_stop={MIN_STOP_MS}ms max_missed={MAX_MISSED} "
          f"=> timeout {HEARTBEAT_MS*MAX_MISSED}ms)")

    spawn_runner(PORT)   # self-contained: bring up our own machine to drive

    # --- 1. baseline bond + need-stop ------------------------------------
    scenario("1. fresh bond, stream OK -> machine must hold STOP (NEED_STOP)")
    r = Remote()
    check(r.bond(), "bond succeeds")
    check(r.stream(MSG_OK, 1.5) == MSG_STOP, "OK before any STOP is answered STOP")
    r.close()

    # --- 2. valid arming gesture ----------------------------------------
    scenario("2. hold STOP > min, release OK -> ARMS")
    r = Remote()
    r.bond()
    check(arm(r), "held STOP->OK arms (reply OK)")
    r.close()

    # --- 3. min-delay refusal -------------------------------------------
    scenario("3. STOP 200ms then OK -> refused (library min delay), then arms")
    r = Remote()
    r.bond()
    r.stream(MSG_STOP, 0.2)
    first = r.xfer(MSG_OK)
    check(first and first["message"] == MSG_STOP, "immediate OK after short STOP refused")
    check(r.stream(MSG_OK, 2.0) == MSG_OK, "steady OK arms once min-delay elapses")
    r.close()

    # --- 4. re-STOP after armed then re-arm ------------------------------
    scenario("4. armed -> STOP -> stops -> re-arm")
    r = Remote()
    r.bond()
    check(arm(r), "armed")
    check(r.stream(MSG_STOP, 0.6) == MSG_STOP, "STOP after armed -> STOP")
    check(arm(r), "re-arms after a full STOP hold")
    r.close()

    # --- 5. rapid stop/reset cycles (state-machine stress) --------------
    scenario("5. 10 rapid valid stop/reset cycles all arm")
    r = Remote()
    r.bond()
    ok = 0
    for _ in range(10):
        if arm(r):
            ok += 1
        r.stream(MSG_STOP, 0.6)  # stop between cycles
    check(ok == 10, f"all 10 cycles armed ({ok}/10)")
    r.close()

    # --- 6. RECONNECT after heartbeat-timeout while ARMED ---------------
    scenario("6. armed, go silent > timeout (machine STOPs+clears), reconnect, re-arm")
    r = Remote()
    r.bond()
    check(arm(r), "armed")
    time.sleep((HEARTBEAT_MS * MAX_MISSED) / 1000.0 + 1.0)  # silence past timeout
    # a stale (non-BOND) message after being cleared must be dropped/ignored
    stale = r.xfer(MSG_OK)
    check(stale is None or stale["message"] in (MSG_STOP, MSG_UNBOND),
          "post-timeout OK is not honored (machine cleared us)")
    check(r.bond(), "fresh BOND reconnects")
    check(arm(r), "re-arms after reconnect  <-- the reported failure path")
    r.close()

    # --- 7. RECONNECT after silence while STOPPED -----------------------
    scenario("7. STOPPED, go silent, reconnect -> still safe, arm works")
    r = Remote()
    r.bond()
    r.stream(MSG_STOP, 0.6)
    time.sleep((HEARTBEAT_MS * MAX_MISSED) / 1000.0 + 1.0)
    check(r.bond(), "fresh BOND reconnects from STOPPED")
    check(arm(r), "arms after reconnect-from-stop")
    r.close()

    # --- 8. connection loss DURING the arming gesture -------------------
    scenario("8. drop mid-gesture (after STOP, before OK completes), reconnect, arm")
    r = Remote()
    r.bond()
    r.stream(MSG_STOP, 0.3)  # partial gesture
    time.sleep((HEARTBEAT_MS * MAX_MISSED) / 1000.0 + 1.0)  # drop
    check(r.bond(), "re-bond after mid-gesture drop")
    check(arm(r), "arms cleanly after mid-gesture drop")
    r.close()

    # --- 9. new remote identity each reconnect (reboot simulation) ------
    scenario("9. 5 sequential 'reboots' (new bond each) all reach OK")
    ok = 0
    for _ in range(5):
        r = Remote(rid=None)  # fresh id == a rebooted/replaced unit
        if r.bond() and arm(r):
            ok += 1
        r.close()
    check(ok == 5, f"all 5 fresh-identity bonds armed ({ok}/5)")

    # --- 10. UNBOND then reconnect --------------------------------------
    scenario("10. explicit UNBOND, then re-bond and arm")
    r = Remote()
    r.bond()
    arm(r)
    r.xfer(MSG_UNBOND)
    check(r.bond(), "re-bond after UNBOND")
    check(arm(r), "arms after UNBOND/re-bond")
    r.close()

    # --- 11. reconnect storm --------------------------------------------
    scenario("11. reconnect storm: 8 back-to-back bonds, final one must arm")
    ok = True
    for i in range(8):
        r = Remote()
        if not r.bond(timeout=8):
            ok = False
        r.close()
    r = Remote()
    check(r.bond() and arm(r), "machine still bonds+arms after a reconnect storm")
    check(ok, "every bond in the storm succeeded")
    r.close()

    # (multi-remote ownership + OR-stop is covered by
    # tools/pstop_multi_remote_test.py, 34/34.)

    print(f"\n{'='*50}\nRESULT: {len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        print("FAILURES:")
        for f in FAIL:
            print("  -", f)
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()
