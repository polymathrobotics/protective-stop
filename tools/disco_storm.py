#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""DISCO-storm injector — synthetic large-tailnet load for peer-scaling tests.

Fires well-formed but UNKNOWN-key DISCO ping packets at the chip's WG UDP
port (51820) over the LAN. Each one, absent a same-key peer, drives the
chip through the exact worst-case path the 100+-peer work bounds: the
disco allowlist gate -> the 2026-07-20 unknown-key admission -> one NaCl
box-open (~30 ms X25519, recomputed every time). This is the load a real
100-peer tailnet's magicsock probing produces; a physical 100-peer
tailnet isn't available on the bench, so we generate the pressure directly.

The chip should ABSORB it: ML_DISCO_OPENS_PER_PASS budgets box-opens per
wg_mgr pass and the priority peer's key is exempt, so the concurrent 10 Hz
pstop link must keep zero heartbeat-timeout STOPs throughout. Run the pstop
machine + a soak alongside and watch for STOPs / rebonds.

    python3 disco_storm.py --host 192.168.107.189 --pps 40 --secs 300

The packets can't decrypt (random keys) so they're dropped after the
box-open — they never enter the peer table. Purpose is CPU load, not
protocol conformance.
"""

import argparse
import os
import socket
import time

DISCO_MAGIC = b'TS\x9a\x8b\xc4\xd5'  # 6-byte magic (matches DISCO_MAGIC)


def make_ping(seq: int) -> bytes:
    # magic(6) + sender disco pubkey(32) + nonce(24) + ciphertext.
    # Random key+nonce+body: format-valid, decrypt-invalid = one box-open.
    key = os.urandom(32)
    nonce = os.urandom(24)
    body = os.urandom(40)  # would-be encrypted ping payload
    return DISCO_MAGIC + key + nonce + body


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', required=True)
    ap.add_argument('--port', type=int, default=51820)
    ap.add_argument('--pps', type=int, default=40, help='packets per second')
    ap.add_argument('--secs', type=int, default=300)
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.port)
    interval = 1.0 / max(args.pps, 1)
    end = time.monotonic() + args.secs
    n = 0
    while time.monotonic() < end:
        s.sendto(make_ping(n), dest)
        n += 1
        if n % args.pps == 0:
            print(f'sent {n} disco pings ({args.pps}/s)', flush=True)
        time.sleep(interval)
    print(f'done: {n} packets over {args.secs}s')


if __name__ == '__main__':
    main()
