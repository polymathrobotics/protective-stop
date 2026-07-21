#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""UDP impairment proxy for pstop chaos testing.

Sits between the chip (remote) and machine_app_runner:

    chip --UDP--> proxy:LISTEN_PORT --UDP--> machine:FWD_PORT
         <--------        <--------

Point the chip's pstop peer at this proxy (same host as the machine) and
inject controlled network pathology per direction. Impairments are set at
start and can be changed at runtime via a tiny UDP control socket, so a
test ladder can sweep conditions without re-bonding:

    echo 'loss=0.05 delay_ms=100 jitter_ms=50 dup=0.02 corrupt=0' \
        | nc -u -w1 127.0.0.1 CTRL_PORT

Impairments (each applied independently per packet, both directions unless
suffixed):  loss (P drop), delay_ms + jitter_ms (uniform, per-packet =>
natural reordering), dup (P duplicate), corrupt (P flip one random byte).
Direction-specific overrides: loss_up (chip->machine), loss_down
(machine->chip), same for delay_ms/jitter_ms/dup/corrupt.

Stats printed every 10 s and on every control change:
    fwd/ret counts, dropped, duped, corrupted, in-flight delayed.

Usage:
    python3 pstop_chaos_proxy.py [--listen 8891] [--fwd 8890]
                                 [--ctrl 8892] [--fwd-host 127.0.0.1]
"""

import argparse
import random
import socket
import threading
import time

PARAMS = ['loss', 'delay_ms', 'jitter_ms', 'dup', 'corrupt']


class Impairments:
    def __init__(self):
        self.lock = threading.Lock()
        self.base = {p: 0.0 for p in PARAMS}
        self.up = {}  # chip -> machine overrides
        self.down = {}  # machine -> chip overrides

    def set_from_line(self, line):
        with self.lock:
            for tok in line.split():
                if '=' not in tok:
                    continue
                k, v = tok.split('=', 1)
                try:
                    v = float(v)
                except ValueError:
                    continue
                if k in PARAMS:
                    self.base[k] = v
                elif k.endswith('_up') and k[:-3] in PARAMS:
                    self.up[k[:-3]] = v
                elif k.endswith('_down') and k[:-5] in PARAMS:
                    self.down[k[:-5]] = v
                elif k == 'clear':
                    self.base = {p: 0.0 for p in PARAMS}
                    self.up.clear()
                    self.down.clear()

    def get(self, direction):
        with self.lock:
            eff = dict(self.base)
            eff.update(self.up if direction == 'up' else self.down)
            return eff


class Stats:
    def __init__(self):
        self.lock = threading.Lock()
        self.c = {
            k: 0 for k in ('fwd', 'ret', 'drop_up', 'drop_down', 'dup_up', 'dup_down', 'corrupt_up', 'corrupt_down')
        }

    def bump(self, key):
        with self.lock:
            self.c[key] += 1

    def snapshot(self):
        with self.lock:
            return dict(self.c)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--listen', type=int, default=8891)
    ap.add_argument('--fwd', type=int, default=8890)
    ap.add_argument('--fwd-host', default='127.0.0.1')
    ap.add_argument('--ctrl', type=int, default=8892)
    args = ap.parse_args()

    imp = Impairments()
    stats = Stats()

    front = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # chip side
    front.bind(('0.0.0.0', args.listen))
    back = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # machine side
    back.connect((args.fwd_host, args.fwd))

    ctrl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctrl.bind(('127.0.0.1', args.ctrl))

    chip_addr = [None]  # learned from first inbound packet

    def send_later(sock_send, data, delay_s):
        t = threading.Timer(delay_s, sock_send, args=(data,))
        t.daemon = True
        t.start()

    def process(data, direction, sock_send):
        """Apply impairments; sock_send(data) transmits one datagram."""
        e = imp.get(direction)
        sfx = '_up' if direction == 'up' else '_down'
        if random.random() < e['loss']:
            stats.bump('drop' + sfx)
            return
        if e['corrupt'] > 0 and random.random() < e['corrupt']:
            b = bytearray(data)
            b[random.randrange(len(b))] ^= 1 << random.randrange(8)
            data = bytes(b)
            stats.bump('corrupt' + sfx)
        copies = 1
        if e['dup'] > 0 and random.random() < e['dup']:
            copies = 2
            stats.bump('dup' + sfx)
        for _ in range(copies):
            d = e['delay_ms'] + (random.uniform(0, e['jitter_ms']) if e['jitter_ms'] else 0)
            if d > 0:
                send_later(sock_send, data, d / 1000.0)
            else:
                sock_send(data)

    def front_loop():  # chip -> machine
        while True:
            data, addr = front.recvfrom(2048)
            chip_addr[0] = addr
            stats.bump('fwd')
            process(data, 'up', back.send)

    def back_loop():  # machine -> chip
        while True:
            data = back.recv(2048)
            if chip_addr[0] is None:
                continue
            stats.bump('ret')
            a = chip_addr[0]
            process(data, 'down', lambda d, a=a: front.sendto(d, a))

    def ctrl_loop():
        while True:
            line, _ = ctrl.recvfrom(512)
            imp.set_from_line(line.decode(errors='replace'))
            print(
                f'[ctrl] {line.decode(errors="replace").strip()} -> base={imp.base} up={imp.up} down={imp.down}',
                flush=True,
            )

    for fn in (front_loop, back_loop, ctrl_loop):
        threading.Thread(target=fn, daemon=True).start()

    print(f'chaos proxy: chip->:{args.listen} => {args.fwd_host}:{args.fwd}, ctrl 127.0.0.1:{args.ctrl}', flush=True)
    while True:
        time.sleep(10)
        print(f'[stats] {stats.snapshot()}', flush=True)


if __name__ == '__main__':
    main()
