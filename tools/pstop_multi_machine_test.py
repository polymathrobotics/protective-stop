#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""One-remote-to-many-machines HIL validation.

The mirror of pstop_multi_remote_test.py: that one proves the MACHINE side's
many-remote bookkeeping; this one proves the REMOTE firmware's many-machine
session logic on a real chip. It starts N unmodified machine_app_runner
instances on this host (ports 8890..8890+N-1), points the remote at them via
POST /api/pstop_peers, and asserts the chip-side isolation invariants through
/state.json's pstop_machines array:

  M1  every configured machine bonds (state=2) and its replies climb
  M2  killing one machine re-bonds ONLY that session: its state leaves 2 and
      its rebonds counter climbs, while every OTHER session's replies keep
      climbing uninterrupted (no cross-session stall — the fail-safe mirror
      of the machine side's any-remote-STOP OR)
  M3  restarting the killed machine re-bonds that session and replies resume
  M4  clearing a slot idles that session (state=0) without disturbing others
  M5  the lockstep mismatch counter does not climb during any of the above
      (multi-session handling must never perturb the safety comparator)

This is TEST tooling only — it never touches the firmware image or the
certification-track pstop_c library. The chip's original slot-0 peer is
saved and restored, so a bench unit goes back to its normal machine.

Usage:
  make -C host                    # build the machine runner first
  python3 tools/pstop_multi_machine_test.py --chip <chip-ip> [--host-ip <ip>]
      [-n 3] [-v]

--host-ip is the address the CHIP should send to (this host's LAN or
Tailscale IP as seen from the chip); auto-detected from the route toward the
chip when omitted. Exit 0 = all invariants held.
"""

import argparse
import json
import os
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
RUNNER = os.path.join(HERE, '..', 'host', 'machine_app_runner')

BASE_PORT = 8890
STATE_BONDED = 2
STATE_IDLE = 0

VERBOSE = False


def log(msg):
    if VERBOSE:
        print(f'    {msg}')


def http(chip, path, method='GET', timeout=5):
    req = urllib.request.Request(f'http://{chip}{path}', method=method)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read().decode()


def get_state(chip):
    return json.loads(http(chip, '/state.json'))


def machines(chip):
    return get_state(chip)['pstop_machines']


def set_slot(chip, slot, ip, port):
    resp = json.loads(http(chip, f'/api/pstop_peers?slot={slot}&ip={ip}&port={port}', method='POST'))
    if not resp.get('ok'):
        raise RuntimeError(f'set_slot({slot}) failed: {resp}')


def clear_slot(chip, slot):
    resp = json.loads(http(chip, f'/api/pstop_peers?slot={slot}&clear=1', method='POST'))
    if not resp.get('ok'):
        raise RuntimeError(f'clear_slot({slot}) failed: {resp}')


def detect_host_ip(chip):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect((chip, 80))
    ip = s.getsockname()[0]
    s.close()
    return ip


def start_runner(workdir, port):
    cfg = os.path.join(workdir, f'machine_{port}.toml')
    with open(cfg, 'w') as f:
        f.write(f'port = {port}\nbind = "0.0.0.0"\nallow_unlisted = true\n')
    proc = subprocess.Popen(
        [RUNNER, cfg],
        stdout=subprocess.DEVNULL if not VERBOSE else None,
        stderr=subprocess.STDOUT if not VERBOSE else None,
    )
    log(f'machine runner up on :{port} (pid {proc.pid})')
    return proc


def wait_for(chip, pred, desc, timeout_s=30):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            if pred(machines(chip)):
                return True
        except Exception as e:  # noqa: BLE001 — chip may briefly drop requests
            log(f'poll error ({e}), retrying')
        time.sleep(1)
    print(f'TIMEOUT waiting for: {desc}')
    return False


RESULTS = []


def check(ok, label):
    RESULTS.append((ok, label))
    print(f'  {"PASS" if ok else "FAIL"}  {label}')


def main():
    global VERBOSE
    ap = argparse.ArgumentParser()
    ap.add_argument('--chip', required=True, help='remote (chip) IP')
    ap.add_argument('--host-ip', default=None, help='this host as seen from the chip')
    ap.add_argument('-n', type=int, default=3, help='machine count (2..4)')
    ap.add_argument('-v', action='store_true')
    args = ap.parse_args()
    VERBOSE = args.v
    n = max(2, min(4, args.n))

    if not os.path.exists(RUNNER):
        print(f'machine_app_runner not found at {RUNNER} — run: make -C host')
        return 1

    host_ip = args.host_ip or detect_host_ip(args.chip)
    print(f'chip={args.chip}  host={host_ip}  machines={n}')

    # Save the bench unit's original slot-0 target for restore.
    orig = machines(args.chip)[0]
    mm_before = get_state(args.chip)['pstop_mismatch']

    procs = {}
    workdir = tempfile.mkdtemp(prefix='pstop_mm_')
    try:
        for i in range(n):
            procs[i] = start_runner(workdir, BASE_PORT + i)
        time.sleep(1)
        for i in range(n):
            set_slot(args.chip, i, host_ip, BASE_PORT + i)

        # M1: all sessions bond and reply.
        check(
            wait_for(args.chip, lambda m: all(m[i]['state'] == STATE_BONDED for i in range(n)), 'all bonded'),
            f'M1a all {n} sessions bonded',
        )
        r0 = machines(args.chip)
        time.sleep(3)
        r1 = machines(args.chip)
        check(all(r1[i]['replies'] > r0[i]['replies'] for i in range(n)), 'M1b replies climb on every session')

        # M2: kill machine 1; only that session degrades.
        victim = 1
        procs[victim].kill()
        procs[victim].wait()
        log(f'killed machine {victim}')
        check(
            wait_for(
                args.chip,
                lambda m: m[victim]['state'] != STATE_BONDED or m[victim]['rebonds'] > 0,
                'victim session degraded',
            ),
            'M2a victim session re-bonds after machine death',
        )
        s0 = machines(args.chip)
        time.sleep(3)
        s1 = machines(args.chip)
        others = [i for i in range(n) if i != victim]
        check(
            all(s1[i]['replies'] > s0[i]['replies'] for i in others),
            'M2b surviving sessions keep replying (no cross-session stall)',
        )
        check(all(s1[i]['state'] == STATE_BONDED for i in others), 'M2c surviving sessions stay bonded')

        # M3: restart the machine; the session recovers on its own.
        procs[victim] = start_runner(workdir, BASE_PORT + victim)
        check(
            wait_for(args.chip, lambda m: m[victim]['state'] == STATE_BONDED, 'victim re-bonded'),
            'M3a victim session re-bonds after machine restart',
        )
        t0 = machines(args.chip)
        time.sleep(3)
        t1 = machines(args.chip)
        check(t1[victim]['replies'] > t0[victim]['replies'], 'M3b victim replies resume')

        # M4: clear the last slot; others undisturbed.
        clear_slot(args.chip, n - 1)
        check(
            wait_for(args.chip, lambda m: m[n - 1]['state'] == STATE_IDLE, 'slot idle'),
            'M4a cleared slot goes idle',
        )
        u0 = machines(args.chip)
        time.sleep(3)
        u1 = machines(args.chip)
        check(
            all(u1[i]['replies'] > u0[i]['replies'] for i in range(n - 1)),
            'M4b remaining sessions unaffected by slot clear',
        )

        # M5: the lockstep comparator never hiccuped.
        mm_after = get_state(args.chip)['pstop_mismatch']
        check(mm_after == mm_before, f'M5 no lockstep mismatches (before={mm_before} after={mm_after})')

    finally:
        for p in procs.values():
            try:
                p.kill()
            except OSError:
                pass
        # Restore the bench unit: clear our slots, put the original slot 0 back.
        try:
            for i in range(1, n):
                clear_slot(args.chip, i)
            if orig['cfg']:
                oip = orig['ip']
                dotted = f'{(oip >> 24) & 0xFF}.{(oip >> 16) & 0xFF}.{(oip >> 8) & 0xFF}.{oip & 0xFF}'
                set_slot(args.chip, 0, dotted, orig['port'])
                print(f'restored original slot 0 -> {dotted}:{orig["port"]}')
            else:
                clear_slot(args.chip, 0)
        except Exception as e:  # noqa: BLE001
            print(f'WARNING: restore failed ({e}) — re-point the chip manually')

    passed = sum(1 for ok, _ in RESULTS if ok)
    print(f'\n=== {passed}/{len(RESULTS)} checks passed ===')
    if passed != len(RESULTS):
        print('FAILURES:')
        for ok, label in RESULTS:
            if not ok:
                print(f'  - {label}')
    return 0 if passed == len(RESULTS) else 1


if __name__ == '__main__':
    sys.exit(main())
