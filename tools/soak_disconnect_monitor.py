#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Long-run remote <-> machine connectivity / disconnect soak monitor.

Polls one or more pstop *remotes* and one or more *machines* over their
`/state.json` (and, when admin creds are given, `/admin/api/monitor`) HTTP
endpoints on a fixed cadence and records every connectivity disruption with the
full multi-unit context needed to tell the causes apart. It is a diagnostic /
acceptance-validation tool for benches and staging tailnets — it is NOT part of
the shipped firmware and never writes to a device.

WHAT IT DETECTS (per remote, sample-over-sample, from the real /state.json shape)
  * peer bond DROPPED   -- a remote's machine slot `state` fell from 2 (bonded)
  * rebonds++           -- the remote re-established a bond (transient loss)
  * ml_reconnects++     -- a control-plane (Tailscale/DERP) reconnect
  * eth_recovery        -- the Ethernet (W5500) watchdog recovered the link
  * reply STALL         -- the slot keeps `sent` advancing but `replies` does not
  * state.json UNREACHABLE -- the unit's HTTP endpoint stopped answering

Each event is written to events.log with a timestamp, the trigger(s), the
remote's send-fail cause codes (pstop_sf_route / pstop_sf_txdrv / pstop_sf_errno),
its net context (ml_state, ml_reconnects, eth_*), and every machine's view of its
bonded_remotes (state / age_ms / rtt_ms / wg_direct) so a disconnect can be
localized to the route, the WireGuard underlay, or the peer cap. A wide
samples.csv row is written every poll, and a finer telemetry.csv (ml_reconnects,
per-slot rtt/wg_direct, rebonds, and — with admin creds — DERP rehome counters)
is written on its own cadence.

DESIGN NOTES
  * Stdlib only (urllib, json, argparse). No external dependencies.
  * Best effort: any network error is swallowed and polling continues; the tool
    is meant to outlive the disruptions it is watching.
  * Targets, credentials, cadences, and duration are all arguments — nothing is
    hardcoded to a bench.

EXAMPLE
  python3 tools/soak_disconnect_monitor.py \
      --remote PSTOP54=http://192.168.107.70 \
      --remote DUT=http://pstop-01d7f344 \
      --machine machn=http://100.84.155.111 \
      --admin-user admin --admin-pass "$ML_ADMIN_PASSWORD" \
      --machine-admin-pass microlink \
      --duration 43200 --out ./soak_run \
      --serial /dev/ttyACM1 --clean-target 28800

See docs/CONNECTIVITY_SOAK.md for how to run an acceptance ("N-hours-clean") soak
and how to read a disconnect event.
"""

import argparse
import base64
import json
import os
import subprocess
import sys
import threading
import time
import urllib.request

# --------------------------------------------------------------------------- #
# Detection thresholds (safe defaults; overridable via args).
# --------------------------------------------------------------------------- #
BOND_STATE_BONDED = 2  # pstop_machines[].state == 2 means bonded/running
DEFAULT_REPLY_STALL_S = 8  # sent advancing but replies frozen this long = stall
HTTP_TIMEOUT_S = 3


def now():
    return time.strftime('%Y-%m-%d %H:%M:%S')


def parse_kv(pairs):
    """['NAME=URL', ...] -> {NAME: URL}; strips a trailing slash on the URL."""
    out = {}
    for p in pairs or []:
        if '=' not in p:
            raise argparse.ArgumentTypeError(f'expected NAME=URL, got {p!r}')
        name, url = p.split('=', 1)
        out[name.strip()] = url.strip().rstrip('/')
    return out


def http_get_json(url, user=None, pw=None, timeout=HTTP_TIMEOUT_S):
    """Best-effort GET + JSON parse. Returns {'__err__': ...} on any failure."""
    try:
        req = urllib.request.Request(url)
        if user:
            token = base64.b64encode(f'{user}:{pw or ""}'.encode()).decode()
            req.add_header('Authorization', 'Basic ' + token)
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode())
    except Exception as e:  # noqa: BLE001 - best effort by design
        return {'__err__': str(e)}


def configured_slots(state):
    """Configured machine slots on a remote's /state.json (pstop_machines[])."""
    return [m for m in state.get('pstop_machines', []) if m.get('cfg')]


def slot_for_machine(state, machine_id):
    """The remote's machine slot pointing at machine_id, else first configured."""
    slots = configured_slots(state)
    if machine_id is not None:
        for m in slots:
            if m.get('id') == machine_id:
                return m
    return slots[0] if slots else None


class SerialCapture:
    """Best-effort continuous capture of a USB console to a file (a thread)."""

    def __init__(self, dev, path, baud='115200'):
        self.dev = dev
        self.path = path
        self.baud = baud

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                subprocess.run(
                    ['stty', '-F', self.dev, self.baud, 'raw', '-echo'],
                    timeout=5,
                    stderr=subprocess.DEVNULL,
                )
                with open(self.dev, 'rb') as src, open(self.path, 'ab', buffering=0) as out:
                    while True:
                        b = src.read(256)
                        if not b:
                            break
                        out.write(b)
            except Exception:  # noqa: BLE001 - device may vanish mid-soak
                time.sleep(3)

    def tail(self, n=50):
        try:
            with open(self.path, 'rb') as f:
                return b''.join(f.readlines()[-n:]).decode('utf-8', 'replace')
        except Exception:  # noqa: BLE001
            return '(no console)'


def machine_remotes_summary(machine_states):
    """Compact one-line render of every machine's bonded_remotes view."""
    parts = []
    for mname, mst in machine_states.items():
        if not isinstance(mst, dict) or '__err__' in mst:
            err = mst.get('__err__') if isinstance(mst, dict) else '?'
            parts.append(f'{mname}=UNREACHABLE({err})')
            continue
        rems = '; '.join(
            f'id={hex(r.get("id", 0))} st={r.get("state")} age={r.get("age_ms")}ms '
            f'rtt={r.get("rtt_ms")} wg_direct={r.get("wg_direct")} stop_only={r.get("stop_only")}'
            for r in mst.get('bonded_remotes', [])
        )
        parts.append(f'{mname} relay_stop={mst.get("relay_stop")} bonded=[{rems}]')
    return ' | '.join(parts)


def run(args):
    remotes = parse_kv(args.remote)
    machines = parse_kv(args.machine)
    if not remotes:
        print('error: at least one --remote NAME=URL is required', file=sys.stderr)
        return 2

    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)
    events_path = os.path.join(out_dir, 'events.log')
    samples_path = os.path.join(out_dir, 'samples.csv')
    telem_path = os.path.join(out_dir, 'telemetry.csv')
    status_path = os.path.join(out_dir, 'status.txt')

    admin_user = args.admin_user or os.environ.get('ML_ADMIN_USER') or 'admin'
    admin_pass = args.admin_pass or os.environ.get('ML_ADMIN_PASSWORD')
    machine_admin_pass = args.machine_admin_pass or os.environ.get('ML_MACHINE_ADMIN_PASSWORD')

    # Optional serial console capture (single device; typically the DUT remote).
    serial = None
    if args.serial:
        console_path = os.path.join(out_dir, 'console.log')
        serial = SerialCapture(args.serial, console_path)
        serial.start()

    def log_event(s):
        with open(events_path, 'a') as f:
            f.write(s + '\n')
        print(s, flush=True)

    # CSV headers (created once; appended thereafter so a run can resume).
    sample_hdr = [
        'ts',
        'elapsed_s',
        'remote',
        'slot_state',
        'sent',
        'replies',
        'send_fail',
        'rebonds',
        'rtt_ms',
        'wg_direct',
        'ml_state',
        'ml_reconnects',
        'eth_link',
        'eth_recoveries',
    ]
    telem_hdr = [
        'ts',
        'unit',
        'ml_reconnects',
        'ml_state',
        'slot_rtt_ms',
        'slot_wg_direct',
        'rebonds',
        'replies',
        'send_fail',
        'derp_home_region',
        'pp_has_direct',
    ]
    if not os.path.exists(samples_path):
        with open(samples_path, 'w') as f:
            f.write(','.join(sample_hdr) + '\n')
    if not os.path.exists(telem_path):
        with open(telem_path, 'w') as f:
            f.write(','.join(telem_hdr) + '\n')

    t0 = time.time()
    last = {}  # remote -> prior derived sample
    reply_ts = {}  # remote -> (last_replies_value, wallclock_of_change)
    event_counts = {n: 0 for n in remotes}
    last_event_wall = t0  # for --clean-target
    next_telem = 0.0  # telemetry cadence tracker

    log_event(
        f'[{now()}] === disconnect soak START '
        f'(remotes={list(remotes)} machines={list(machines)} '
        f'poll={args.poll_sec}s dur={args.duration}s) ==='
    )

    while time.time() - t0 < args.duration:
        elapsed = int(time.time() - t0)

        # ---- poll every unit once per cycle ----
        remote_states = {n: http_get_json(u + '/state.json') for n, u in remotes.items()}
        machine_states = {n: http_get_json(u + '/state.json') for n, u in machines.items()}

        for rname, url in remotes.items():
            st = remote_states[rname]
            if '__err__' in st:
                event_counts[rname] += 1
                last_event_wall = time.time()
                log_event(
                    f'[{now()}] EVENT #{event_counts[rname]} {rname}: '
                    f'state.json UNREACHABLE ({st["__err__"]}) elapsed={elapsed}s'
                )
                log_event('  machines: ' + machine_remotes_summary(machine_states))
                continue

            slot = slot_for_machine(st, args.machine_id)
            cur = dict(
                slot_state=(slot or {}).get('state'),
                sent=(slot or {}).get('sent', 0),
                replies=(slot or {}).get('replies', 0),
                send_fail=(slot or {}).get('send_fail', 0),
                rebonds=(slot or {}).get('rebonds', 0),
                rtt_ms=(slot or {}).get('rtt_ms'),
                wg_direct=(slot or {}).get('wg_direct'),
                ml_state=st.get('ml_state'),
                ml_reconnects=st.get('ml_reconnects', 0),
                eth_link=st.get('eth_link'),
                eth_recoveries=st.get('eth_recoveries', 0),
            )

            # reply-advance tracking (for stall detection)
            prev_rt = reply_ts.get(rname)
            if prev_rt is None or cur['replies'] != prev_rt[0]:
                reply_ts[rname] = (cur['replies'], time.time())

            # ---- disconnect detection vs previous sample ----
            lp = last.get(rname)
            if lp:
                trig = []
                if lp['slot_state'] == BOND_STATE_BONDED and cur['slot_state'] != BOND_STATE_BONDED:
                    trig.append(f'peer bond DROPPED {lp["slot_state"]}->{cur["slot_state"]}')
                if cur['rebonds'] > lp['rebonds']:
                    trig.append(f'rebonds {lp["rebonds"]}->{cur["rebonds"]}')
                if cur['ml_reconnects'] > lp['ml_reconnects']:
                    trig.append(f'ml_reconnects {lp["ml_reconnects"]}->{cur["ml_reconnects"]}')
                if cur['eth_recoveries'] > (lp['eth_recoveries'] or 0):
                    trig.append(f'eth_recovery reason={st.get("eth_rec_reason")}')
                stall = time.time() - reply_ts[rname][1]
                if cur['slot_state'] == BOND_STATE_BONDED and cur['sent'] > lp['sent'] and stall > args.reply_stall_sec:
                    trig.append(f'reply STALL {int(stall)}s (sent advancing, replies frozen)')

                if trig:
                    event_counts[rname] += 1
                    last_event_wall = time.time()
                    log_event(f'\n[{now()}] ===== EVENT #{event_counts[rname]} {rname} (elapsed={elapsed}s) =====')
                    log_event(f'  triggers: {"; ".join(trig)}')
                    log_event(
                        f'  {rname} slot: state={cur["slot_state"]} '
                        f'sent={cur["sent"]} replies={cur["replies"]} '
                        f'send_fail={cur["send_fail"]} rebonds={cur["rebonds"]} '
                        f'rtt_ms={cur["rtt_ms"]} wg_direct={cur["wg_direct"]} '
                        f'last_reply_ms={(slot or {}).get("last_reply_ms")}'
                    )
                    log_event(
                        f'  {rname} net:  ml_state={st.get("ml_state")} '
                        f'ml_reconnects={st.get("ml_reconnects")} '
                        f'eth_link={st.get("eth_link")} '
                        f'eth_rec={st.get("eth_recoveries")}/reason={st.get("eth_rec_reason")} '
                        f'sf_route={st.get("pstop_sf_route")} '
                        f'sf_txdrv={st.get("pstop_sf_txdrv")} '
                        f'sf_errno={st.get("pstop_sf_errno")} '
                        f'wg_pbuf_fails={st.get("wg_pbuf_fails")} '
                        f'gw_rtt_ms={st.get("gw_rtt_ms")}'
                    )
                    log_event('  machines: ' + machine_remotes_summary(machine_states))
                    if serial:
                        log_event(f'  --- console tail ({args.serial}) ---\n{serial.tail(50)}\n  --- end console ---')
                    if args.on_disconnect:
                        try:
                            subprocess.run(args.on_disconnect, shell=True, timeout=30)
                        except Exception as e:  # noqa: BLE001
                            log_event(f'  on-disconnect hook FAILED: {e}')

            last[rname] = cur

            # ---- wide sample row ----
            with open(samples_path, 'a') as f:
                f.write(
                    ','.join(
                        str(x)
                        for x in [
                            now(),
                            elapsed,
                            rname,
                            cur['slot_state'],
                            cur['sent'],
                            cur['replies'],
                            cur['send_fail'],
                            cur['rebonds'],
                            cur['rtt_ms'],
                            cur['wg_direct'],
                            cur['ml_state'],
                            cur['ml_reconnects'],
                            cur['eth_link'],
                            cur['eth_recoveries'],
                        ]
                    )
                    + '\n'
                )

        # ---- finer telemetry cadence (with DERP counters if admin creds set) ----
        if time.time() - t0 >= next_telem:
            next_telem = (time.time() - t0) + args.telem_sec
            all_units = list(remotes.items()) + list(machines.items())
            for uname, url in all_units:
                is_machine = uname in machines
                st = remote_states.get(uname) or machine_states.get(uname) or {}
                pw = machine_admin_pass if is_machine else admin_pass
                mon = {}
                if pw:
                    mon = http_get_json(url + '/admin/api/monitor', admin_user, pw, timeout=4)
                slot = slot_for_machine(st, None) if isinstance(st, dict) else None
                with open(telem_path, 'a') as f:
                    f.write(
                        ','.join(
                            str(x)
                            for x in [
                                now(),
                                uname,
                                st.get('ml_reconnects', '') if isinstance(st, dict) else '',
                                st.get('ml_state', '') if isinstance(st, dict) else '',
                                (slot or {}).get('rtt_ms', ''),
                                (slot or {}).get('wg_direct', ''),
                                (slot or {}).get('rebonds', ''),
                                (slot or {}).get('replies', ''),
                                (slot or {}).get('send_fail', ''),
                                mon.get('derp_home_region', '') if isinstance(mon, dict) else '',
                                mon.get('pp_has_direct', '') if isinstance(mon, dict) else '',
                            ]
                        )
                        + '\n'
                    )

        # ---- rolling human-readable status ----
        with open(status_path, 'w') as f:
            f.write(f'[{now()}] elapsed={elapsed}s/{args.duration}s  events: {dict(event_counts)}\n')
            clean_s = int(time.time() - last_event_wall)
            f.write(
                f'  clean streak: {clean_s}s' + (f' / target {args.clean_target}s' if args.clean_target else '') + '\n'
            )
            for rname, lp in last.items():
                f.write(
                    f'  {rname}: state={lp["slot_state"]} replies={lp["replies"]} '
                    f'rebonds={lp["rebonds"]} ml={lp["ml_state"]} '
                    f'ml_reconnects={lp["ml_reconnects"]}\n'
                )

        # ---- N-hours-clean acceptance exit ----
        if args.clean_target and (time.time() - last_event_wall) >= args.clean_target:
            log_event(
                f'[{now()}] === CLEAN TARGET MET: {args.clean_target}s with '
                f'zero events (total events so far: {dict(event_counts)}) ==='
            )
            return 0

        time.sleep(args.poll_sec)

    total = sum(event_counts.values())
    log_event(f'[{now()}] === soak END: events={dict(event_counts)} (total={total}) ===')
    # Non-clean acceptance runs still exit 0; the events.log is the verdict.
    return 0


def build_parser():
    p = argparse.ArgumentParser(
        prog='soak_disconnect_monitor.py',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__,
    )
    p.add_argument(
        '--remote',
        action='append',
        metavar='NAME=URL',
        required=True,
        help='A pstop remote to watch, e.g. PSTOP54=http://192.168.107.70. Repeatable.',
    )
    p.add_argument(
        '--machine',
        action='append',
        metavar='NAME=URL',
        default=[],
        help='A pstop machine node, e.g. machn=http://100.84.155.111. '
        'Repeatable. Used for the bonded_remotes context in events.',
    )
    p.add_argument(
        '--machine-id',
        type=lambda s: int(s, 0),
        default=None,
        help='Numeric machine device id (e.g. 0x01020304) to select the '
        'matching slot on each remote. Default: first configured slot.',
    )
    p.add_argument(
        '--admin-user', default=None, help="Admin user for /admin/api/monitor (default env ML_ADMIN_USER or 'admin')."
    )
    p.add_argument(
        '--admin-pass',
        default=None,
        help='Admin password for remotes (default env ML_ADMIN_PASSWORD). '
        'If unset, DERP telemetry columns are left blank.',
    )
    p.add_argument(
        '--machine-admin-pass',
        default=None,
        help='Admin password for machine nodes if different from remotes (default env ML_MACHINE_ADMIN_PASSWORD).',
    )
    p.add_argument(
        '--poll-sec', dest='poll_sec', type=float, default=5.0, help='Seconds between full poll cycles (default 5).'
    )
    p.add_argument(
        '--telem-sec',
        dest='telem_sec',
        type=float,
        default=30.0,
        help='Seconds between telemetry.csv rows (default 30).',
    )
    p.add_argument(
        '--reply-stall-sec',
        dest='reply_stall_sec',
        type=float,
        default=DEFAULT_REPLY_STALL_S,
        help='A bonded slot with sent advancing but replies frozen this long is a stall event (default 8).',
    )
    p.add_argument('--duration', type=float, default=43200, help='Total run time in seconds (default 43200 = 12h).')
    p.add_argument(
        '--out',
        default='./soak_run',
        help='Output directory for events.log / samples.csv / telemetry.csv / status.txt (default ./soak_run).',
    )
    p.add_argument(
        '--serial',
        default=None,
        metavar='DEV',
        help='Optional USB console device (e.g. /dev/ttyACM1) to capture and tail into each event record.',
    )
    p.add_argument(
        '--on-disconnect',
        default=None,
        metavar='CMD',
        help='Optional shell command run once per detected event (generic hook; e.g. to reset a HIL fixture).',
    )
    p.add_argument(
        '--clean-target',
        dest='clean_target',
        type=float,
        default=None,
        metavar='SECONDS',
        help='Acceptance mode: exit 0 as soon as this many seconds elapse '
        "with zero new events (the 'N-hours-clean' run).",
    )
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        return run(args)
    except KeyboardInterrupt:
        print('\ninterrupted; partial logs retained in --out dir', file=sys.stderr)
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
