#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Per-remote soak gate: measure each remote's stop events independently.

Generalizes the bench drivers that were previously sed-copied per run
(soak_ros2_run48/49/50/51.py — one file per run number was the tooling debt
this replaces). Everything run-specific is a TOML config + argv:

  soak_per_remote.py bench.toml --name run-52 --duration-h 8

Per-remote stop events, cause-classified exactly as the runs 48-51 gates:
  AGE       reply_age_ms > age_limit_ms (machine would drop this remote)
  REBOND    the remote's rebond counter ticked (bond dropped + re-established)
  MISMATCH  the remote's pstop_mismatch ticked (self-asserted STOP blip)

Verdict: PASS if `criterion = "any"` and at least one remote logged zero
events (Ilia's run-48 spec), or `criterion = "all"` for zero-exclusion.

Config (TOML — repo config convention):

  machine_hb_topic = "/pstop_hb"          # ROS 2 heartbeat source, OR:
  #machine_state_url = "http://10.0.0.5/state.json"   # ESP32 machine source
  criterion = "any"                        # "any" | "all"
  age_limit_ms = 2000
  rearm_cmd = "python3 rearm_gesture.py"  # optional; run when disarmed+settled
  machine_id = 33686276                    # optional; scopes REBOND counting to
                                           # this machine's slot on each remote
  [remotes.PS]
  device_id = "01d7eed0"                   # hex id as published by the machine
  admin_url = "http://10.0.0.2"            # remote's own HTTP (mismatch polls)

The ROS 2 source needs rclpy + protective_stop_msg on PYTHONPATH (source the
workspace); the HTTP source has no dependencies beyond stdlib + tomllib.
"""

import argparse
import json
import shlex
import subprocess
import time
import urllib.request

import tomllib


def http_json(url, timeout=6):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return json.load(resp)
    except Exception:
        return None


class Ros2Source:
    """armed truth from /pstop_hb + per-remote ages from ~/remotes."""

    def __init__(self, hb_topic):
        import rclpy
        from protective_stop_msg.msg import BondedRemoteArray, ProtectiveStopHeartbeat
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        self._rclpy = rclpy
        rclpy.init()
        self._node = Node('soak_per_remote')
        self._hb_stop = None
        self._hb_at = 0.0
        self._remotes = {}
        self._remotes_at = 0.0

        def on_hb(msg):
            self._hb_stop = bool(msg.stop)
            self._hb_at = time.time()

        def on_remotes(msg):
            self._remotes = {r.device_id: (int(r.reply_age_ms), int(r.rebonds)) for r in msg.remotes}
            self._remotes_at = time.time()

        self._node.create_subscription(
            ProtectiveStopHeartbeat, hb_topic, on_hb, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )
        self._node.create_subscription(BondedRemoteArray, '/machine_bridge/remotes', on_remotes, 10)

    def sample(self):
        self._rclpy.spin_once(self._node, timeout_sec=0.2)
        now = time.time()
        armed = (now - self._hb_at) < 1.5 and self._hb_stop is False
        fresh = (now - self._remotes_at) < 3.0
        ages = {dev: age for dev, (age, _) in self._remotes.items()} if fresh else {}
        return armed, ages

    def idle(self, seconds):
        self._rclpy.spin_once(self._node, timeout_sec=seconds)


class HttpSource:
    """armed truth + per-remote ages from an ESP32 machine's /state.json."""

    def __init__(self, state_url):
        self._url = state_url

    def idle(self, seconds):
        time.sleep(seconds)

    def sample(self):
        state = http_json(self._url, timeout=4)
        if state is None:
            return False, {}
        armed = state.get('l0') == 1 and state.get('l1') == 1
        remotes = {}
        for rec in state.get('bonded_remotes', []):
            # bonded_remotes[] carries no rebond counter (review red) — ages
            # only here; rebonds come from each REMOTE's own machine-slot
            # records in the 30 s admin poll below.
            remotes['%08x' % rec.get('id', 0)] = int(rec.get('age_ms', 0))
        return armed, remotes


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('config', help='TOML config (see module docstring)')
    ap.add_argument('--name', required=True, help='run name; log goes to <name>.log next to the config')
    ap.add_argument('--duration-h', type=float, default=8.0)
    args = ap.parse_args()

    with open(args.config, 'rb') as fh:
        cfg = tomllib.load(fh)
    remotes = cfg['remotes']
    age_limit = int(cfg.get('age_limit_ms', 2000))
    criterion = cfg.get('criterion', 'any')
    logpath = args.name + '.log'

    def log(msg):
        line = '[%s] %s' % (time.strftime('%Y-%m-%d %H:%M:%S'), msg)
        print(line, flush=True)
        with open(logpath, 'a') as fh:
            fh.write(line + '\n')

    if 'machine_hb_topic' in cfg:
        source = Ros2Source(cfg['machine_hb_topic'])
    else:
        source = HttpSource(cfg['machine_state_url'])

    events = {name: {'AGE': 0, 'REBOND': 0, 'MISMATCH': 0} for name in remotes}
    last_rebonds = {}
    # MISMATCH baselines lazily on the first SUCCESSFUL poll (same first-sample
    # guard REBOND uses): an unreachable-at-start remote with a pre-existing
    # nonzero counter must not get charged its history as run events (review 🟡).
    last_mismatch = {}
    in_age = {name: False for name in remotes}
    seen = set()  # remotes actually observed at least once — an unobserved
    # remote must not masquerade as clean (review 🟡)

    log(
        '%s START (%.1fh per-remote gate, criterion=%s, age_limit=%dms)'
        % (args.name, args.duration_h, criterion, age_limit)
    )

    t0 = time.time()
    next_sample = t0
    next_http = t0
    next_hour = t0 + 3600
    last_rearm = 0.0
    by_id = {rc['device_id']: name for name, rc in remotes.items()}

    while time.time() - t0 < args.duration_h * 3600:
        now = time.time()
        if now < next_sample:
            # Throttle BEFORE sampling (review yellow: the HTTP source was an
            # unthrottled busy-loop hammering the machine between samples).
            source.idle(min(0.2, next_sample - now))
            continue
        armed, ages = source.sample()
        now = time.time()
        next_sample = now + 2.0

        for dev_id, age in ages.items():
            name = by_id.get(dev_id)
            if name is None:
                continue
            seen.add(name)
            if age > age_limit and not in_age[name]:
                in_age[name] = True
                events[name]['AGE'] += 1
                log('STOP-EVENT %s AGE #%d (age=%dms) +%dm' % (name, events[name]['AGE'], age, (now - t0) // 60))
            elif age <= age_limit:
                in_age[name] = False

        if now >= next_http:
            next_http = now + 30
            for name, rc in remotes.items():
                state = http_json(rc['admin_url'] + '/state.json', timeout=4)
                if state is None:
                    continue
                seen.add(name)
                mm = state.get('pstop_mismatch') or 0
                if name in last_mismatch and mm > last_mismatch[name]:
                    events[name]['MISMATCH'] += mm - last_mismatch[name]
                    log('STOP-EVENT %s MISMATCH #%d +%dm' % (name, events[name]['MISMATCH'], (now - t0) // 60))
                last_mismatch[name] = mm
                # Rebonds from the remote's own machine-slot records (summed
                # over configured slots): bonded_remotes[] on the machine side
                # has no rebond counter (review red).
                machine_id = cfg.get('machine_id')  # optional: only charge the machine under soak
                rb = sum(
                    slot.get('rebonds', 0)
                    for slot in state.get('pstop_machines', [])
                    if slot.get('cfg') and (machine_id is None or slot.get('id') == machine_id)
                )
                if name in last_rebonds and rb > last_rebonds[name]:
                    events[name]['REBOND'] += rb - last_rebonds[name]
                    log('STOP-EVENT %s REBOND #%d +%dm' % (name, events[name]['REBOND'], (now - t0) // 60))
                last_rebonds[name] = rb

        if not armed and cfg.get('rearm_cmd') and (now - last_rearm) > 90:
            # Settle-check scoped to THIS run's remotes (review 🟡: a stale
            # unrelated bonded remote could pin it False forever), and every
            # configured remote must be present and fresh.
            tracked = {by_id[dev]: age for dev, age in ages.items() if dev in by_id}
            if len(tracked) == len(remotes) and all(age < age_limit * 0.75 for age in tracked.values()):
                last_rearm = now
                subprocess.Popen(shlex.split(cfg['rearm_cmd']))
                log('rearm attempt')

        if now >= next_hour:
            next_hour += 3600
            log('HOURLY +%dm armed=%s | events=%s' % ((now - t0) // 60, armed, events))

    unobserved = [name for name in remotes if name not in seen]
    if unobserved:
        log('WARNING: never observed: %s (excluded from clean set)' % unobserved)
    clean = [name for name, ev in events.items() if sum(ev.values()) == 0 and name in seen]
    ok = bool(clean) if criterion == 'any' else len(clean) == len(remotes)
    log(
        '%s %s: clean_remotes=%s | events=%s (criterion=%s)'
        % (args.name, 'PASS' if ok else 'FAIL', clean or 'none', events, criterion)
    )


if __name__ == '__main__':
    main()
