#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""SR-H-03 / FMEA DU-4 — the host machine runner must REFUSE unsafe timing
config (min_stop_ms below the arming floor, an out-of-range heartbeat window,
or an out-of-range max_missed), matching the ROS2 node's validate_timing floors.
A machine that cannot enforce the arming gesture (SF-3) or stop latency (SF-1)
must fail-safe by refusing to start.

Self-contained: writes temp configs, runs host/machine_app_runner, and checks it
exits nonzero with a reason on unsafe values and starts on a valid one."""

import os
import signal
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
RUNNER = os.path.join(HERE, '..', 'host', 'machine_app_runner')
PORT = 8955

BASE = """
[network]
bind = "127.0.0.1"
[machine]
machine_device_id = 0x01020304
[limits]
max_missed_heartbeats = {max_missed}
max_remotes = 4
[policy]
allow_unlisted = true
default_heartbeat_ms = {hb}
min_stop_ms = {min_stop}
"""


def cfg(hb=400, max_missed=5, min_stop=500):
    return BASE.format(hb=hb, max_missed=max_missed, min_stop=min_stop)


def run(cfg_text):
    """Return (returncode, stderr). returncode None => still running (started)."""
    with tempfile.NamedTemporaryFile('w', suffix='.toml', delete=False) as f:
        f.write(cfg_text)
        path = f.name
    try:
        p = subprocess.Popen([RUNNER, path, str(PORT)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            _, err = p.communicate(timeout=3)
            return p.returncode, err.decode(errors='replace')
        except subprocess.TimeoutExpired:
            p.send_signal(signal.SIGTERM)
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
            return None, ''  # None => it bound + ran => started
    finally:
        os.unlink(path)


checks = 0
fails = 0


def check(cond, name):
    global checks, fails
    checks += 1
    if cond:
        print(f'  ok:   {name}')
    else:
        fails += 1
        print(f'  FAIL: {name}')


if not os.path.exists(RUNNER):
    print(f'machine_app_runner not built at {RUNNER} — run `make -C host` first')
    sys.exit(2)

# Unsafe configs MUST be refused (nonzero exit + a reason mentioning the field).
unsafe = [
    ('min_stop_ms=0 defeats arming (SF-3)', cfg(min_stop=0), 'min_stop_ms'),
    ('min_stop_ms=50 below floor 100', cfg(min_stop=50), 'min_stop_ms'),
    ('max_missed=0', cfg(max_missed=0), 'max_missed'),
    ('max_missed=9 (>5)', cfg(max_missed=9), 'max_missed'),
    ('heartbeat=30 (<50)', cfg(hb=30), 'heartbeat'),
    ('heartbeat=2000 (>1000, window>1s defeats SF-1 latency)', cfg(hb=2000), 'heartbeat'),
]
for name, c, token in unsafe:
    rc, err = run(c)
    check(rc not in (None, 0) and token in err and 'UNSAFE CONFIG' in err, f'REFUSED: {name}')

# A valid config MUST start (defaults 400 ms / 3 / 500 ms).
rc, _ = run(cfg())
check(rc is None, 'valid config (400/3/500) starts')

print(f'config-floor tests: {checks} checks, {fails} failures')
sys.exit(1 if fails else 0)
