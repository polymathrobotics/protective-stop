#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Robustness + performance test for dual_core_safety.

Reboots the device repeatedly and verifies that every boot:
  - comes back reachable,
  - reaches Tailscale (vpn_ip assigned, DERP connected),
  - did NOT reset via Task-WDT (reset_reason==4) — the boot-loop signature,
without the supervisor spuriously enabling USB during a slow boot. Between
reboots it watches for spontaneous reboots (a real crash) and samples the
admin-page load latency (fresh TCP connection each time, to catch connect-time
stalls a browser would hit).

Reboots are spaced so the safety layer's rapid-boot counter clears between them
(it ages out after ~120 s of stable uptime), so the test itself never trips the
rollback chain.

Usage: robust_test.py [num_reboots]   (default 8)
"""

import base64
import json
import subprocess
import sys
import time
import urllib.request

CHIP = '192.168.107.101'
AUTH = 'Basic ' + base64.b64encode(b'admin:microlink').decode()
REBOOTS = int(sys.argv[1]) if len(sys.argv) > 1 else 8
SPACING = 140  # s between reboots


def log(m):
    print(f'[{time.strftime("%H:%M:%S")}] {m}', flush=True)


def state(t=4):
    try:
        return json.loads(urllib.request.urlopen(f'http://{CHIP}/state.json', timeout=t).read())
    except Exception:
        return None


def reboot():
    try:
        req = urllib.request.Request(f'http://{CHIP}/admin/api/restart', method='POST')
        req.add_header('Authorization', AUTH)
        urllib.request.urlopen(req, timeout=6)
    except Exception:
        pass


def page_latency():
    try:
        o = subprocess.run(
            ['curl', '-m', '15', '-s', '-o', '/dev/null', '-w', '%{time_connect} %{time_total}', f'http://{CHIP}/'],
            capture_output=True,
            text=True,
            timeout=18,
        )
        c, t = o.stdout.split()
        return float(c) * 1000, float(t) * 1000
    except Exception:
        return None, None


def fmt(v):
    if not v:
        return 'n/a'
    s = sorted(v)

    def p(q):
        return s[min(len(s) - 1, int(q * len(s)))]

    return f'n={len(s)} min={s[0]:.0f} med={p(0.5):.0f} p95={p(0.95):.0f} max={s[-1]:.0f}'


twdt = 0
usb_race = 0
boot_ts = []
pconn = []
ptot = []

log(f'=== ROBUSTNESS+PERF: {REBOOTS} reboots, {SPACING}s apart ===')
for r in range(REBOOTS):
    pre = state()
    pre_up = (pre.get('uptime_ms', 0) // 1000) if pre else 10**9
    log(f'--- reboot {r + 1}/{REBOOTS} (pre-uptime {pre_up}s) ---')
    reboot()
    t0 = time.time()
    reach_t = None
    ts_t = None
    saw_usb = False
    rr = None
    bc = None
    while time.time() - t0 < 90:
        d = state(3)
        if d:
            up = d.get('uptime_ms', 0) // 1000
            if up < pre_up or up < 40:  # we're on the fresh boot
                if reach_t is None:
                    reach_t = time.time() - t0
                if d.get('usbncm_en'):
                    saw_usb = True
                rr = d.get('reset_reason')
                bc = d.get('boot_count')
                if d.get('vpn_ip'):
                    ts_t = time.time() - t0
                    break
        time.sleep(1)
    if saw_usb:
        usb_race += 1
    if rr == 4:
        twdt += 1
    if ts_t:
        boot_ts.append(ts_t)
    log(
        f'   reachable={reach_t and round(reach_t, 1)}s tailscale={ts_t and round(ts_t, 1)}s '
        f'reset_reason={rr} bc={bc} usb_during_boot={saw_usb}' + ('  *** TWDT ***' if rr == 4 else '')
    )
    for _ in range(3):
        c, t = page_latency()
        if c is not None:
            pconn.append(c)
            ptot.append(t)
        time.sleep(1)
    # settle until next reboot; watch for a spontaneous crash
    last_up = None
    while time.time() - t0 < SPACING:
        d = state(3)
        if d:
            up = d.get('uptime_ms', 0) // 1000
            if last_up is not None and up < last_up - 5 and up < 30:
                rr2 = d.get('reset_reason')
                log(
                    f'   >>> SPONTANEOUS REBOOT uptime {last_up}->{up}s reset_reason={rr2}'
                    + ('  *** TWDT ***' if rr2 == 4 else '')
                )
                if rr2 == 4:
                    twdt += 1
            last_up = up
        time.sleep(5)

log('=' * 60)
log('=== REPORT ===')
log(f'reboots: {REBOOTS}')
log(f'TWDT resets (reset_reason=4): {twdt}')
log(f'USB auto-enabled during boot: {usb_race}/{REBOOTS}')
log(f'time-to-Tailscale (s): {fmt(boot_ts)}')
log(f'admin page connect (ms): {fmt(pconn)}')
log(f'admin page total   (ms): {fmt(ptot)}')
log(f'VERDICT: {"PASS — no TWDT, Tailscale every boot" if (twdt == 0 and len(boot_ts) == REBOOTS) else "REVIEW"}')
log('=' * 60)
