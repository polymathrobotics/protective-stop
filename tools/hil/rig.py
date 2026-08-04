# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Pure helpers for the pstop HIL rig — no pytest imports here.

Three actors:

  RelayRig      — the 4-channel USB relay board mapped onto rig functions
                  (open/close each E-stop loop, cut/restore DUT power).
  Chip          — the DUT's HTTP API (state.json, peer-slot table).
  MachineRunner — a dedicated machine_app_runner subprocess whose stderr is
                  captured as a timestamped event stream tests assert on.
"""

from __future__ import annotations

import json
import re
import subprocess
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

# Machine-runner stderr vocabulary (host/machine_app_runner.c).
STATUS_RE = re.compile(r'\*\*\* ROBOT STATUS -> (OK|STOP)')
STATUS_OK_RE = re.compile(r'\*\*\* ROBOT STATUS -> OK')
STATUS_STOP_RE = re.compile(r'\*\*\* ROBOT STATUS -> STOP')
RX_RE = re.compile(r'RX <- remote 0x[0-9A-Fa-f]+\s+now sending (STOP|OK)')
RX_STOP_RE = re.compile(r'now sending STOP')
RX_OK_RE = re.compile(r'now sending OK')
LIVENESS_RE = re.compile(r'declared lost \[MISSED_HEARTBEATS\]')
DEFERRED_RE = re.compile(r'ANOMALY: arming DEFERRED')


def discover_chip_ip(iface: str = 'esp-pstop0', timeout: float = 15.0) -> str:
    """Return the chip's USB-NCM IP from the kernel neighbor table.

    Retries: right after a chip (re)boot the NCM interface and its neighbor
    entry can lag USB enumeration by several seconds."""
    deadline = time.monotonic() + timeout
    last = 'no attempt'
    while time.monotonic() < deadline:
        r = subprocess.run(
            ['ip', '-4', 'neigh', 'show', 'dev', iface],
            capture_output=True,
            text=True,
        )
        if r.returncode != 0:
            last = f'interface {iface} not present'
        else:
            candidates = [ln.split()[0] for ln in r.stdout.splitlines() if 'lladdr' in ln]
            if len(candidates) == 1:
                return candidates[0]
            last = f'neighbors on {iface}: {candidates}'
            if len(candidates) > 1:
                raise RuntimeError(f'multiple neighbors on {iface}: {candidates}')
        time.sleep(0.5)
    raise RuntimeError(f'chip discovery failed after {timeout}s — {last}')


class ChipUnreachable(Exception):
    pass


class Chip:
    """The DUT's HTTP admin API."""

    def __init__(self, host: str, timeout: float = 2.0):
        self.base = f'http://{host}'
        self.timeout = timeout

    def _get(self, path: str) -> str:
        try:
            with urllib.request.urlopen(self.base + path, timeout=self.timeout) as r:
                return r.read().decode()
        except (urllib.error.URLError, OSError, TimeoutError) as e:
            raise ChipUnreachable(f'GET {path}: {e}') from e

    def _post(self, path: str) -> str:
        req = urllib.request.Request(self.base + path, method='POST', data=b'')
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return r.read().decode()
        except (urllib.error.URLError, OSError, TimeoutError) as e:
            raise ChipUnreachable(f'POST {path}: {e}') from e

    def state(self) -> dict:
        return json.loads(self._get('/state.json'))

    def reachable(self) -> bool:
        try:
            self.state()
            return True
        except ChipUnreachable:
            return False

    def fw(self) -> tuple[str, str] | None:
        """(fw_ver, fw_sha) of the running build, or None on firmware
        older than 4c90711 (which does not report identity)."""
        s = self.state()
        if 'fw_ver' not in s or 'fw_sha' not in s:
            return None
        return str(s['fw_ver']), str(s['fw_sha'])

    def loops_closed(self) -> tuple[bool, bool]:
        """(channel_a_closed, channel_b_closed) from the loop health bits."""
        s = self.state()
        return (
            bool(s['e_hi0']) and bool(s['e_lo0']),
            bool(s['e_hi1']) and bool(s['e_lo1']),
        )

    def wait_state(self, pred, timeout: float, poll: float = 0.25, what: str = 'condition'):
        """Poll state() until pred(state) is truthy; returns the state dict."""
        deadline = time.monotonic() + timeout
        last_exc: Exception | None = None
        while time.monotonic() < deadline:
            try:
                s = self.state()
                if pred(s):
                    return s
                last_exc = None
            except ChipUnreachable as e:
                last_exc = e
            time.sleep(poll)
        raise TimeoutError(f'chip: {what} not met within {timeout}s (last error: {last_exc})')

    def wait_reachable(self, timeout: float) -> dict:
        return self.wait_state(lambda s: True, timeout, what='reachable')

    def wait_gone(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self.reachable():
                return
            time.sleep(0.25)
        raise TimeoutError(f'chip still reachable {timeout}s after power cut')

    def set_peer_slot(self, slot: int, ip: str, port: int, machine_id: int) -> None:
        # The firmware parses id with strtoul(base=0): the 0x prefix is
        # REQUIRED — a bare "01020390" would be read as octal (= 4227).
        resp = json.loads(self._post(f'/api/pstop_peers?slot={slot}&ip={ip}&port={port}&id=0x{machine_id:08X}'))
        if not resp.get('ok') or resp.get('id') != machine_id:
            raise RuntimeError(f'set_peer_slot({slot}) failed or id mismatch: {resp}')

    def clear_peer_slot(self, slot: int) -> None:
        resp = json.loads(self._post(f'/api/pstop_peers?slot={slot}&clear=1'))
        if not resp.get('ok'):
            raise RuntimeError(f'clear_peer_slot({slot}) failed: {resp}')


class RelayRig:
    """Maps relay channels onto rig functions, handling contact polarity."""

    def __init__(self, board, relays_cfg: dict):
        self.board = board
        self.ch = {'a': relays_cfg['channel_a'], 'b': relays_cfg['channel_b']}
        self.power_ch = relays_cfg['dut_power']
        self.press_energizes = relays_cfg['press_energizes']
        self.cut_energizes = relays_cfg['power_cut_energizes']

    def open_loop(self, channel: str) -> None:
        """Open one E-stop loop — half (or all) of a button press."""
        self.board.set(self.ch[channel], self.press_energizes)

    def close_loop(self, channel: str) -> None:
        self.board.set(self.ch[channel], not self.press_energizes)

    def press(self) -> None:
        """Open BOTH loops in one serial write — like a real DPST press,
        both poles move together (skew ~= relay mechanics, not protocol)."""
        self.board.set_many({self.ch['a']: self.press_energizes, self.ch['b']: self.press_energizes})

    def release(self) -> None:
        self.board.set_many({self.ch['a']: not self.press_energizes, self.ch['b']: not self.press_energizes})

    def power_off(self) -> None:
        self.board.set(self.power_ch, self.cut_energizes)

    def power_on(self) -> None:
        self.board.set(self.power_ch, not self.cut_energizes)

    def idle(self) -> None:
        """Bench-normal: loops closed, DUT powered."""
        self.release()
        self.power_on()


class MachineRunner:
    """Dedicated machine_app_runner whose stderr becomes an event stream."""

    def __init__(self, binary: Path, workdir: Path, port: int, device_id: int, timing: dict):
        self.binary = Path(binary)
        self.workdir = Path(workdir)
        self.port = port
        self.device_id = device_id
        self.timing = timing
        self.events: list[tuple[float, str]] = []
        self._cond = threading.Condition()
        self._proc: subprocess.Popen | None = None
        self._reader: threading.Thread | None = None

    def _write_config(self) -> Path:
        cfg = self.workdir / 'hil_machine.toml'
        cfg.write_text(
            '[network]\n'
            f'port = {self.port}\n'
            'bind = "0.0.0.0"\n'
            '[machine]\n'
            f'machine_device_id = 0x{self.device_id:08X}\n'
            '[limits]\n'
            'max_lost_messages = 10\n'
            f'max_missed_heartbeats = {self.timing["max_missed_heartbeats"]}\n'
            'max_remotes = 3\n'
            '[policy]\n'
            'allow_unlisted = true\n'
            f'default_heartbeat_ms = {self.timing["heartbeat_ms"]}\n'
            f'min_stop_ms = {self.timing["min_stop_ms"]}\n'
        )
        return cfg

    def start(self) -> None:
        cfg = self._write_config()
        self._proc = subprocess.Popen(
            [str(self.binary.resolve()), str(cfg)],
            cwd=self.workdir,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        self._reader = threading.Thread(target=self._pump, daemon=True)
        self._reader.start()
        # Fail fast on startup errors (bad config, port already bound, ...)
        # instead of letting callers time out waiting for a dead process.
        time.sleep(0.3)
        if self._proc.poll() is not None:
            with self._cond:
                tail = '\n'.join(line for _, line in self.events[-5:])
            raise RuntimeError(f'machine_app_runner exited rc={self._proc.returncode} at startup:\n{tail}')

    def _pump(self) -> None:
        assert self._proc is not None and self._proc.stderr is not None
        for line in self._proc.stderr:
            with self._cond:
                self.events.append((time.monotonic(), line.rstrip('\n')))
                self._cond.notify_all()

    def stop(self) -> None:
        if self._proc is not None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait()
        if self._reader is not None:
            self._reader.join(timeout=2)

    def mark(self) -> int:
        """Bookmark the event stream; pass to wait_for/find as `since`."""
        with self._cond:
            return len(self.events)

    def wait_for(self, pattern: re.Pattern, timeout: float, since: int = 0):
        """Block until an event at index >= since matches; returns (idx, ts, match)."""
        deadline = time.monotonic() + timeout
        idx = since
        with self._cond:
            while True:
                while idx < len(self.events):
                    ts, line = self.events[idx]
                    m = pattern.search(line)
                    if m:
                        return idx, ts, m
                    idx += 1
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    tail = '\n'.join(line for _, line in self.events[-8:])
                    raise TimeoutError(f'no /{pattern.pattern}/ within {timeout}s; log tail:\n{tail}')
                self._cond.wait(remaining)

    def find(self, pattern: re.Pattern, since: int = 0):
        """Non-blocking scan; returns list of (idx, ts, match)."""
        with self._cond:
            snapshot = list(enumerate(self.events))[since:]
        return [(i, ts, m) for i, (ts, line) in snapshot if (m := pattern.search(line))]

    def robot_status(self, since: int = 0) -> str | None:
        """Last ROBOT STATUS ('OK'/'STOP') at or after `since`, else None."""
        with self._cond:
            snapshot = list(self.events[since:])
        status = None
        for _, line in snapshot:
            m = STATUS_RE.search(line)
            if m:
                status = m.group(1)
        return status
