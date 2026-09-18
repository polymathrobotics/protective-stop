# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Regression tests for flash_pstop.sh, run as a real bash subprocess against
fake_esptool.py and a fake udevadm/uv on PATH — no hardware, no real device."""

import os
import shutil
import stat
import subprocess
from pathlib import Path

import pytest
from conftest import FAKE_ESPTOOL, read_argv_log

pytestmark = pytest.mark.skipif(shutil.which('bash') is None, reason='bash is not available on this runner')

REPO_TOOLS = Path(__file__).resolve().parents[1]
SCRIPT_SRC = REPO_TOOLS / 'flash_pstop.sh'

FAKE_UV = """#!/usr/bin/env bash
set -euo pipefail
if [ "${1:-}" != "run" ]; then
  echo "fake uv: unsupported invocation: $*" >&2
  exit 1
fi
shift
if [ "${1:-}" = "--project" ]; then
  shift 2
fi
if [ "${1:-}" = "--quiet" ]; then
  shift
fi
if [ "${1:-}" = "esptool" ]; then
  shift
fi
exec python3 "$FAKE_ESPTOOL_PATH" "$@"
"""

FAKE_UDEVADM = """#!/usr/bin/env bash
echo "ID_VENDOR_ID=${FAKE_UDEV_VID:-303a}"
echo "ID_MODEL_ID=${FAKE_UDEV_PID:-0009}"
"""


def _write_exe(path: Path, content: str):
    path.write_text(content)
    path.chmod(path.stat().st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)


@pytest.fixture
def script(tmp_path) -> Path:
    """A real copy of flash_pstop.sh, run in place (not edited)."""
    dst = tmp_path / 'flash_pstop.sh'
    shutil.copy2(SCRIPT_SRC, dst)
    return dst


@pytest.fixture
def fake_bin(tmp_path) -> Path:
    """A directory holding fake uv and udevadm, put first on PATH."""
    bindir = tmp_path / 'fakebin'
    bindir.mkdir()
    _write_exe(bindir / 'uv', FAKE_UV)
    _write_exe(bindir / 'udevadm', FAKE_UDEVADM)
    return bindir


def stage(base: Path, app_name: str):
    base.mkdir(parents=True, exist_ok=True)
    for name in ('bootloader.bin', 'partition-table.bin', 'ota_data_initial.bin', app_name):
        (base / name).write_bytes(bytes(range(32)))


def run_script(script, args, fake_bin, argv_log, extra_env=None):
    env = dict(os.environ)
    env['PATH'] = f'{fake_bin}:{env["PATH"]}'
    env['FAKE_ESPTOOL_PATH'] = str(FAKE_ESPTOOL)
    env['FAKE_ESPTOOL_ARGV_LOG'] = str(argv_log)
    if extra_env:
        env.update(extra_env)
    return subprocess.run(
        ['bash', str(script), *args],
        cwd=script.parent,
        env=env,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        timeout=30,
    )


def _expect_write_flash_argv(port, baud, img_dir, app_name):
    return [
        '--chip', 'esp32s3',
        '-p', port,
        '-b', baud,
        '--before', 'default-reset',
        '--after', 'hard-reset',
        'write-flash',
        '--flash-mode', 'dio',
        '--flash-size', '8MB',
        '--flash-freq', '80m',
        '0x0', str(img_dir / 'bootloader.bin'),
        '0x8000', str(img_dir / 'partition-table.bin'),
        '0x19000', str(img_dir / 'ota_data_initial.bin'),
        '0x20000', str(img_dir / app_name),
    ]  # fmt: skip


def test_remote_role_flashes_pstop_remote_bin(script, fake_bin, tmp_path):
    """--remote flashes production_image/pstop_remote.bin at 0x20000."""
    img = script.parent / 'production_image'
    stage(img, 'pstop_remote.bin')
    log = tmp_path / 'argv.jsonl'
    r = run_script(script, ['--remote', '/dev/ttyACM0'], fake_bin, log)
    assert r.returncode == 0, r.stderr
    [entry] = [e for e in read_argv_log(log) if 'write-flash' in e['argv']]
    assert entry['argv'] == _expect_write_flash_argv('/dev/ttyACM0', '460800', img, 'pstop_remote.bin')


def test_machine_role_flashes_machn_machine_bin(script, fake_bin, tmp_path):
    """--machine flashes production_image_machn/machn_machine.bin, not the remote app —
    this is the point of having a separate role/dir/app mapping at all."""
    img = script.parent / 'production_image_machn'
    stage(img, 'machn_machine.bin')
    log = tmp_path / 'argv.jsonl'
    r = run_script(script, ['--machine', '/dev/ttyACM0'], fake_bin, log)
    assert r.returncode == 0, r.stderr
    [entry] = [e for e in read_argv_log(log) if 'write-flash' in e['argv']]
    assert entry['argv'] == _expect_write_flash_argv('/dev/ttyACM0', '460800', img, 'machn_machine.bin')


def test_erase_runs_before_write_flash(script, fake_bin, tmp_path):
    """--erase erases the whole chip before writing; without it, no erase at all."""
    img = script.parent / 'production_image'
    stage(img, 'pstop_remote.bin')

    log_erase = tmp_path / 'erase.jsonl'
    r = run_script(script, ['--erase', '--remote', '/dev/ttyACM0'], fake_bin, log_erase)
    assert r.returncode == 0, r.stderr
    entries = read_argv_log(log_erase)
    erase_i = next(i for i, e in enumerate(entries) if 'erase-flash' in e['argv'])
    write_i = next(i for i, e in enumerate(entries) if 'write-flash' in e['argv'])
    assert erase_i < write_i

    log_no_erase = tmp_path / 'no_erase.jsonl'
    r = run_script(script, ['--remote', '/dev/ttyACM0'], fake_bin, log_no_erase)
    assert r.returncode == 0, r.stderr
    assert all('erase-flash' not in e['argv'] for e in read_argv_log(log_no_erase))


def test_running_unit_is_refused_without_flashing(script, fake_bin, tmp_path):
    """A port reporting the running-app PID (4001) is refused before any write —
    esptool can't talk ROM protocol to live firmware."""
    img = script.parent / 'production_image'
    stage(img, 'pstop_remote.bin')
    log = tmp_path / 'argv.jsonl'
    r = run_script(script, ['--remote', '/dev/ttyACM0'], fake_bin, log, extra_env={'FAKE_UDEV_PID': '4001'})
    assert r.returncode != 0
    assert 'not in download mode' in (r.stdout + r.stderr) or 'RUNNING' in (r.stdout + r.stderr)
    assert all('write-flash' not in e['argv'] for e in read_argv_log(log))


def test_no_role_and_no_tty_is_rejected(script, fake_bin, tmp_path):
    """With stdin not a TTY, the script must not block waiting for an answer;
    it must fail fast and name --remote/--machine."""
    log = tmp_path / 'argv.jsonl'
    r = run_script(script, [], fake_bin, log)
    assert r.returncode != 0
    assert '--remote' in r.stderr and '--machine' in r.stderr


def test_missing_staged_image_is_rejected_before_esptool(script, fake_bin, tmp_path):
    """A role whose staged directory is missing a file fails naming that file,
    before esptool is invoked at all."""
    img = script.parent / 'production_image'
    img.mkdir(parents=True)
    for name in ('bootloader.bin', 'partition-table.bin'):  # ota_data_initial.bin and app missing
        (img / name).write_bytes(b'\x00')
    log = tmp_path / 'argv.jsonl'
    r = run_script(script, ['--remote', '/dev/ttyACM0'], fake_bin, log)
    assert r.returncode != 0
    assert 'ota_data_initial.bin' in r.stderr
    assert read_argv_log(log) == []


def test_baud_flag_reaches_esptool(script, fake_bin, tmp_path):
    """--baud overrides the default 460800 baud passed to esptool's -b."""
    img = script.parent / 'production_image'
    stage(img, 'pstop_remote.bin')
    log = tmp_path / 'argv.jsonl'
    r = run_script(script, ['--baud', '115200', '--remote', '/dev/ttyACM0'], fake_bin, log)
    assert r.returncode == 0, r.stderr
    [entry] = [e for e in read_argv_log(log) if 'write-flash' in e['argv']]
    assert '-b' in entry['argv']
    assert entry['argv'][entry['argv'].index('-b') + 1] == '115200'
