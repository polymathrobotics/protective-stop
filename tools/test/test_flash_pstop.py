# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Regression tests for flash_pstop.sh, run as a real bash subprocess against
fake_esptool.py and a fake udevadm/uv on PATH — no hardware, no real device."""

import os
import re
import shutil
import stat
import subprocess
import sys
from pathlib import Path

import pytest
from conftest import FAKE_ESPTOOL, read_argv_log

pytestmark = pytest.mark.skipif(shutil.which('bash') is None, reason='bash is not available on this runner')

REPO_TOOLS = Path(__file__).resolve().parents[1]
SCRIPT_SRC = REPO_TOOLS / 'flash_pstop.sh'
CREDENTIALS = 'TAILSCALE_AUTH_KEY="tskey-auth-kTEST1-abcdefghijklmnop"\nADMIN_PASSWORD="bench-pw"\n'

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
  exec python3 "$FAKE_ESPTOOL_PATH" "$@"
fi
if [ "${1:-}" = "python" ]; then
  shift
  args=()
  while [ $# -gt 0 ]; do
    if [ "$1" = "--port" ]; then args+=(--virt-efuse "$FAKE_EFUSE_FILE"); shift 2; else args+=("$1"); shift; fi
  done
  exec "$REAL_PYTHON" "${args[@]}"
fi
echo "fake uv: unsupported run target: $*" >&2
exit 1
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
    """A real copy of flash_pstop.sh, run in place (not edited), beside
    provision_secrets.py and a credentials.env."""
    dst = tmp_path / 'flash_pstop.sh'
    shutil.copy2(SCRIPT_SRC, dst)
    shutil.copy2(REPO_TOOLS / 'provision_secrets.py', tmp_path / 'provision_secrets.py')
    (tmp_path / 'credentials.env').write_text(CREDENTIALS)
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
    env['FAKE_EFUSE_FILE'] = str(script.parent / 'efuse.bin')
    env['REAL_PYTHON'] = sys.executable
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


def _expect_write_flash_argv(port, baud, img_dir, app_name, secrets=True):
    secrets_args = ['0x1C000', '<secrets.bin>'] if secrets else []
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
        *secrets_args,
        '0x20000', str(img_dir / app_name),
    ]  # fmt: skip


def _normalized(argv):
    """argv with the per-run temporary secrets image path replaced by a placeholder."""
    return ['<secrets.bin>' if re.search(r'/secrets\.bin$', a) else a for a in argv]


def test_remote_role_flashes_pstop_remote_bin(script, fake_bin, tmp_path):
    """--remote flashes production_image/pstop_remote.bin at 0x20000."""
    img = script.parent / 'production_image'
    stage(img, 'pstop_remote.bin')
    log = tmp_path / 'argv.jsonl'
    r = run_script(script, ['--remote', '/dev/ttyACM0'], fake_bin, log)
    assert r.returncode == 0, r.stderr
    [entry] = [e for e in read_argv_log(log) if 'write-flash' in e['argv']]
    assert _normalized(entry['argv']) == _expect_write_flash_argv('/dev/ttyACM0', '460800', img, 'pstop_remote.bin')


def test_machine_role_flashes_machn_machine_bin(script, fake_bin, tmp_path):
    """--machine flashes production_image_machn/machn_machine.bin, not the remote app —
    this is the point of having a separate role/dir/app mapping at all."""
    img = script.parent / 'production_image_machn'
    stage(img, 'machn_machine.bin')
    log = tmp_path / 'argv.jsonl'
    r = run_script(script, ['--machine', '/dev/ttyACM0'], fake_bin, log)
    assert r.returncode == 0, r.stderr
    [entry] = [e for e in read_argv_log(log) if 'write-flash' in e['argv']]
    assert _normalized(entry['argv']) == _expect_write_flash_argv('/dev/ttyACM0', '460800', img, 'machn_machine.bin')


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


# --- secrets partition -----------------------------------------------------


def test_secrets_image_is_provisioned_and_flashed(script, fake_bin, tmp_path):
    """The first flash burns the unit's key, keeps it in device_keys/, and writes
    a partition-sized image at 0x1C000 that is gone once the script exits."""
    stage(script.parent / 'production_image', 'pstop_remote.bin')
    log = tmp_path / 'argv.jsonl'
    r = run_script(script, ['--remote', '/dev/ttyACM0'], fake_bin, log)
    assert r.returncode == 0, r.stderr
    [key_file] = (script.parent / 'device_keys').iterdir()
    assert stat.S_IMODE(key_file.stat().st_mode) == 0o600
    [entry] = [e for e in read_argv_log(log) if 'write-flash' in e['argv']]
    image = entry['argv'][entry['argv'].index('0x1C000') + 1]
    assert image.endswith('/secrets.bin')
    assert not Path(image).exists()


def test_missing_credentials_is_rejected_before_esptool(script, fake_bin, tmp_path):
    stage(script.parent / 'production_image', 'pstop_remote.bin')
    (script.parent / 'credentials.env').unlink()
    log = tmp_path / 'argv.jsonl'
    r = run_script(script, ['--remote', '/dev/ttyACM0'], fake_bin, log)
    assert r.returncode != 0
    assert 'credentials.env' in r.stderr and '--no-secrets' in r.stderr
    assert read_argv_log(log) == []


def test_no_secrets_skips_provisioning(script, fake_bin, tmp_path):
    img = script.parent / 'production_image'
    stage(img, 'pstop_remote.bin')
    (script.parent / 'credentials.env').unlink()
    log = tmp_path / 'argv.jsonl'
    r = run_script(script, ['--no-secrets', '--remote', '/dev/ttyACM0'], fake_bin, log)
    assert r.returncode == 0, r.stderr
    [entry] = [e for e in read_argv_log(log) if 'write-flash' in e['argv']]
    assert entry['argv'] == _expect_write_flash_argv('/dev/ttyACM0', '460800', img, 'pstop_remote.bin', secrets=False)
    assert not (script.parent / 'device_keys').exists()


def test_bad_credentials_stop_before_erase_or_write(script, fake_bin, tmp_path):
    """A typo in credentials.env leaves the unit untouched: no burn, no erase, no write."""
    stage(script.parent / 'production_image', 'pstop_remote.bin')
    (script.parent / 'credentials.env').write_text('TAILSCALE_KEY=typo\n')
    log = tmp_path / 'argv.jsonl'
    r = run_script(script, ['--erase', '--remote', '/dev/ttyACM0'], fake_bin, log)
    assert r.returncode != 0
    assert 'unknown key' in r.stderr
    assert read_argv_log(log) == []
    assert not (script.parent / 'device_keys').exists()
