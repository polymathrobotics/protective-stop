# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Regression tests for flash_station.py's esptool version gate, MAC read,
and flash-argument construction, against fake_esptool.py — no hardware."""

import os
import sys

import flash_station
import pytest
from conftest import FAKE_ESPTOOL, read_argv_log


def _cmd():
    return [sys.executable, str(FAKE_ESPTOOL)]


# --- esptool_cmd: version gate --------------------------------------------


def test_esptool_cmd_accepts_v5(monkeypatch):
    """A v5 shim is accepted and its candidate returned unchanged."""
    monkeypatch.setenv('FAKE_ESPTOOL_VERSION', '5.4.0')
    candidate = _cmd()
    assert flash_station.esptool_cmd(candidate=candidate) == candidate


def test_esptool_cmd_rejects_v4(monkeypatch):
    """The v4.12.0 esptool ESP-IDF 5.5 ships is refused, naming the required major."""
    monkeypatch.setenv('FAKE_ESPTOOL_VERSION', '4.12.0')
    with pytest.raises(SystemExit) as exc:
        flash_station.esptool_cmd(candidate=_cmd())
    assert f'v{flash_station.ESPTOOL_MIN_MAJOR}' in str(exc.value)


def test_esptool_cmd_rejects_nonzero_version_rc(monkeypatch):
    """A `version` subcommand that itself fails to run is refused, pointing at the uv environment."""
    monkeypatch.setenv('FAKE_ESPTOOL_VERSION_RC', '1')
    with pytest.raises(SystemExit) as exc:
        flash_station.esptool_cmd(candidate=_cmd())
    assert 'uv' in str(exc.value)


def test_esptool_cmd_rejects_unparseable_version(monkeypatch):
    """Output with no major.minor number is treated the same as no esptool at all."""
    monkeypatch.setenv('FAKE_ESPTOOL_VERSION', 'unknown')
    with pytest.raises(SystemExit):
        flash_station.esptool_cmd(candidate=_cmd())


def test_esptool_version_missing_binary_returns_none():
    """A command that can't even run (no such executable) is None, not an exception."""
    assert flash_station._esptool_version(['/nonexistent/binary/does-not-exist-xyz']) is None


# --- read_mac ---------------------------------------------------------------


def test_read_mac_parses_last_three_bytes(fake_cmd, argv_log, monkeypatch):
    """mac24 is the LAST 3 bytes of the MAC — a mismatched first/last byte
    catches a wrong slice direction."""
    monkeypatch.setenv('FAKE_ESPTOOL_MAC', 'de:ad:be:ef:12:34')
    mac, mac24 = flash_station.read_mac(fake_cmd, '/dev/ttyACM0')
    assert mac == 'de:ad:be:ef:12:34'
    assert mac24 == 'ef1234'
    [entry] = read_argv_log(argv_log)
    assert entry['argv'] == [
        '--chip', flash_station.CHIP,
        '--connect-attempts', flash_station.CONNECT_ATTEMPTS,
        '-p', '/dev/ttyACM0',
        'read-mac',
    ]  # fmt: skip


def test_read_mac_retries_then_succeeds(fake_cmd, argv_log, monkeypatch):
    """The first sync failure is absorbed by the outer retry, not surfaced."""
    monkeypatch.setenv('FAKE_ESPTOOL_FAIL_FIRST', '1')
    mac, mac24 = flash_station.read_mac(fake_cmd, '/dev/ttyACM0', tries=2)
    assert mac24 is not None
    assert len(read_argv_log(argv_log)) == 2


def test_read_mac_gives_up_without_raising(fake_cmd, argv_log, monkeypatch):
    """Exhausting every retry returns (None, None) rather than raising."""
    monkeypatch.setenv('FAKE_ESPTOOL_FAIL_FIRST', '99')
    mac, mac24 = flash_station.read_mac(fake_cmd, '/dev/ttyACM0', tries=2)
    assert (mac, mac24) == (None, None)
    assert len(read_argv_log(argv_log)) == 2


# --- flash: staged images --------------------------------------------------


@pytest.fixture
def staged_images(tmp_path, monkeypatch):
    """Dummy images for every entry in flash_station.IMAGES, staged under a
    tmp_path directory monkeypatched in as flash_station.IMG."""
    for _, fn in flash_station.IMAGES:
        (tmp_path / fn).write_bytes(bytes(range(64)))
    monkeypatch.setattr(flash_station, 'IMG', str(tmp_path))
    return tmp_path


# The flash layout as literals. These must NOT be derived from
# flash_station.IMAGES: the offsets are a contract with the partition table and
# with flash_pstop.sh, so an expectation computed from the value under test
# would follow a wrong edit instead of catching it.
EXPECTED_LAYOUT = (
    (0x0, 'bootloader.bin'),
    (0x8000, 'partition-table.bin'),
    (0x19000, 'ota_data_initial.bin'),
    (0x20000, 'pstop_remote.bin'),
)


def test_images_match_the_expected_flash_layout():
    """IMAGES is pinned offset-by-offset; flash_pstop.sh and the partition table
    carry the same numbers, so a change here must be a deliberate wire break."""
    assert tuple(flash_station.IMAGES) == EXPECTED_LAYOUT


def _expected_write_flash_argv(port, img_dir):
    argv = [
        '--chip', 'esp32s3',
        '--connect-attempts', flash_station.CONNECT_ATTEMPTS,
        '-p', port,
        '-b', '460800',
        '--before', 'default-reset',
        '--after', 'hard-reset',
        'write-flash',
        '--flash-mode', 'dio',
        '--flash-size', '8MB',
        '--flash-freq', '80m',
    ]  # fmt: skip
    for base, fn in EXPECTED_LAYOUT:
        argv += [hex(base), os.path.join(str(img_dir), fn)]
    return argv


def test_flash_builds_complete_write_flash_argv(fake_cmd, argv_log, staged_images):
    """Every image is paired with its own literal offset, and every required
    flag is present — not just 'write-flash' somewhere in the argv."""
    ok, err = flash_station.flash(fake_cmd, '/dev/ttyACM0', erase=False, on_progress=lambda f, p: None)
    assert ok, err
    [entry] = read_argv_log(argv_log)
    assert entry['argv'] == _expected_write_flash_argv('/dev/ttyACM0', staged_images)


def test_flash_without_erase_issues_no_erase_flash(fake_cmd, argv_log, staged_images):
    """erase=False must never touch erase-flash."""
    ok, _ = flash_station.flash(fake_cmd, '/dev/ttyACM0', erase=False, on_progress=lambda f, p: None)
    assert ok
    assert all('erase-flash' not in e['argv'] for e in read_argv_log(argv_log))


def test_flash_with_erase_runs_erase_before_write(fake_cmd, argv_log, staged_images):
    """erase=True must erase BEFORE writing — order, not just presence of both."""
    ok, _ = flash_station.flash(fake_cmd, '/dev/ttyACM0', erase=True, on_progress=lambda f, p: None)
    assert ok
    entries = read_argv_log(argv_log)
    erase_i = next(i for i, e in enumerate(entries) if 'erase-flash' in e['argv'])
    write_i = next(i for i, e in enumerate(entries) if 'write-flash' in e['argv'])
    assert erase_i < write_i


def test_flash_retries_whole_write_on_transient_failure(fake_cmd, argv_log, staged_images, monkeypatch):
    """A single failed write-flash is retried as a whole (idempotent re-flash), not surfaced."""
    monkeypatch.setenv('FAKE_ESPTOOL_FAIL_FIRST', '1')
    ok, err = flash_station.flash(fake_cmd, '/dev/ttyACM0', erase=False, on_progress=lambda f, p: None)
    assert ok, err
    assert len(read_argv_log(argv_log)) == 2


def test_flash_gives_up_after_flash_tries(fake_cmd, argv_log, staged_images, monkeypatch):
    """Every attempt failing exhausts FLASH_TRIES exactly and reports the try count."""
    monkeypatch.setenv('FAKE_ESPTOOL_FAIL_FIRST', '99')
    ok, err = flash_station.flash(fake_cmd, '/dev/ttyACM0', erase=False, on_progress=lambda f, p: None)
    assert not ok
    assert str(flash_station.FLASH_TRIES) in err
    assert len(read_argv_log(argv_log)) == flash_station.FLASH_TRIES


def test_flash_drives_on_progress_monotonically(fake_cmd, staged_images):
    """on_progress must see a non-decreasing sweep that actually leaves 0, not just the terminal 1.0."""
    fractions = []
    ok, err = flash_station.flash(
        fake_cmd, '/dev/ttyACM0', erase=False, on_progress=lambda f, phase: fractions.append(f)
    )
    assert ok, err
    assert all(0.0 <= f <= 1.0 for f in fractions)
    assert all(a <= b for a, b in zip(fractions, fractions[1:]))
    assert max(fractions) > 0.0


def test_flash_refuses_early_on_missing_image(fake_cmd, argv_log, tmp_path, monkeypatch):
    """A missing staged image is caught before esptool is ever invoked."""
    for _, fn in flash_station.IMAGES[:-1]:  # leave the last image un-staged
        (tmp_path / fn).write_bytes(b'\x00')
    monkeypatch.setattr(flash_station, 'IMG', str(tmp_path))
    ok, err = flash_station.flash(fake_cmd, '/dev/ttyACM0', erase=False, on_progress=lambda f, p: None)
    assert not ok
    assert 'missing' in err
    assert read_argv_log(argv_log) == []


# --- secrets partition -----------------------------------------------------


def test_flash_writes_extra_images_in_address_order(fake_cmd, argv_log, staged_images, tmp_path):
    """The per-unit secrets image lands at its own offset, between otadata and the app."""
    secrets_bin = tmp_path / 'secrets.bin'
    secrets_bin.write_bytes(bytes(64))
    ok, err = flash_station.flash(
        fake_cmd, '/dev/ttyACM0', erase=False, on_progress=lambda f, p: None, extra=[(0x1C000, str(secrets_bin))]
    )
    assert ok, err
    [entry] = read_argv_log(argv_log)
    pairs = [(a, entry['argv'][i + 1]) for i, a in enumerate(entry['argv']) if a.startswith('0x')]
    assert [a for a, _ in pairs] == ['0x0', '0x8000', '0x19000', '0x1c000', '0x20000']
    assert pairs[3][1] == str(secrets_bin)


def test_flash_one_fails_the_unit_before_writing_when_secrets_are_refused(
    fake_cmd, argv_log, staged_images, tmp_path, monkeypatch
):
    """A unit whose eFuse block holds another host's key is failed at the
    secrets stage; write-flash never runs."""
    import provision_secrets

    virt = provision_secrets.espefuse_base(None, tmp_path / 'efuse.bin')
    creds = 'ADMIN_PASSWORD=bench-pw\n'
    provision_secrets.provision(virt, creds, tmp_path / 'other_host', log=lambda m: None)
    monkeypatch.setattr(provision_secrets, 'espefuse_base', lambda port: virt)
    monkeypatch.setattr(flash_station, 'ts_status_hosts', lambda: {})

    cfg = {'credentials': creds, 'keys_dir': tmp_path / 'keys', 'key_block': provision_secrets.DEFAULT_KEY_BLOCK}
    ok, info = flash_station.flash_one(fake_cmd, '/dev/ttyACM0', False, 1, secrets_cfg=cfg)
    assert not ok
    assert info['stage'] == 'secrets'
    assert all('write-flash' not in e['argv'] for e in read_argv_log(argv_log))
