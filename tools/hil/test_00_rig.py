# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Rig self-tests: relay board, chip API, and loop wiring — run these first
after any bench change; everything else assumes they pass."""

import time


def test_relay_board_roundtrip(rig):
    """Every relay channel acks and reads back both states — EXCEPT the
    DUT power channel, which must not be blipped outside the power tests
    (toggling it reboots the chip and poisons the tests that follow)."""
    board = rig.board
    for ch in (1, 2, 3, 4):
        if ch == rig.power_ch:
            continue
        before = board.status()[ch]
        board.set(ch, not before)
        assert board.status()[ch] == (not before)
        board.set(ch, before)
        assert board.status()[ch] == before
    rig.idle()


def test_chip_state_has_loop_health(chip):
    s = chip.state()
    for key in ('e_hi0', 'e_lo0', 'e_hi1', 'e_lo1', 'pstop_mismatch', 'uptime_ms'):
        assert key in s, f'state.json missing {key}'


def test_both_loops_wired(rig, chip, wired):
    """Each relay opens exactly its own loop (A relay must not touch B)."""
    for mine, other, my_idx in (('a', 'b', 0), ('b', 'a', 1)):
        rig.open_loop(mine)
        s = chip.wait_state(
            lambda s: not (s[f'e_hi{my_idx}'] and s[f'e_lo{my_idx}']),
            timeout=3.0,
            what=f'loop {mine.upper()} open',
        )
        other_idx = 1 - my_idx
        assert s[f'e_hi{other_idx}'] and s[f'e_lo{other_idx}'], (
            f'opening loop {mine.upper()} also opened {other.upper()} — miswired'
        )
        rig.close_loop(mine)
        chip.wait_state(
            lambda s: s[f'e_hi{my_idx}'] and s[f'e_lo{my_idx}'],
            timeout=3.0,
            what=f'loop {mine.upper()} closed',
        )
    time.sleep(0.5)


def test_firmware_reports_identity(chip):
    """The DUT reports which build it runs (fw_ver + fw_sha) — the CI gate
    (`--expect-fw`) depends on it. Skips (with a nudge) on firmware older
    than 4c90711; under `--expect-fw` the session already hard-failed."""
    import pytest

    fw = chip.fw()
    if fw is None:
        pytest.skip('DUT firmware predates fw_ver/fw_sha (< 4c90711) — update it')
    ver, sha = fw
    assert ver and sha, f'empty firmware identity: fw_ver={ver!r} fw_sha={sha!r}'
    print(f'DUT running fw_ver={ver} fw_sha={sha}')
