# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Single-channel failure modes: one E-stop loop opens, the other stays
closed. The lockstep cores must DISAGREE, the comparator must send NOTHING,
and the machine must stop on heartbeat liveness — never see a spoofable
half-verdict, never stay armed."""

import time

import pytest
from conftest import do_arming_gesture
from rig import LIVENESS_RE, RX_STOP_RE


@pytest.mark.parametrize("channel,idx", [("a", 0), ("b", 1)])
def test_single_open_channel_stops_via_liveness(wired, machine, rig, chip, cfg, channel, idx):
    do_arming_gesture(machine, rig, cfg)
    mismatch_before = chip.state()["pstop_mismatch"]

    since = machine.mark()
    rig.open_loop(channel)

    # The machine must stop on MISSED_HEARTBEATS (comparator went silent) —
    # within heartbeat_ms x max_missed plus margin.
    t = cfg["timing"]
    budget = t["heartbeat_ms"] * t["max_missed_heartbeats"] / 1000 + 2.0
    machine.wait_for(LIVENESS_RE, timeout=budget, since=since)
    assert machine.robot_status(since) == "STOP"

    # It must NOT have been a STOP message: a single-channel fault makes the
    # cores disagree, so nothing goes on the wire.
    assert not machine.find(RX_STOP_RE, since=since), (
        f"chip sent a STOP with only channel {channel.upper()} open — "
        "comparator should have withheld"
    )

    # Chip's own view: exactly the faulted loop open, mismatches counted.
    s = chip.state()
    assert not (s[f"e_hi{idx}"] and s[f"e_lo{idx}"]), f"loop {channel.upper()} not open"
    other = 1 - idx
    assert s[f"e_hi{other}"] and s[f"e_lo{other}"], "healthy loop reads open"
    assert s["pstop_mismatch"] > mismatch_before, "lockstep mismatch not counted"

    # Healing the fault must NOT re-arm: recovery requires a deliberate
    # press->release gesture.
    since = machine.mark()
    rig.close_loop(channel)
    time.sleep(2.5)
    assert machine.robot_status(since) != "OK", (
        "machine re-armed on single-channel heal without an arming gesture"
    )

    do_arming_gesture(machine, rig, cfg)
