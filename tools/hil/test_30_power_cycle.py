# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Power-loss and cold-boot recovery. Cuts real USB power to the DUT, so
these run only with the power relay wired (-m power to select, deselect
with -m 'not power' for a quicker loop-only run)."""

import time

import pytest
from conftest import do_arming_gesture
from rig import LIVENESS_RE, RX_RE, STATUS_OK_RE

pytestmark = pytest.mark.power


def test_power_loss_stops_and_boot_does_not_rearm(wired, machine, rig, chip, cfg):
    t = cfg["timing"]
    do_arming_gesture(machine, rig, cfg)
    uptime_before = chip.state()["uptime_ms"]

    # Kill power: machine must stop on liveness within the heartbeat budget.
    since = machine.mark()
    rig.power_off()
    budget = t["heartbeat_ms"] * t["max_missed_heartbeats"] / 1000 + 2.0
    machine.wait_for(LIVENESS_RE, timeout=budget, since=since)
    assert machine.robot_status(since) == "STOP"
    chip.wait_gone(timeout=10.0)

    time.sleep(3.0)

    # Restore: chip must boot (uptime reset proves a real power cycle, not a
    # network blip), re-bond to the persisted HIL peer slot on its own...
    since = machine.mark()
    rig.power_on()
    s = chip.wait_reachable(timeout=t["boot_timeout_s"])
    assert s["uptime_ms"] < uptime_before, "uptime did not reset — no real power cycle"
    machine.wait_for(RX_RE, timeout=t["bond_timeout_s"], since=since)

    # ...and the robot must STAY stopped: a cold boot with the button
    # released must never complete the arming gesture (boot-priming hold).
    time.sleep(3.0)
    assert machine.robot_status(since) != "OK", (
        "machine re-armed after DUT power cycle with no operator action"
    )

    # A deliberate gesture arms it again.
    do_arming_gesture(machine, rig, cfg)


def test_boot_with_button_held_flows_stop(wired, machine, rig, chip, cfg):
    """Button held across a cold boot: STOP must flow once settled, and
    releasing then completes an arming gesture."""
    t = cfg["timing"]
    rig.press()
    time.sleep(0.5)

    rig.power_off()
    chip.wait_gone(timeout=10.0)
    time.sleep(3.0)

    since = machine.mark()
    rig.power_on()
    chip.wait_reachable(timeout=t["boot_timeout_s"])
    # The chip boots, settles its loops open (held), and must put STOP on
    # the wire — the machine sees the remote sending STOP.
    machine.wait_for(RX_RE, timeout=t["bond_timeout_s"], since=since)
    time.sleep(1.0)
    assert machine.robot_status(since) != "OK"

    time.sleep(t["min_stop_ms"] / 1000)
    rig.release()
    machine.wait_for(STATUS_OK_RE, timeout=5.0, since=since)
