# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Button press/release through the real E-stop loops, asserted at a real
machine instance over the real transport."""

import time

from conftest import do_arming_gesture
from rig import DEFERRED_RE, RX_STOP_RE, STATUS_OK_RE, STATUS_STOP_RE


def test_fresh_machine_needs_gesture(wired, machine, rig, cfg):
    """A freshly bonded remote streaming OK must NOT arm the machine."""
    time.sleep(1.5)
    assert machine.robot_status() != "OK", (
        "machine armed from a fresh bond with no press/release gesture"
    )


def test_press_stops_release_arms(wired, machine, rig, cfg):
    """The full gesture: press -> STOP on the wire; release -> armed OK,
    never sooner than min_stop_ms after the STOP."""
    since = machine.mark()
    rig.press()
    _, t_stop, _ = machine.wait_for(RX_STOP_RE, timeout=3.0, since=since)

    time.sleep(cfg["timing"]["min_stop_ms"] / 1000 + 0.3)
    rig.release()
    _, t_ok, _ = machine.wait_for(STATUS_OK_RE, timeout=5.0, since=since)

    held_ms = (t_ok - t_stop) * 1000
    min_stop = cfg["timing"]["min_stop_ms"]
    assert held_ms >= min_stop * 0.9, (
        f"armed {held_ms:.0f} ms after STOP — violates min_stop_ms={min_stop}"
    )


def test_hold_keeps_machine_stopped(wired, machine, rig, cfg):
    """While the button is held the robot stays STOP the whole time."""
    since = machine.mark()
    rig.press()
    machine.wait_for(RX_STOP_RE, timeout=3.0, since=since)
    time.sleep(2.0)
    assert machine.robot_status(since) != "OK"
    rig.release()
    time.sleep(1.0)


def test_short_blip_defers_arming(wired, machine, rig, cfg):
    """A press shorter than min_stop_ms must not arm early: the machine
    defers (ANOMALY) and completes the same episode only once min_stop has
    elapsed since the last STOP (pstop_c #59 native semantics)."""
    min_stop_ms = cfg["timing"]["min_stop_ms"]
    since = machine.mark()
    rig.press()
    _, t_stop, _ = machine.wait_for(RX_STOP_RE, timeout=3.0, since=since)
    rig.release()  # released well inside min_stop

    machine.wait_for(DEFERRED_RE, timeout=3.0, since=since)

    # The episode must still complete on steady OKs — but never early.
    _, t_ok, _ = machine.wait_for(STATUS_OK_RE, timeout=min_stop_ms / 1000 + 5.0, since=since)
    held_ms = (t_ok - t_stop) * 1000
    assert held_ms >= min_stop_ms * 0.9, f"blip armed after only {held_ms:.0f} ms"


def test_rearm_after_stop_while_armed(wired, machine, rig, cfg):
    """Armed robot: press stops it; release re-arms after min_stop."""
    do_arming_gesture(machine, rig, cfg)

    since = machine.mark()
    rig.press()
    machine.wait_for(STATUS_STOP_RE, timeout=3.0, since=since)

    time.sleep(cfg["timing"]["min_stop_ms"] / 1000 + 0.3)
    rig.release()
    machine.wait_for(STATUS_OK_RE, timeout=5.0, since=machine.mark())
