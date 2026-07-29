# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""pytest fixtures wiring the HIL rig together. See README.md for the rig."""

from __future__ import annotations

import sys

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib
from pathlib import Path

import pytest

HIL_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(HIL_DIR.parent))  # tools/ — for usb_relay4
sys.path.insert(0, str(HIL_DIR))

from rig import RX_RE, Chip, MachineRunner, RelayRig, discover_chip_ip  # noqa: E402
from usb_relay4 import UsbRelay4  # noqa: E402


def pytest_addoption(parser):
    parser.addoption(
        "--hil-config", default=str(HIL_DIR / "hil.toml"),
        help="rig description TOML (default: tools/hil/hil.toml)",
    )


@pytest.fixture(scope="session")
def cfg(request) -> dict:
    with open(request.config.getoption("--hil-config"), "rb") as f:
        return tomllib.load(f)


@pytest.fixture(scope="session")
def rig(cfg):
    """Relay board mapped onto rig functions. Enters and leaves bench-normal
    (loops closed, DUT powered)."""
    board = UsbRelay4()
    r = RelayRig(board, cfg["relays"])
    r.idle()
    yield r
    r.idle()
    board.close()


@pytest.fixture(scope="session")
def chip(cfg, rig) -> Chip:
    host = cfg["rig"]["chip_host"]
    if host == "auto":
        host = discover_chip_ip(cfg["rig"].get("ncm_iface", "esp-pstop0"))
    c = Chip(host)
    try:
        # Generous: the chip may still be booting from a previous run's
        # power-cycle teardown (USB enumerate + DHCP + httpd ~ 15 s).
        c.wait_reachable(timeout=30.0)
    except TimeoutError:
        pytest.skip(f"chip not reachable at {host}")
    return c


@pytest.fixture(scope="session")
def wired(cfg, rig, chip):
    """Prove the relays are actually in the E-stop loops: open loop A, watch
    the chip report it open, close it, watch it close. Skips the wired-only
    tests on an unwired bench instead of failing them confusingly."""
    try:
        rig.open_loop("a")
        chip.wait_state(
            lambda s: not (s["e_hi0"] and s["e_lo0"]), timeout=3.0,
            what="loop A open after relay",
        )
        rig.close_loop("a")
        chip.wait_state(
            lambda s: s["e_hi0"] and s["e_lo0"], timeout=3.0,
            what="loop A closed after relay",
        )
    except TimeoutError:
        rig.idle()
        pytest.skip("relay channel A does not move the chip's loop A — rig not wired")


@pytest.fixture
def machine(cfg, chip, rig, tmp_path):
    """Fresh dedicated machine instance per test: spawn the runner, claim the
    chip's HIL peer slot, wait for the chip's first message, and tear both
    down afterwards. Fresh per test => arming state starts clean."""
    r = cfg["rig"]
    m = MachineRunner(
        binary=HIL_DIR / r["machine_binary"],
        workdir=tmp_path,
        port=r["machine_port"],
        device_id=r["machine_device_id"],
        timing=cfg["timing"],
    )
    m.start()
    chip.set_peer_slot(r["peer_slot"], r["host_ip"], r["machine_port"], r["machine_device_id"])
    try:
        m.wait_for(RX_RE, timeout=cfg["timing"]["bond_timeout_s"])
    except TimeoutError:
        chip.clear_peer_slot(r["peer_slot"])
        m.stop()
        raise
    yield m
    try:
        chip.clear_peer_slot(r["peer_slot"])
    finally:
        m.stop()
        rig.idle()


def do_arming_gesture(machine: MachineRunner, rig: RelayRig, cfg: dict) -> None:
    """Press (>= min_stop), release, wait for ROBOT STATUS -> OK."""
    import time

    from rig import STATUS_OK_RE

    since = machine.mark()
    rig.press()
    time.sleep(cfg["timing"]["min_stop_ms"] / 1000 + 0.3)
    rig.release()
    machine.wait_for(STATUS_OK_RE, timeout=5.0, since=since)
