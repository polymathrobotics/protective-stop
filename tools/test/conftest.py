# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Fixtures shared by the flashing-tool regression tests."""

import json
import sys
from pathlib import Path

import pytest

TEST_DIR = Path(__file__).resolve().parent
TOOLS_DIR = TEST_DIR.parent
sys.path.insert(0, str(TOOLS_DIR))  # flash_station.py is a flat script, not a package

import flash_station  # noqa: E402

FAKE_ESPTOOL = TEST_DIR / 'fake_esptool.py'


def read_argv_log(path: Path) -> list:
    """Every invocation `fake_esptool.py` logged to `path`, in call order."""
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


@pytest.fixture
def argv_log(tmp_path) -> Path:
    """A fresh JSONL log path, unique per test, never shared or reused."""
    return tmp_path / 'argv.jsonl'


@pytest.fixture
def fake_cmd(monkeypatch, argv_log):
    """A `flash_station.esptool_cmd`-verified command list pointed at the fake
    esptool, with FAKE_ESPTOOL_ARGV_LOG wired to `argv_log` for the rest of
    the test. The version probe this triggers is never logged (see
    fake_esptool.py), so `argv_log` stays empty until the test's own calls."""
    monkeypatch.setenv('FAKE_ESPTOOL_ARGV_LOG', str(argv_log))
    return flash_station.esptool_cmd(candidate=[sys.executable, str(FAKE_ESPTOOL)])
