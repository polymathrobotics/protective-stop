<!--
SPDX-FileCopyrightText: 2026 Polymath Robotics
SPDX-License-Identifier: Apache-2.0
-->

# pstop tools

Flashing, fleet, chaos, soak, and HIL tooling for the remote and machine nodes.
Every Python tool here runs from one environment, described by
[`pyproject.toml`](pyproject.toml) and pinned by `uv.lock`.

## First-time setup

These tools use [uv](https://docs.astral.sh/uv/), a Python package and
environment manager.
It replaces `pip` and `venv`: you never create or activate a virtualenv by hand,
and everyone gets the identical dependency versions recorded in `uv.lock`.

### 1. Install uv

```sh
curl -LsSf https://astral.sh/uv/install.sh | sh
```

The installer puts `uv` in `~/.local/bin`.
Open a new shell afterwards, or `source ~/.local/bin/env`, then check it:

```sh
uv --version
```

macOS users can use `brew install uv` instead; Windows and other install methods
are in the [uv installation docs](https://docs.astral.sh/uv/getting-started/installation/).

### 2. Create the environment

```sh
cd tools
uv sync
```

This creates `tools/.venv` and installs the locked dependencies into it —
one environment for every tool here, flashing through HIL.
It takes a few seconds and needs no further configuration.
You do not need Python installed already — uv fetches an interpreter if the
system one is older than 3.11.

## Running a tool

Prefix any command with `uv run`.
There is no virtualenv to activate.

```sh
cd tools
uv run python flash_station.py --selftest
uv run python pstop_test_remote.py --port 8890
uv run esptool version
```

`uv run` re-checks the lock before each command, so a dependency added by a
teammate is installed automatically on your next run.

The shell tools need no prefix.
They re-enter this environment through `uv run` for each call, so they always
get the locked `esptool`:

```sh
tools/flash_pstop.sh --remote
```

## Flashing a unit

`flash_pstop.sh` provisions one device over USB from a staged image directory;
`flash_station.py` runs the unattended production loop.
Both need a staged image in `tools/production_image/` (remote) or
`tools/production_image_machn/` (machine), which carries per-fleet secrets and
is git-ignored.
See the header comment in each script for the full argument list.

## HIL suite

The rig tests live in [`hil/`](hil/).
Run them through `hil/run.sh`, which strips `PYTHONPATH` — a sourced ROS
environment otherwise puts broken plugins on it:

```sh
cd tools/hil
./run.sh                    # everything, power cycles included
./run.sh -m 'not power'     # skip the power-cycle tests
./run.sh test_00_rig.py     # rig self-check after (re)wiring
```

## Safety traceability linter

`safety_lint/` parses the safety documents, checks the requirement, function,
and evidence mappings, and owns the generated coverage numbers in
`docs/safety/TRACEABILITY.md`.
It is stdlib-only but runs from this environment like everything else here.
It resolves the safety documents against the repository root it is checked into,
so the working directory does not matter:

```sh
cd tools
uv run python -m safety_lint --check    # CI's gate
uv run python -m safety_lint --write    # refresh coverage
```

## Tests

`pyproject.toml` holds the only pytest configuration. The default run is the
suites that need no hardware, so it passes on any checkout:

```sh
cd tools
uv run pytest          # linter self-test + the flashing tests; no bench needed
```

The rig tests are opt-in, because their fixtures fail rather than skip when no
relay board is attached:

```sh
tools/hil/run.sh       # the HIL suite; needs the bench (see hil/README.md)
```

The flat scripts are deliberately outside that: several take a positional
integer or exit at import, so they are not collectible, and the repo-root
[`../test/`](../test/) ladders plus the `pstop_*_test.py` harnesses are bash-
and script-driven. See
[`../docs/TESTING.md`](../docs/TESTING.md) for those.

### Flashing-tool regression tests

`tools/test/` covers `flash_station.py` and `flash_pstop.sh` against
`test/fake_esptool.py`, a shim that stands in for the real esptool.
No hardware is needed; it never opens a port.

```sh
cd tools
uv run pytest test/
```

This is part of the default `uv run pytest` collection above, so a plain run
from `tools/` already includes it.

## The one exception: `soak_per_remote.py` in ROS mode

`soak_per_remote.py` reads remote state from either the chip's HTTP
`state.json` or a ROS 2 topic.
The HTTP source is stdlib-only and runs under `uv run` like everything else.
The ROS source imports `rclpy` and `protective_stop_msg` lazily, and those come
from a sourced ROS workspace rather than PyPI, so the uv environment cannot
provide them:

```sh
source /opt/ros/$ROS_DISTRO/setup.bash
source <workspace>/install/setup.bash
python3 tools/soak_per_remote.py ...      # system python, not uv run
```

## Adding a dependency

```sh
cd tools
uv add <package>
```

`uv add` edits `pyproject.toml` and updates `uv.lock`.
Commit both.
