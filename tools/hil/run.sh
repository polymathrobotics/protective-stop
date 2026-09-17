#!/bin/sh
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
# Run the HIL suite in the tools/ uv environment, isolated from ROS PYTHONPATH
# plugins.
cd "$(dirname "$0")"
# Default to this directory so the rig suite runs alone; pytest config and
# markers come from tools/pyproject.toml.
[ $# -eq 0 ] && set -- .
PYTHONPATH= exec env -u VIRTUAL_ENV uv run --project .. python -m pytest "$@"
