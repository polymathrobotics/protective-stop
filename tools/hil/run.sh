#!/bin/sh
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
# Run the HIL suite in the tools/ uv environment, isolated from ROS PYTHONPATH
# plugins.
cd "$(dirname "$0")"
PYTHONPATH= exec env -u VIRTUAL_ENV uv run --project .. python -m pytest "$@"
