#!/bin/sh
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
# Run the HIL suite in its venv, isolated from ROS PYTHONPATH plugins.
cd "$(dirname "$0")"
[ -d .venv ] || { python3 -m venv .venv && ./.venv/bin/pip -q install pytest pyserial tomli; }
PYTHONPATH= exec ./.venv/bin/python -m pytest "$@"
