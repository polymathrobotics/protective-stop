#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Discover and run the change-control self-test modules present in this package."""

import sys
import unittest
from pathlib import Path


def main():
    suite = unittest.defaultTestLoader.discover(Path(__file__).parent, pattern='test_*.py')
    return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1


if __name__ == '__main__':
    sys.exit(main())
