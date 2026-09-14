# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Load all canonical linter inputs as one analysis snapshot."""

import re
from pathlib import Path

from .model import Analysis, LintError
from .parse_srs import convention_statuses, parse_srs
from .parse_system_definition import parse_system_definition
from .parse_traceability import parse_traceability


def _ids(path, pattern):
    if not path.is_file():
        raise LintError(f'required document missing: {path}')
    return frozenset(re.findall(pattern, path.read_text(encoding='utf-8')))


def analyze(root):
    root = Path(root).resolve()
    safety = root / 'docs/safety'
    srs_path = safety / 'SAFETY_REQUIREMENTS.md'
    trace, reverse, issues = parse_traceability(root)
    return Analysis(
        root,
        parse_srs(srs_path),
        trace,
        parse_system_definition(safety / 'SYSTEM_DEFINITION.md'),
        reverse,
        issues,
        convention_statuses(srs_path),
        _ids(safety / 'HARA.md', r'\b(?:SG-\d+|H-\d{2})\b'),
        _ids(safety / 'FMEA.md', r'\bDU-\d+\b'),
    )
