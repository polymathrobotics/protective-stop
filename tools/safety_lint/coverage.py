# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Compute citation, status, and function coverage from parsed records."""

from collections import Counter
from dataclasses import dataclass


@dataclass(frozen=True)
class Coverage:
    total: int
    cited_tests: int
    verified: int
    areas: dict[str, dict[str, int]]
    functions_total: int
    functions_traced: int
    safety_functions_traced: int
    safety_functions_total: int


def compute_coverage(analysis):
    areas = {}
    for area in ('SYS', 'R', 'H', 'M', 'I'):
        rows = [row for row in analysis.trace if row.sr_id.split('-')[1] == area]
        counts = Counter(row.status for row in rows)
        areas[area] = {
            'count': len(rows),
            'Verified': counts['Verified'],
            'Partially-verified': counts['Partially-verified'],
            'Unverified-gap': counts['Unverified-gap'],
            'Residual-accepted': counts['Residual-accepted'],
            'cited': sum(bool(row.test_refs) for row in rows),
        }
    traced = sum(bool(entry.sr_ids) for entry in analysis.reverse.values() if entry.function_id in analysis.functions)
    non_safety = sum(
        entry.declared_non_safety for entry in analysis.reverse.values() if entry.function_id in analysis.functions
    )
    safety_traced = sum(
        bool(entry.sr_ids)
        for entry in analysis.reverse.values()
        if entry.function_id in analysis.functions and not entry.declared_non_safety
    )
    return Coverage(
        len(analysis.trace),
        sum(bool(row.test_refs) for row in analysis.trace),
        sum(row.status == 'Verified' for row in analysis.trace),
        areas,
        len(analysis.functions),
        traced,
        safety_traced,
        len(analysis.functions) - non_safety,
    )
