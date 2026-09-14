# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Immutable records shared by the safety traceability linter."""

from dataclasses import dataclass
from pathlib import Path


class LintError(Exception):
    """A required input is absent or structurally unparseable."""


@dataclass(frozen=True)
class SafetyRequirement:
    sr_id: str
    area: str
    number: int
    shall_text: str
    derived_from: tuple[str, ...]
    allocated_to: tuple[str, ...]
    integrity: str
    verify_methods: tuple[str, ...]
    status: str
    status_prose: str
    source_line: int


@dataclass(frozen=True)
class TraceRow:
    sr_id: str
    allocated_to: tuple[str, ...]
    code_refs: tuple[str, ...]
    test_refs: tuple[str, ...]
    methods: tuple[str, ...]
    status: str
    status_prose: str
    has_no_test_marker: bool
    source_line: int
    raw_test_cell: str


@dataclass(frozen=True)
class Function:
    function_id: str
    name: str
    source_line: int


@dataclass(frozen=True)
class ReverseEntry:
    function_id: str
    name: str
    sr_ids: tuple[str, ...]
    declared_non_safety: bool
    source_line: int


@dataclass(frozen=True)
class ResolutionIssue:
    kind: str
    sr_id: str
    literal: str
    message: str
    source_line: int
    category: str


@dataclass(frozen=True)
class Finding:
    check_id: str
    severity: str
    subject: str
    message: str
    file: str
    line: int


@dataclass(frozen=True)
class Analysis:
    root: Path
    srs: tuple[SafetyRequirement, ...]
    trace: tuple[TraceRow, ...]
    functions: dict[str, Function]
    reverse: dict[str, ReverseEntry]
    issues: tuple[ResolutionIssue, ...]
    conventions_statuses: frozenset[str]
    hara_ids: frozenset[str]
    fmea_ids: frozenset[str]
