# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Parse trace rows, reverse maps, and repository evidence citations."""

import os
import re
from collections import defaultdict
from pathlib import Path

from .checks import TRACE_STATUSES
from .model import LintError, ResolutionIssue, ReverseEntry, TraceRow
from .parse_srs import SR_RE, expand_allocations, normalize_status, split_row

SHORTHAND = {
    'EV': 'firmware/test/test_estop_verdict.c',
    'MR': 'tools/pstop_multi_remote_test.py',
    'HIL10': 'tools/hil/test_10_button.py',
    'HIL20': 'tools/hil/test_20_discordance.py',
    'HIL30': 'tools/hil/test_30_power_cycle.py',
    'JL': 'ros2/protective_stop_machine/test/test_json_lite.cpp',
}


def _repo_index(root):
    by_name = defaultdict(list)
    by_stem = defaultdict(list)
    all_paths = set()
    for directory, names, files in os.walk(root):
        relative_directory = Path(directory).relative_to(root).as_posix()
        if relative_directory == 'tools/safety_lint/fixtures':
            names[:] = []
            continue
        names[:] = [name for name in names if name not in {'.git', '.venv', '__pycache__', 'build', 'install', 'log'}]
        for filename in files:
            relative = Path(directory, filename).relative_to(root).as_posix()
            all_paths.add(relative)
            by_name[filename].append(relative)
            by_stem[Path(filename).stem].append(relative)
    return all_paths, by_name, by_stem


def _expand_sr_list(cell):
    result = []
    for match in re.finditer(r'SR-(SYS|R|H|M|I)-(\d{2}(?:/\d{2})*)', cell):
        area, numbers = match.groups()
        result.extend(f'SR-{area}-{number}' for number in numbers.split('/'))
    return tuple(dict.fromkeys(result))


def _test_refs(root, sr_id, cell, line, index):
    all_paths, by_name, by_stem = index
    # Text following NO TEST describes an uncovered leg, not verifying evidence.
    evidence_cell = cell.split('NO TEST', 1)[0]
    refs = []
    issues = []
    attached_symbols = set(
        re.findall(
            r'`[^`]+\.(?:c|cc|cpp|py):\d+(?:-\d+)?`\s*\(`(test_[A-Za-z0-9_]+)`\)',
            evidence_cell,
        )
    )

    for match in re.finditer(r'\bHIL(10|20|30)((?:/(?:10|20|30))*)', evidence_cell):
        numbers = (match.group(1), *match.group(2).lstrip('/').split('/'))
        for number in filter(None, numbers):
            token = f'HIL{number}'
            path = SHORTHAND[token]
            if path in all_paths:
                refs.append(path)
            else:
                issues.append(
                    ResolutionIssue(
                        'missing-shorthand', sr_id, token, f'shorthand resolves to missing {path}', line, 'test'
                    )
                )
    for token, path in SHORTHAND.items():
        if token.startswith('HIL'):
            continue
        if re.search(rf'\b{token}(?:\[[A-Z/]+\])?\b', evidence_cell):
            if path in all_paths:
                refs.append(path)
            else:
                issues.append(
                    ResolutionIssue(
                        'missing-shorthand', sr_id, token, f'shorthand resolves to missing {path}', line, 'test'
                    )
                )
    for match in re.finditer(r'\bREQ\s+(\d+_\d+(?:/\d+_\d+)*)', evidence_cell):
        for number in match.group(1).split('/'):
            path = f'pstop_c/pstop/test/src/pstop/requirements/req_{number}_test.c'
            if path in all_paths:
                refs.append(path)
            else:
                issues.append(
                    ResolutionIssue(
                        'missing-shorthand', sr_id, f'REQ {number}', f'REQ resolves to missing {path}', line, 'test'
                    )
                )

    literals = [literal for literal in re.findall(r'`([^`]+)`', evidence_cell) if _looks_like_test_citation(literal)]
    literals += re.findall(
        r'(?<![\w/`])((?:test_[A-Za-z0-9_]+|[A-Za-z0-9_]+_test)(?:\.(?:c|cc|cpp|py))?)(?![\w/`])', evidence_cell
    )
    literals += re.findall(
        r'(?<![`\w])((?:docs|tools|test|tests|firmware|host|machn|ros2)/[A-Za-z0-9_./-]+\.(?:c|cc|cpp|py|md))(?![\w])',
        evidence_cell,
    )
    literals += [token for token in re.findall(r'\btest_[A-Za-z0-9_]+\b', evidence_cell) if token in by_stem]
    for literal in dict.fromkeys(literals):
        literal = literal.rstrip('.,;:')
        if literal in SHORTHAND or literal.startswith('REQ ') or literal in attached_symbols:
            continue
        literal = re.sub(r':\d+(?:-\d+)?$', '', literal)
        candidates = []
        explicit = '/' in literal
        if explicit:
            candidates = [literal] if literal in all_paths else []
        elif '.' in Path(literal).name:
            candidates = by_name.get(Path(literal).name, [])
        else:
            candidates = by_stem.get(literal, [])
        if not explicit:
            candidates = [path for path in candidates if _is_test_artifact(path)]
        if len(candidates) == 1:
            path = candidates[0]
            if path.endswith('.md'):
                if path.startswith('pstop_c/'):
                    continue
                if sr_id not in (root / path).read_text(encoding='utf-8', errors='replace'):
                    issues.append(
                        ResolutionIssue(
                            'report-does-not-name-sr',
                            sr_id,
                            literal,
                            'evidence report does not name cited SR',
                            line,
                            'test',
                        )
                    )
                    continue
            refs.append(path)
        elif len(candidates) > 1:
            issues.append(
                ResolutionIssue(
                    'ambiguous', sr_id, literal, f'ambiguous evidence citation: {", ".join(candidates)}', line, 'test'
                )
            )
        elif explicit or _is_test_literal(literal):
            issues.append(
                ResolutionIssue('missing', sr_id, literal, 'cited evidence file does not exist', line, 'test')
            )
    return tuple(dict.fromkeys(refs)), tuple(issues)


def _is_test_artifact(path):
    """Return whether a bare candidate is independently recognizable as test evidence."""
    candidate = Path(path)
    lower_parts = {part.lower() for part in candidate.parts}
    stem = candidate.stem.lower()
    if candidate.suffix.lower() == '.md':
        return True
    return stem.startswith('test_') or stem.endswith('_test') or bool(lower_parts & {'test', 'tests', 'requirements'})


def _looks_like_test_citation(literal):
    without_line = re.sub(r':\d+(?:-\d+)?$', '', literal)
    return (
        ('/' in without_line and not without_line.startswith('/') and '.' in Path(without_line).name)
        or without_line.startswith('test_')
        or without_line.endswith(('.c', '.cc', '.cpp', '.py', '.md'))
    )


def _is_test_literal(literal):
    stem = Path(literal).stem.lower()
    return stem.startswith('test_') or stem.endswith('_test')


def _code_refs(sr_id, cell, line, index):
    all_paths, by_name, _ = index
    refs = []
    issues = []
    documented_locations = {
        'main.c': 'firmware/main/main.c',
        'machine_app_runner.c': 'host/machine_app_runner.c',
        'machine.c': 'pstop_c/pstop/src/pstop/machine.c',
    }
    citations = re.findall(
        r'(?<![\w])([A-Za-z0-9_.-]+(?:/[A-Za-z0-9_.-]+)*\.(?:cpp|hpp|yaml|yml|cc|py|sh|c|h))(?::\d+(?:-\d+)?)?(?![A-Za-z0-9_.])',
        cell,
    )
    for citation in dict.fromkeys(citations):
        if '/' in citation and citation in all_paths:
            refs.append(citation)
        elif '/' in citation:
            refs.append(citation)
            issues.append(ResolutionIssue('missing', sr_id, citation, 'cited code file does not exist', line, 'code'))
        else:
            documented = documented_locations.get(citation)
            if documented in all_paths:
                refs.append(documented)
                continue
            candidates = by_name.get(citation, [])
            if len(candidates) == 1:
                refs.append(candidates[0])
            elif len(candidates) > 1:
                issues.append(
                    ResolutionIssue(
                        'ambiguous', sr_id, citation, 'unresolved ambiguous bare code filename', line, 'code'
                    )
                )
            else:
                issues.append(
                    ResolutionIssue('unresolved', sr_id, citation, 'unresolved bare code filename', line, 'code')
                )
    return tuple(dict.fromkeys(refs)), tuple(issues)


def parse_traceability(root):
    root = Path(root)
    path = root / 'docs/safety/TRACEABILITY.md'
    if not path.is_file():
        raise LintError(f'required document missing: {path}')
    lines = path.read_text(encoding='utf-8').splitlines()
    index_data = _repo_index(root)
    rows = []
    reverse = {}
    issues = []
    index = 0
    while index < len(lines):
        if not lines[index].lstrip().startswith('|'):
            index += 1
            continue
        header = split_row(lines[index])
        is_trace = header and header[0] == 'SR' and any(cell.startswith('Verifying test') for cell in header)
        is_reverse = header and header[0] == 'F-xx' and 'SRs touching it' in header
        if not is_trace and not is_reverse:
            index += 1
            continue
        columns = {name: position for position, name in enumerate(header)}
        index += 2
        while index < len(lines) and lines[index].lstrip().startswith('|'):
            line_number = index + 1
            cells = split_row(lines[index])
            if len(cells) != len(header):
                raise LintError(f'{path}:{line_number}: malformed table row')
            if is_trace:
                sr_id = cells[0].strip('*')
                if not SR_RE.fullmatch(sr_id):
                    raise LintError(f'{path}:{line_number}: invalid trace SR ID {sr_id!r}')
                test_cell = cells[next(pos for name, pos in columns.items() if name.startswith('Verifying test'))]
                test_refs, test_issues = _test_refs(root, sr_id, test_cell, line_number, index_data)
                code_refs, code_issues = _code_refs(sr_id, cells[2], line_number, index_data)
                status, prose = normalize_status(cells[-1], TRACE_STATUSES, path, line_number)
                rows.append(
                    TraceRow(
                        sr_id,
                        expand_allocations(cells[1]),
                        code_refs,
                        test_refs,
                        tuple(part.strip() for part in cells[-2].split('+') if part.strip()),
                        status,
                        prose,
                        'NO TEST' in test_cell,
                        line_number,
                        test_cell,
                    )
                )
                issues.extend(test_issues + code_issues)
            else:
                function_id = cells[0].strip('*')
                sr_cell = cells[2]
                reverse[function_id] = ReverseEntry(
                    function_id,
                    cells[1],
                    _expand_sr_list(sr_cell),
                    'non-safety' in sr_cell.lower(),
                    line_number,
                )
            index += 1
    if not rows:
        raise LintError(f'{path}: no traceability rows found')
    return tuple(rows), reverse, tuple(issues)
