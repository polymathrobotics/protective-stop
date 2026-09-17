# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Parse canonical safety requirements markdown without rewriting it."""

import re
from pathlib import Path

from .checks import SRS_STATUSES
from .model import LintError, SafetyRequirement

SR_RE = re.compile(r'^SR-(SYS|R|H|M|I)-(\d{2})$')


def split_row(line):
    """Split unescaped markdown pipes while preserving pipes in code spans."""
    cells = []
    current = []
    in_code = False
    escaped = False
    for char in line.strip():
        if escaped:
            current.append(char)
            escaped = False
        elif char == '\\':
            current.append(char)
            escaped = True
        elif char == '`':
            in_code = not in_code
            current.append(char)
        elif char == '|' and not in_code:
            cells.append(''.join(current).strip())
            current = []
        else:
            current.append(char)
    cells.append(''.join(current).strip())
    if cells and not cells[0]:
        cells.pop(0)
    if cells and not cells[-1]:
        cells.pop()
    return cells


def _strip_bold(value):
    value = value.strip()
    if value.startswith('**') and value.endswith('**') and len(value) >= 4:
        return value[2:-2].strip()
    return value


def normalize_status(cell, vocabulary, path, line):
    """Return a canonical leading status token and untouched trailing prose."""
    value = cell.strip()
    if value.startswith('**'):
        close = value.find('**', 2)
        if close != -1:
            value = value[2:close] + value[close + 2 :]
    value = value.strip()
    for token in vocabulary:
        if re.match(re.escape(token) + r'(?:\b|\s|$|[—(\[])', value, re.IGNORECASE):
            return token, value[len(token) :].strip()
    raise LintError(f'{path}:{line}: unknown status cell {cell!r}')


def expand_allocations(cell, path='<allocation>', line=1):
    """Expand compact ranges and arbitrary slash chains, rejecting truncation."""
    result = []
    pattern = re.compile(r'F-([A-Z])-([0-9]{2})(?:\.\.([0-9]{2})|((?:/[0-9]{2})+))?')
    for match in pattern.finditer(cell):
        area, first, end, alternates = match.groups()
        trailing = cell[match.end() :]
        malformed = (
            (trailing.startswith('/') and not trailing.startswith('/F-'))
            or (trailing.startswith('/F-') and re.match(r'^/F-[A-Z]-[0-9]{2}(?=$|[\s,.;)])', trailing) is None)
            or trailing.startswith('..')
            or re.match(r'^[.,-][0-9]', trailing) is not None
            or (trailing and trailing[0].isalnum())
        )
        if malformed:
            literal = re.match(r'[^\s,|)]+', cell[match.start() :]).group()
            raise LintError(f'{path}:{line}: malformed allocation {literal!r}')
        if end:
            if int(end) < int(first):
                raise LintError(f'{path}:{line}: descending allocation range F-{area}-{first}..{end}')
            result.extend(f'F-{area}-{number:02d}' for number in range(int(first), int(end) + 1))
        else:
            result.append(f'F-{area}-{first}')
            result.extend(f'F-{area}-{number}' for number in (alternates or '').lstrip('/').split('/') if number)
    return tuple(dict.fromkeys(result))


def parse_srs(path):
    """Parse all requirement tables discovered by their semantic headers."""
    path = Path(path)
    if not path.is_file():
        raise LintError(f'required document missing: {path}')
    lines = path.read_text(encoding='utf-8').splitlines()
    rows = []
    index = 0
    while index < len(lines):
        if not lines[index].lstrip().startswith('|'):
            index += 1
            continue
        header = split_row(lines[index])
        if not header or header[0] != 'ID' or 'Derived from' not in header:
            index += 1
            continue
        columns = {name: position for position, name in enumerate(header)}
        index += 2
        while index < len(lines) and lines[index].lstrip().startswith('|'):
            line_number = index + 1
            cells = split_row(lines[index])
            if len(cells) != len(header):
                raise LintError(f'{path}:{line_number}: expected {len(header)} cells, got {len(cells)}')
            sr_id = _strip_bold(cells[columns['ID']])
            match = SR_RE.fullmatch(sr_id)
            if not match:
                raise LintError(f'{path}:{line_number}: invalid requirement ID {sr_id!r}')
            status, prose = normalize_status(cells[columns['Status']], SRS_STATUSES, path, line_number)
            derived = tuple(
                dict.fromkeys(re.findall(r'SG-\d+|H-\d{2}|DU-\d+|\b[A-Z]\d{2}-\d\b', cells[columns['Derived from']]))
            )
            verify = tuple(part.strip() for part in re.split(r'\s*\+\s*', cells[columns['Verify']]) if part.strip())
            rows.append(
                SafetyRequirement(
                    sr_id,
                    match.group(1),
                    int(match.group(2)),
                    cells[columns['Requirement (shall)']],
                    derived,
                    expand_allocations(cells[columns['Allocated to']], path, line_number),
                    cells[columns['Integrity']],
                    verify,
                    status,
                    prose,
                    line_number,
                )
            )
            index += 1
    if not rows:
        raise LintError(f'{path}: no requirements table found')
    return tuple(rows)


def convention_statuses(path):
    """Extract only statuses explicitly listed in conventions section 1."""
    text = Path(path).read_text(encoding='utf-8')
    section = text.split('## 1.', 1)[1].split('\n## 2.', 1)[0] if '## 1.' in text else text
    return frozenset(status for status in SRS_STATUSES if re.search(rf'\*\*{re.escape(status)}\*\*', section))
