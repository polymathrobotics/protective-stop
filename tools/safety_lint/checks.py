# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Bidirectional consistency checks and baseline ratchet."""

import re
from collections import Counter, defaultdict
from pathlib import Path

from .model import Finding

SRS_STATUSES = ('Partially satisfied', 'Residual-accepted', 'Satisfied', 'Gap')
TRACE_STATUSES = ('Partially-verified', 'Residual-accepted', 'Unverified-gap', 'Verified')

_GENERATED_REGION = re.compile(
    r'<!-- BEGIN GENERATED: safety-lint (?P<name>[\w-]+) -->.*?'
    r'<!-- END GENERATED: safety-lint (?P=name) -->',
    re.DOTALL,
)
_COVERAGE_CLAIM = re.compile(
    r'\d+\s*/\s*\d+\s*=\s*~?\d+(?:\.\d+)?\s*%'
    r'|\d+\s*/\s*\d+'
    r'|~?\d+(?:\.\d+)?\s*%'
    r'|\d+\s+(?:Partially-verified|Residual-with-test|Residual-accepted|'
    r'Unverified-gap(?:\s+SRs?)?|Verified|Partials?)\b',
    re.IGNORECASE,
)
_COMPACT_SAFETY_ID = re.compile(r'\b(?:SR-[A-Z]+-\d+(?:/\d+)*|DU-\d+(?:/\d+)*)\b')


def _finding(check, severity, subject, message, file, line=1):
    return Finding(check, severity, subject, message, file, line)


def evidence_rejection(root, path, sr_id):
    """Return why an existing path is not one of the three approved evidence classes."""
    candidate = Path(path)
    parts = tuple(part.lower() for part in candidate.parts)
    stem = candidate.stem.lower()

    # Markdown is always a report first; test-like paths and names cannot bypass report rules.
    if candidate.suffix.lower() == '.md':
        if candidate.name.lower() == 'readme.md':
            return 'README files are not approved evidence reports'
        if not parts or parts[0] != 'docs':
            return 'Markdown evidence reports must be under docs/'
        content = (Path(root) / candidate).read_text(encoding='utf-8', errors='replace')
        token = re.compile(rf'(?<![A-Za-z0-9_-]){re.escape(sr_id)}(?:[a-z])?(?![A-Za-z0-9_-])')
        if token.search(content) is None:
            return 'evidence report does not name cited SR as a complete token'
        return None

    # 1. Test sources are identified by a test/requirements directory or source naming.
    if {'test', 'tests', 'requirements'} & set(parts) or stem.startswith('test_') or stem.endswith('_test'):
        return None

    # 2. Repository guard scripts are scripts/check_*.sh only.
    if len(parts) == 2 and parts[0] == 'scripts' and candidate.name.startswith('check_') and candidate.suffix == '.sh':
        return None

    return 'path is not an approved evidence class (test artifact, SR-naming docs report, or scripts/check_*.sh guard)'


def run_checks(analysis):
    findings = []
    srs_counts = Counter(row.sr_id for row in analysis.srs)
    trace_counts = Counter(row.sr_id for row in analysis.trace)
    for sr_id in sorted(set(srs_counts) | set(trace_counts)):
        if srs_counts[sr_id] != 1 or trace_counts[sr_id] != 1:
            findings.append(
                _finding(
                    'C1',
                    'error',
                    sr_id,
                    f'SRS count {srs_counts[sr_id]}, matrix count {trace_counts[sr_id]}',
                    'docs/safety/TRACEABILITY.md',
                )
            )

    used_statuses = {row.status for row in analysis.srs}
    for status in sorted(used_statuses - analysis.conventions_statuses):
        findings.append(
            _finding(
                'C2',
                'info',
                status,
                'valid status is in actual use but absent from SRS conventions section 1',
                'docs/safety/SAFETY_REQUIREMENTS.md',
            )
        )
    for row in analysis.srs:
        if row.status not in SRS_STATUSES:
            findings.append(
                _finding(
                    'C2',
                    'error',
                    row.sr_id,
                    f'invalid SRS status {row.status}',
                    'docs/safety/SAFETY_REQUIREMENTS.md',
                    row.source_line,
                )
            )
    for row in analysis.trace:
        if row.status not in TRACE_STATUSES:
            findings.append(
                _finding(
                    'C2',
                    'error',
                    row.sr_id,
                    f'invalid trace status {row.status}',
                    'docs/safety/TRACEABILITY.md',
                    row.source_line,
                )
            )

    for row in analysis.trace:
        if row.status in ('Verified', 'Partially-verified') and not row.test_refs:
            findings.append(
                _finding(
                    'C3',
                    'error',
                    row.sr_id,
                    f'{row.status} has no resolvable test citation',
                    'docs/safety/TRACEABILITY.md',
                    row.source_line,
                )
            )
        if row.status == 'Unverified-gap' and (not row.has_no_test_marker or row.test_refs):
            findings.append(
                _finding(
                    'C3',
                    'error',
                    row.sr_id,
                    'Unverified-gap must carry NO TEST and no resolvable test citation',
                    'docs/safety/TRACEABILITY.md',
                    row.source_line,
                )
            )

    for issue in analysis.issues:
        if issue.category == 'test':
            findings.append(
                _finding(
                    'C4',
                    'error',
                    issue.sr_id,
                    f'{issue.literal}: {issue.message}',
                    'docs/safety/TRACEABILITY.md',
                    issue.source_line,
                )
            )
        else:
            severity = 'warning' if issue.kind in ('unresolved', 'ambiguous') else 'error'
            findings.append(
                _finding(
                    'C5',
                    severity,
                    issue.sr_id,
                    f'{issue.literal}: {issue.message}',
                    'docs/safety/TRACEABILITY.md',
                    issue.source_line,
                )
            )

    for row in analysis.srs:
        for function_id in row.allocated_to:
            if function_id not in analysis.functions:
                findings.append(
                    _finding(
                        'C6',
                        'error',
                        f'SRS:{function_id}',
                        f'allocated by {row.sr_id} but absent from system definition',
                        'docs/safety/SAFETY_REQUIREMENTS.md',
                        row.source_line,
                    )
                )
    for row in analysis.trace:
        for function_id in row.allocated_to:
            if function_id not in analysis.functions:
                findings.append(
                    _finding(
                        'C6',
                        'error',
                        f'TRACE:{function_id}',
                        f'allocated by {row.sr_id} but absent from system definition',
                        'docs/safety/TRACEABILITY.md',
                        row.source_line,
                    )
                )

    for row in analysis.srs:
        for reference in row.derived_from:
            if not reference.startswith(('SG-', 'H-', 'DU-')):
                continue
            known = reference in (analysis.fmea_ids if reference.startswith('DU-') else analysis.hara_ids)
            if not known:
                findings.append(
                    _finding(
                        'C7',
                        'warning',
                        reference,
                        f'upstream reference from {row.sr_id} not found',
                        'docs/safety/SAFETY_REQUIREMENTS.md',
                        row.source_line,
                    )
                )

    allocated = defaultdict(set)
    for row in analysis.srs:
        for function_id in row.allocated_to:
            allocated[function_id].add(row.sr_id)
    for function_id in sorted(set(analysis.functions) | set(analysis.reverse)):
        entry = analysis.reverse.get(function_id)
        if entry is None:
            findings.append(
                _finding(
                    'C8', 'error', function_id, 'system function absent from reverse map', 'docs/safety/TRACEABILITY.md'
                )
            )
            continue
        expected = allocated.get(function_id, set())
        actual = set(entry.sr_ids)
        if expected != actual:
            findings.append(
                _finding(
                    'C8',
                    'error',
                    function_id,
                    f'reverse map {sorted(actual)} != allocated SRs {sorted(expected)}',
                    'docs/safety/TRACEABILITY.md',
                    entry.source_line,
                )
            )
        if not expected:
            findings.append(
                _finding(
                    'C8',
                    'warning',
                    function_id,
                    'function has no allocated safety requirement',
                    'docs/safety/TRACEABILITY.md',
                    entry.source_line,
                )
            )

    srs_by_id = {row.sr_id: row for row in analysis.srs}
    for row in analysis.trace:
        if row.sr_id in srs_by_id and set(row.allocated_to) != set(srs_by_id[row.sr_id].allocated_to):
            findings.append(
                _finding(
                    'C9',
                    'error',
                    row.sr_id,
                    f'matrix allocation {sorted(row.allocated_to)} != SRS allocation {sorted(srs_by_id[row.sr_id].allocated_to)}',
                    'docs/safety/TRACEABILITY.md',
                    row.source_line,
                )
            )
    return tuple(findings)


def check_numeric_coverage_claims(text):
    """Reject numeric section-3 coverage claims outside generated regions."""
    heading = re.search(r'^## 3\. Requirements coverage summary\s*$', text, re.MULTILINE)
    if heading is None:
        return ()
    remainder = text[heading.end() :]
    boundaries = [
        match.start()
        for pattern in (r'^\*\*Reading:\*\*', r'^## (?!3\.)')
        if (match := re.search(pattern, remainder, re.MULTILINE)) is not None
    ]
    end = heading.end() + min(boundaries) if boundaries else len(text)
    section = text[heading.start() : end]
    section = _GENERATED_REGION.sub('', section)
    section = _COMPACT_SAFETY_ID.sub(lambda match: ' ' * len(match.group()), section)

    claims = []
    for match in _COVERAGE_CLAIM.finditer(section):
        claim = re.sub(r'\s*/\s*', ' / ', match.group())
        claim = re.sub(r'\s*=\s*', ' = ', claim)
        claim = re.sub(r'\s*%', ' %', claim)
        claims.append(' '.join(claim.split()))
    if not claims:
        return ()
    return (
        _finding(
            'C10',
            'error',
            'section 3 outside generated regions',
            f'numeric coverage claims outside generated regions: [{", ".join(claims)}]',
            'docs/safety/TRACEABILITY.md',
            text.count('\n', 0, heading.start()) + 1,
        ),
    )


def apply_baseline(findings, baseline):
    active = []
    suppressed = []
    matched = set()
    for finding in findings:
        key = (finding.check_id, finding.subject, finding.message)
        if key in baseline and key not in matched and finding.severity != 'info':
            suppressed.append(
                Finding(finding.check_id, 'info', finding.subject, finding.message, finding.file, finding.line)
            )
            matched.add(key)
        else:
            active.append(finding)
    for key in sorted(set(baseline) - matched):
        active.append(
            _finding(
                'BASELINE',
                'error',
                f'{key[0]}:{key[1]}',
                f'stale baseline entry has no exact current finding: {key[2]}',
                'docs/safety/lint-baseline.json',
            )
        )
    return tuple(active), tuple(suppressed)
