# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Command-line entry point for the safety traceability linter."""

import argparse
import json
import sys
from pathlib import Path

from .checks import apply_baseline, check_numeric_coverage_claims, check_summary_prose, run_checks
from .coverage import compute_coverage
from .model import LintError
from .render import render_traceability
from .runner import analyze


def _load_baseline(path):
    if not path.is_file():
        raise LintError(f'required document missing: {path}')
    try:
        document = json.loads(path.read_text(encoding='utf-8'))
        entries = document['findings']
        baseline = {}
        for entry in entries:
            if not entry.get('owner') or not entry.get('reason') or not entry.get('finding'):
                raise LintError(f'baseline entry needs owner, reason, and exact finding: {entry}')
            key = (entry['check_id'], entry['subject'], entry['finding'])
            if key in baseline:
                raise LintError(f'duplicate exact baseline entry: {key}')
            baseline[key] = entry
        return baseline
    except (KeyError, TypeError, json.JSONDecodeError) as error:
        raise LintError(f'invalid baseline {path}: {error}') from error


def _coverage_dict(coverage):
    return {
        'total': coverage.total,
        'cited_tests': coverage.cited_tests,
        'verified': coverage.verified,
        'areas': coverage.areas,
        'functions': {
            'traced': coverage.functions_traced,
            'total': coverage.functions_total,
            'safety_total': coverage.safety_functions_total,
        },
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--write', action='store_true')
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--root', default='.', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.check and args.write:
        parser.error('--check and --write are mutually exclusive')
    try:
        root = Path(args.root).resolve()
        analysis = analyze(root)
        baseline = _load_baseline(root / 'docs/safety/lint-baseline.json')
        coverage = compute_coverage(analysis)
        trace_path = root / 'docs/safety/TRACEABILITY.md'
        original = trace_path.read_text(encoding='utf-8')
        active, suppressed = apply_baseline(
            run_checks(analysis) + check_numeric_coverage_claims(original) + check_summary_prose(original, coverage),
            baseline,
        )
        rendered = render_traceability(original, coverage)
        stale = rendered != original
        active_errors = [finding for finding in active if finding.severity == 'error']
        if args.write and not active_errors:
            trace_path.write_text(rendered, encoding='utf-8')
            stale = False
        if args.json:
            print(
                json.dumps(
                    {
                        'findings': [finding.__dict__ for finding in active],
                        'baselined': [finding.__dict__ for finding in suppressed],
                        'coverage': _coverage_dict(coverage),
                        'stale': stale,
                    },
                    indent=2,
                    sort_keys=True,
                )
            )
        else:
            for severity in ('error', 'warning', 'info'):
                for finding in active:
                    if finding.severity == severity:
                        print(
                            f'{finding.file}:{finding.line}: [{finding.check_id}] {finding.subject} — {finding.message}'
                        )
            print('Baselined findings:')
            for finding in suppressed:
                print(f'{finding.file}:{finding.line}: [{finding.check_id}] {finding.subject} — {finding.message}')
            print(
                f'Coverage: cited tests {coverage.cited_tests}/{coverage.total}; strict Verified {coverage.verified}/{coverage.total}; functions {coverage.functions_traced}/{coverage.functions_total} ({coverage.functions_traced}/{coverage.safety_functions_total} excluding declared non-safety)'
            )
            print('Citation limitation: resolution does not verify test execution or passing state.')
            for area, data in coverage.areas.items():
                print(f'  SR-{area}: {data["count"]} total, {data["cited"]} cited, {data["Verified"]} Verified')
            if args.check and stale:
                print(
                    'docs/safety/TRACEABILITY.md: generated regions are stale; run python3 -m tools.safety_lint --write'
                )
        failed = bool(active_errors) or (args.check and stale)
        return 1 if failed else 0
    except (LintError, OSError) as error:
        print(f'safety-lint: cannot run: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
