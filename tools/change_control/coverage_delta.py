# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Compare deterministic safety-linter output between two Git revisions."""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from .__main__ import GhApi

MARKER = '<!-- coverage-delta -->'


def compare_reports(base, head):
    """Render coverage and citation changes without assigning safety significance."""
    if head.get('unavailable'):
        raise RuntimeError(f'head coverage unavailable: {head["unavailable"]}')
    if base.get('unavailable'):
        return (
            'Coverage before: unavailable. The base predates the stacked dependency on '
            f'change-0001 (`tools/safety_lint`): {base["unavailable"]}\n'
            f'Coverage after: {head.get("coverage", {}).get("cited_tests", "?")}/{head.get("coverage", {}).get("total", "?")} cited.'
        )
    before = base.get('coverage', {})
    after = head.get('coverage', {})
    lines = [
        f'Coverage before: {before.get("cited_tests", "?")}/{before.get("total", "?")} cited.',
        f'Coverage after: {after.get("cited_tests", "?")}/{after.get("total", "?")} cited.',
    ]
    base_citations = base.get('citations', {})
    head_citations = head.get('citations', {})
    for sr_id in sorted(set(base_citations) | set(head_citations)):
        old = set(base_citations.get(sr_id, []))
        new = set(head_citations.get(sr_id, []))
        if old - new:
            lines.append(f'- {sr_id} lost citation(s): {", ".join(sorted(old - new))}')
        if new - old:
            lines.append(f'- {sr_id} gained citation(s): {", ".join(sorted(new - old))}')
    old_unresolved = {
        (item.get('check_id'), item.get('subject'), item.get('message'))
        for item in base.get('findings', [])
        if item.get('check_id') in ('C3', 'C4')
    }
    new_unresolved = {
        (item.get('check_id'), item.get('subject'), item.get('message'))
        for item in head.get('findings', [])
        if item.get('check_id') in ('C3', 'C4')
    }
    for _, subject, message in sorted(new_unresolved - old_unresolved):
        lines.append(f'- Newly unresolvable citation for {subject}: {message}')
    if len(lines) == 2:
        lines.append('- No citation gains, losses, or newly unresolvable citations.')
    lines.append(
        'Limitation: this is deterministic citation resolution, not evidence that a cited test executed or passed.'
    )
    return '\n'.join(lines)


def run_linter_at_tree(worktree):
    """Run the revision's own unchanged linter and add its parsed citation map."""
    if not (worktree / 'tools/safety_lint/__main__.py').is_file():
        return {'unavailable': 'tools/safety_lint is absent at this revision'}
    child_environment = os.environ.copy()
    for name in ('GH_TOKEN', 'GITHUB_TOKEN'):
        child_environment.pop(name, None)
    result = subprocess.run(
        [sys.executable, '-m', 'tools.safety_lint', '--json'],
        cwd=worktree,
        env=child_environment,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode == 2:
        raise RuntimeError(result.stderr.strip() or 'safety linter could not run')
    try:
        report = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError('safety linter emitted invalid JSON') from error
    citation_code = (
        'import json; from tools.safety_lint.runner import analyze; '
        'print(json.dumps({r.sr_id: sorted(set(r.test_refs)) for r in analyze(".").trace}, sort_keys=True))'
    )
    citations = subprocess.run(
        [sys.executable, '-c', citation_code],
        cwd=worktree,
        env=child_environment,
        check=False,
        capture_output=True,
        text=True,
    )
    if citations.returncode:
        raise RuntimeError(citations.stderr.strip() or 'cannot extract linter citations')
    report['citations'] = json.loads(citations.stdout)
    return report


def report_at_revision(root, revision):
    """Run the unchanged checked-in linter at one detached revision."""
    temporary = Path(tempfile.mkdtemp(prefix='pstop-coverage-delta-'))
    try:
        result = subprocess.run(
            ['git', 'worktree', 'add', '--detach', str(temporary), revision],
            cwd=root,
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or f'cannot materialize revision {revision}')
        return run_linter_at_tree(temporary)
    finally:
        subprocess.run(
            ['git', 'worktree', 'remove', '--force', str(temporary)], cwd=root, check=False, capture_output=True
        )
        shutil.rmtree(temporary, ignore_errors=True)


def upsert_coverage_comment(api, repository, pr, report):
    """Create or update the single marker-owned deterministic coverage comment."""
    comments = api('GET', f'repos/{repository}/issues/{pr}/comments', paginate=True)
    existing = next((comment for comment in comments if MARKER in comment.get('body', '')), None)
    body = f'{MARKER}\n{report}'
    if existing:
        api('PATCH', f'repos/{repository}/issues/comments/{existing["id"]}', {'body': body})
    else:
        api('POST', f'repos/{repository}/issues/{pr}/comments', {'body': body})


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', default='.')
    parser.add_argument('--base', required=True)
    parser.add_argument('--head', default='HEAD')
    parser.add_argument('--repository', required=True)
    parser.add_argument('--pr', required=True, type=int)
    parser.add_argument('--gh', default='gh')
    parser.add_argument('--no-comment', action='store_true')
    args = parser.parse_args(argv)
    try:
        root = Path(args.root).resolve()
        report = compare_reports(report_at_revision(root, args.base), report_at_revision(root, args.head))
        print(report)
        if not args.no_comment:
            try:
                upsert_coverage_comment(GhApi(args.gh, args.repository), args.repository, args.pr, report)
            except RuntimeError as error:
                print(f'coverage-delta: comment publication warning: {error}', file=sys.stderr)
        return 0
    except (OSError, RuntimeError, ValueError) as error:
        print(f'coverage-delta: cannot run: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
