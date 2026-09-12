# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Run change-control checks against pull-request state obtained through gh api."""

import argparse
import json
import shlex
import subprocess
import sys
from pathlib import Path

from tools.safety_lint.model import LintError

from .checks import evaluate, load_mode, render_report, upsert_comment


class GhApi:
    """Small fail-closed subprocess adapter around gh api."""

    def __init__(self, command, repository):
        self.command = shlex.split(command)
        self.repository = repository

    def __call__(self, method, path, body=None, paginate=False, collection_key=None):
        endpoint = path.replace('{repo}', self.repository)
        command = [*self.command, 'api', endpoint]
        if paginate:
            command.extend(['--paginate', '--slurp'])
        if method != 'GET':
            command.extend(['--method', method])
        if body:
            for key, value in body.items():
                command.extend(['--field', f'{key}={value}'])
        result = subprocess.run(command, check=False, capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or f'gh api failed for {endpoint}')
        try:
            response = json.loads(result.stdout or '{}')
        except json.JSONDecodeError as error:
            raise RuntimeError(f'gh api returned invalid JSON for {endpoint}') from error
        if not paginate:
            return response
        if not isinstance(response, list):
            raise RuntimeError(f'paginated gh api response is not a page list for {endpoint}')
        if collection_key:
            merged = []
            for page in response:
                if not isinstance(page, dict) or not isinstance(page.get(collection_key), list):
                    raise RuntimeError(f'paginated gh api response lacks {collection_key} for {endpoint}')
                merged.extend(page[collection_key])
            return {collection_key: merged}
        if not all(isinstance(page, list) for page in response):
            raise RuntimeError(f'paginated gh api response contains a non-list page for {endpoint}')
        return [item for page in response for item in page]


def _require(mapping, path):
    value = mapping
    for key in path:
        if not isinstance(value, dict) or key not in value:
            raise RuntimeError(f'partial GitHub response missing {".".join(path)}')
        value = value[key]
    return value


def _cr_number(body):
    import re

    values = set(
        re.findall(
            r'(?im)^\s*(?:closes|refs)\s+(?:(?:https://github\.com/[^/]+/[^/]+/issues/)?#?)(\d+)\s*$',
            body or '',
        )
    )
    return int(next(iter(values))) if len(values) == 1 else None


def collect(api, repository, pr_number):
    """Collect the complete GitHub snapshot used by pure policy evaluation."""
    prefix = f'repos/{repository}'
    pr = api('GET', f'{prefix}/pulls/{pr_number}')
    _require(pr, ('user', 'login'))
    head = _require(pr, ('head', 'sha'))
    if 'body' not in pr or 'labels' not in pr:
        raise RuntimeError('partial GitHub response missing PR body or labels')
    cr = _cr_number(pr['body'])
    issue = api('GET', f'{prefix}/issues/{cr}') if cr else {'labels': [], 'body': ''}
    comments = api('GET', f'{prefix}/issues/{cr}/comments', paginate=True) if cr else []
    reviews = api('GET', f'{prefix}/pulls/{pr_number}/reviews', paginate=True)
    files = api('GET', f'{prefix}/pulls/{pr_number}/files', paginate=True)
    checks = api('GET', f'{prefix}/commits/{head}/check-runs', paginate=True, collection_key='check_runs')
    workflows = api(
        'GET',
        f'{prefix}/actions/runs?head_sha={head}',
        paginate=True,
        collection_key='workflow_runs',
    )
    pr_comments = api('GET', f'{prefix}/issues/{pr_number}/comments', paginate=True)
    for name, value in (('files', files), ('comments', comments), ('reviews', reviews), ('PR comments', pr_comments)):
        if not isinstance(value, list):
            raise RuntimeError(f'partial GitHub response: {name} is not a list')
    return {
        'pr': pr,
        'issue': issue,
        'issue_comments': comments,
        'reviews': reviews,
        'files': files,
        'check_runs': _require(checks, ('check_runs',)),
        'workflow_runs': _require(workflows, ('workflow_runs',)),
        'pr_comments': pr_comments,
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', default='.')
    parser.add_argument('--repository', required=True)
    parser.add_argument('--pr', required=True, type=int)
    parser.add_argument('--gh', default='gh')
    parser.add_argument('--no-comment', action='store_true')
    args = parser.parse_args(argv)
    try:
        mode = load_mode(args.root)
        api = GhApi(args.gh, args.repository)
        data = collect(api, args.repository, args.pr)
        results = evaluate(Path(args.root), data)
        report = render_report(mode, results)
        print(report)
        if not args.no_comment:
            upsert_comment(api, args.pr, report, data['pr_comments'])
        findings = any(item.status == 'fail' for item in results)
        return 1 if findings and mode == 'enforce' else 0
    except (OSError, RuntimeError, ValueError, KeyError, LintError) as error:
        print(f'change-control: cannot run: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
