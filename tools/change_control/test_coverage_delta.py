# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Spec-driven tests for deterministic safety-citation coverage delta reporting."""

import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from tools.change_control.coverage_delta import (  # noqa: E402
    compare_reports,
    run_linter_at_tree,
    upsert_coverage_comment,
)


class CoverageDeltaTests(unittest.TestCase):
    def test_coverage_delta_detects_lost_citation(self):
        """Deleting real cited evidence in a scratch repository must name the affected requirement."""
        with tempfile.TemporaryDirectory() as directory:
            scratch = Path(directory) / 'repository'
            shutil.copytree(ROOT / 'tools/safety_lint/fixtures/repository', scratch)
            (scratch / 'tools').mkdir(exist_ok=True)
            shutil.copytree(
                ROOT / 'tools/safety_lint',
                scratch / 'tools/safety_lint',
                ignore=shutil.ignore_patterns('__pycache__'),
            )
            base = run_linter_at_tree(scratch)
            (scratch / 'tests/test_unique_probe.py').unlink()
            head = run_linter_at_tree(scratch)
            delta = compare_reports(base, head)
        self.assertIn('SR-R-01', delta)
        self.assertIn('unresolvable', delta.lower())

    def test_revision_linter_subprocesses_cannot_observe_actions_credentials(self):
        """Code from an untrusted revision must receive no Actions token while producing a normal report."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / 'tools/safety_lint'
            package.mkdir(parents=True)
            (root / 'tools/__init__.py').write_text('', encoding='utf-8')
            (package / '__init__.py').write_text('', encoding='utf-8')
            guard = (
                "if os.environ.get('GH_TOKEN') or os.environ.get('GITHUB_TOKEN'): raise RuntimeError('token leaked')"
            )
            (package / '__main__.py').write_text(
                'import json, os\n'
                f'{guard}\n'
                "print(json.dumps({'coverage': {'cited_tests': 1, 'total': 1}, 'findings': []}))\n",
                encoding='utf-8',
            )
            (package / 'runner.py').write_text(
                'import os\nfrom types import SimpleNamespace\n'
                f'{guard}\n'
                "def analyze(root): return SimpleNamespace(trace=[SimpleNamespace(sr_id='SR-X-01', test_refs=['probe'])])\n",
                encoding='utf-8',
            )
            with mock.patch.dict(os.environ, {'GH_TOKEN': 'gh-sentinel', 'GITHUB_TOKEN': 'github-sentinel'}):
                report = run_linter_at_tree(root)
                self.assertEqual(os.environ['GH_TOKEN'], 'gh-sentinel')
                self.assertEqual(os.environ['GITHUB_TOKEN'], 'github-sentinel')
        self.assertEqual(report['coverage'], {'cited_tests': 1, 'total': 1})
        self.assertEqual(report['citations'], {'SR-X-01': ['probe']})

    def test_base_without_linter_exposes_stacked_dependency(self):
        """A base predating change-0001 must be reported as unavailable, never treated as zero coverage."""
        delta = compare_reports(
            {'unavailable': 'tools/safety_lint absent'},
            {'coverage': {'total': 1, 'cited_tests': 1}, 'findings': [], 'citations': {}},
        )
        self.assertIn('stacked dependency', delta.lower())

    def test_head_without_linter_is_an_execution_error(self):
        """Missing head coverage must fail explicitly rather than render unknown values as a delta."""
        with self.assertRaisesRegex(RuntimeError, 'head coverage unavailable'):
            compare_reports(
                {'coverage': {'total': 1, 'cited_tests': 1}, 'findings': [], 'citations': {}},
                {'unavailable': 'tools/safety_lint absent'},
            )

    def test_coverage_comment_updates_instead_of_appending(self):
        """Coverage delta must update its marker-owned comment rather than append on every run."""
        writes = []

        def api(method, path, body=None, paginate=False):
            if method == 'GET':
                self.assertTrue(paginate)
                return [{'id': 12, 'body': '<!-- coverage-delta -->\nold'}]
            writes.append((method, path, body))
            return {}

        upsert_coverage_comment(api, 'acme/project', 7, 'new')
        self.assertEqual(writes[0][0:2], ('PATCH', 'repos/acme/project/issues/comments/12'))

    def test_coverage_comment_marker_on_second_page_is_updated(self):
        """Comment lookup must paginate so a marker beyond page one is updated rather than duplicated."""
        calls = []

        def api(method, path, body=None, paginate=False):
            calls.append((method, path, body, paginate))
            if method == 'GET':
                self.assertTrue(paginate)
                return [{'id': 1, 'body': 'first page'}, {'id': 12, 'body': '<!-- coverage-delta -->\nold'}]
            return {}

        upsert_coverage_comment(api, 'acme/project', 7, 'new')
        self.assertIn(
            ('PATCH', 'repos/acme/project/issues/comments/12', {'body': '<!-- coverage-delta -->\nnew'}, False), calls
        )
        self.assertFalse(any(call[0] == 'POST' for call in calls))

    def test_coverage_cli_no_comment_avoids_write_api(self):
        """Fork-safe coverage reporting must support stdout-only operation when write tokens are unavailable."""
        result = subprocess.run(
            [sys.executable, '-m', 'tools.change_control.coverage_delta', '--help'],
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn('--no-comment', result.stdout)


class WorkflowTests(unittest.TestCase):
    def test_workflows_disable_writes_for_fork_pull_requests(self):
        """Fork pull requests must still report coverage without attempting unavailable comment writes."""
        coverage = (ROOT / '.github/workflows/coverage-delta.yml').read_text(encoding='utf-8')
        self.assertIn('CAN_COMMENT', coverage)
        self.assertIn('--no-comment', coverage)

    def test_marker_comment_workflows_cancel_superseded_pr_runs(self):
        """The marker-comment writer must cancel stale runs in its per-PR concurrency group."""
        coverage = (ROOT / '.github/workflows/coverage-delta.yml').read_text(encoding='utf-8')
        self.assertIn('group: coverage-delta-${{ github.event.pull_request.number }}', coverage)
        self.assertEqual(coverage.count('cancel-in-progress: true'), 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
