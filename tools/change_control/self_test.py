#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Spec-driven tests for repository change-control tooling."""

import json
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

from tools.change_control.__main__ import GhApi  # noqa: E402
from tools.change_control.checks import (  # noqa: E402
    AUTHORIZERS,
    evaluate,
    impact_analysis_complete,
    minimum_class,
    upsert_comment,
)
from tools.change_control.coverage_delta import (  # noqa: E402
    compare_reports,
    run_linter_at_tree,
    upsert_coverage_comment,
)
from tools.change_control.issue_form import parse_issue_form, validate_issue_form  # noqa: E402
from tools.change_control.wire_format import HEADER_NAMES, check_wire_format, compute_signature  # noqa: E402

FORM = ROOT / '.github/ISSUE_TEMPLATE/change-request.yml'
WIRE_EXPECTED = ROOT / 'tools/change_control/wire_format.sha256'


def snapshot(**overrides):
    """Return a complete synthetic GitHub state shaped like the real API snapshot."""
    ia = (ROOT / 'docs/process/templates/IMPACT_ANALYSIS.md').read_text(encoding='utf-8')
    filled = ia.replace('Modules changed:', 'Modules changed: tools/change_control').replace(
        '|  |  |', '| None | None |'
    )
    data = {
        'pr': {
            'user': {'login': 'contributor'},
            'body': 'Closes #17',
            'labels': [{'name': 'class-b'}],
            'head': {'sha': 'abc123'},
        },
        'files': [{'filename': 'tools/change_control/checks.py'}],
        'issue': {
            'labels': [{'name': 'change-request'}, {'name': 'status:authorized'}],
            'body': '### Reason for the change\nNeeded\n### Hazards that may be affected\nNone identified, because tooling only\n### Description of the proposed change\nTooling\n### Baseline affected\nmain\n### Proposed class\nB',
        },
        'issue_comments': [
            {'user': {'login': AUTHORIZERS[0]}, 'body': 'Authorized: proceed.'},
            {'user': {'login': 'analyst'}, 'body': filled},
        ],
        'reviews': [],
        'check_runs': [{'name': 'change-control', 'conclusion': 'success', 'head_sha': 'abc123'}],
        'workflow_runs': [],
    }
    data.update(overrides)
    return data


class IssueFormTests(unittest.TestCase):
    def test_issue_form_is_valid_yaml_and_parses(self):
        """The checked-in issue form must parse as the deliberately supported YAML subset."""
        parsed = parse_issue_form(FORM)
        self.assertEqual(parsed['name'], 'Change Request')
        self.assertGreater(len(parsed['body']), 5)

    def test_required_fields_present(self):
        """All five creation-time fields mandated by the procedure must be required."""
        fields = {field['id']: field for field in parse_issue_form(FORM)['body']}
        self.assertTrue(
            all(fields[name]['required'] for name in ('reason', 'hazards', 'description', 'baseline', 'class'))
        )

    def test_class_dropdown_defaults_to_c(self):
        """An unclassified request must conservatively default to Class C."""
        fields = {field['id']: field for field in parse_issue_form(FORM)['body']}
        self.assertEqual(fields['class']['options'][fields['class']['default']], 'C')

    def test_every_source_field_is_represented(self):
        """The issue form must carry every Change Request source section without a duplicate field list."""
        ids = {field['id'] for field in parse_issue_form(FORM)['body']}
        expected = {
            'reason',
            'hazards',
            'description',
            'baseline',
            'requester',
            'impact-analysis',
            'class',
            'authorization',
            'implementation',
            'gate-0',
            'gate-1',
            'review',
            'deviations',
            'release',
            'status',
        }
        self.assertEqual(ids, expected)

    def test_source_field_details_survive_issue_form_conversion(self):
        """YAML conversion must retain source details needed to complete implementation and Gate 1 records."""
        fields = {field['id']: field for field in parse_issue_form(FORM)['body']}
        self.assertIn('Yes / No, with link', fields['implementation']['description'])
        self.assertIn('Run by', fields['gate-1']['description'])
        self.assertIn('Forward -', fields['gate-1']['description'])
        self.assertIn('Backward -', fields['gate-1']['description'])

    def test_malformed_issue_form_is_rejected(self):
        """Malformed indentation must fail instead of silently degrading to a blank issue."""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'form.yml'
            path.write_text('name: Bad\nbody:\n - type: input\n    id: broken\n', encoding='utf-8')
            with self.assertRaises(ValueError):
                parse_issue_form(path)

    def test_blank_issue_fallback_risk_is_rejected(self):
        """A form lacking required top-level metadata or body controls must be invalid."""
        with self.assertRaises(ValueError):
            validate_issue_form({'name': 'Change Request', 'body': []})


class RepositoryPolicyTests(unittest.TestCase):
    def test_issue_chooser_keeps_blank_issues_and_links_security_policy(self):
        """Public reports must remain available while safety defects are directed to the private policy."""
        config = (ROOT / '.github/ISSUE_TEMPLATE/config.yml').read_text(encoding='utf-8')
        self.assertIn('blank_issues_enabled: true', config)
        self.assertIn('https://github.com/polymathrobotics/protective-stop/security/policy', config)

    def test_labels_json_has_no_duplicate_names(self):
        """The reproducible label definition must contain unique names."""
        labels = json.loads((ROOT / 'tools/change_control/labels.json').read_text(encoding='utf-8'))
        names = [label['name'] for label in labels]
        self.assertEqual(len(names), len(set(names)))

    def test_labels_json_contains_the_complete_settled_label_set(self):
        """The reproducible data must contain every label settled by the modification procedure plan."""
        labels = json.loads((ROOT / 'tools/change_control/labels.json').read_text(encoding='utf-8'))
        names = {label['name'] for label in labels}
        self.assertEqual(
            names,
            {
                'change-request',
                'class-a',
                'class-b',
                'class-c',
                'emergency',
                'safety-defect',
                'wire-break',
                'needs-change-request',
                'status:proposed',
                'status:under-analysis',
                'status:authorized',
                'status:rejected',
                'status:in-implementation',
                'status:in-verification',
                'status:merged',
                'status:released',
            },
        )

    def test_label_sync_dry_run_is_deterministic_and_network_free(self):
        """Two dry runs must produce identical plans without invoking GitHub."""
        environment = os.environ.copy()
        environment['PATH'] = '/usr/bin:/bin'
        first = subprocess.run(
            ['scripts/sync_labels.sh', '--dry-run'],
            cwd=ROOT,
            env=environment,
            check=False,
            capture_output=True,
            text=True,
        )
        second = subprocess.run(
            ['scripts/sync_labels.sh', '--dry-run'],
            cwd=ROOT,
            env=environment,
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual((first.returncode, second.returncode), (0, 0))
        self.assertEqual(first.stdout, second.stdout)
        self.assertEqual(first.stdout.count('would sync label:'), 16)

    def test_label_sync_targets_the_intended_repository_explicitly(self):
        """Label writes must not depend on ambiguous git-remote repository inference."""
        script = (ROOT / 'scripts/sync_labels.sh').read_text(encoding='utf-8')
        self.assertIn('--repo "$REPOSITORY"', script)

    def test_every_label_referenced_in_the_procedure_exists_in_labels_json(self):
        """Every backticked process label must exist in the reproducible label set."""
        import re

        procedure = (ROOT / 'docs/process/MODIFICATION_PROCEDURE.md').read_text(encoding='utf-8')
        referenced = set(
            re.findall(
                r'`((?:class-[abc]|change-request|emergency|safety-defect|wire-break|needs-change-request|status:[a-z-]+))`',
                procedure,
            )
        )
        labels = {
            item['name'] for item in json.loads((ROOT / 'tools/change_control/labels.json').read_text(encoding='utf-8'))
        }
        self.assertTrue(referenced)
        self.assertEqual(referenced - labels, set())

    def test_codeowners_parses_and_covers_root(self):
        """One CODEOWNERS rule must cover the repository root with all verified authorizers."""
        lines = [
            line.split()
            for line in (ROOT / '.github/CODEOWNERS').read_text(encoding='utf-8').splitlines()
            if line and not line.startswith('#')
        ]
        self.assertEqual(lines, [['*', *('@' + name for name in AUTHORIZERS)]])

    def test_procedure_authorizers_match_enforcement_and_codeowners(self):
        """The procedure, enforcement code, and CODEOWNERS must name one identical authorizer set."""
        import re

        procedure = (ROOT / 'docs/process/MODIFICATION_PROCEDURE.md').read_text(encoding='utf-8')
        named = set(re.findall(r'@(iliabaranov|rajasimman-madhivanan|davidt315)', procedure))
        self.assertEqual(named, set(AUTHORIZERS))

    @unittest.skipUnless(
        os.environ.get('GH_TOKEN'), 'GH_TOKEN absent; CODEOWNERS handle resolution test visibly skipped'
    )
    def test_codeowners_handles_resolve(self):
        """Every CODEOWNERS account must resolve through authenticated GitHub API access."""
        for handle in AUTHORIZERS:
            result = subprocess.run(['gh', 'api', f'users/{handle}'], check=False, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


class ImpactAndClassificationTests(unittest.TestCase):
    def test_e2_rejects_heading_present_but_empty(self):
        """An IA heading followed only by whitespace must fail completeness checking."""
        headings = ['# One', '# Two']
        complete, missing = impact_analysis_complete('# One\n \t\n# Two\nanswer\n', headings)
        self.assertFalse(complete)
        self.assertIn('# One', missing)

    def test_e2_accepts_na_content_without_judging_truth(self):
        """The checker verifies nonblank content but does not judge whether N/A is adequate."""
        self.assertEqual(impact_analysis_complete('# One\nN/A\n# Two\nN/A\n', ['# One', '# Two']), (True, []))

    def test_e3_rejects_invalid_sr_ids(self):
        """A cited token shaped like an SR but outside the canonical grammar must be a finding."""
        data = snapshot()
        data['issue_comments'][1]['body'] += '\nSR-BOGUS-99\n'
        results = evaluate(ROOT, data)
        self.assertEqual(next(item for item in results if item.check_id == 'E3').status, 'fail')

    def test_e4_under_classification_is_a_finding(self):
        """A class label below the path-derived floor must fail E4."""
        data = snapshot(files=[{'filename': 'docs/safety/HARA.md'}])
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E4').status, 'fail')

    def test_e4_over_classification_is_not_a_finding(self):
        """A class label above the path-derived floor must be accepted."""
        data = snapshot(files=[{'filename': 'README.md'}])
        data['pr']['labels'] = [{'name': 'class-c'}]
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E4').status, 'pass')

    def test_missing_class_label_is_a_finding(self):
        """A PR without a classification label must fail classification checking."""
        data = snapshot()
        data['pr']['labels'] = []
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E4').status, 'fail')

    def test_duplicate_class_labels_are_a_finding(self):
        """Multiple classification labels are ambiguous and must fail E4."""
        data = snapshot()
        data['pr']['labels'] = [{'name': 'class-a'}, {'name': 'class-b'}]
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E4').status, 'fail')

    def test_minimum_class_rules(self):
        """Every settled path rule must produce its documented minimum classification floor."""
        self.assertEqual(minimum_class(['pstop_c/x.c']), 'C')
        self.assertEqual(minimum_class(['firmware/main/main.c']), 'C')
        self.assertEqual(minimum_class(['components/x.c']), 'B')
        self.assertEqual(minimum_class(['firmware/sdkconfig.defaults']), 'B')
        self.assertEqual(minimum_class(['.github/workflows/x.yml']), 'B')


class ApprovalAndLinkTests(unittest.TestCase):
    def _class_c(self):
        data = snapshot(files=[{'filename': 'docs/safety/HARA.md'}])
        data['pr']['labels'] = [{'name': 'class-c'}]
        data['reviews'] = [
            {'user': {'login': AUTHORIZERS[0]}, 'state': 'APPROVED', 'commit_id': 'abc123'},
            {'user': {'login': AUTHORIZERS[1]}, 'state': 'APPROVED', 'commit_id': 'abc123'},
        ]
        return data

    def test_class_c_two_distinct_approvals_pass(self):
        """Class C requires two distinct current approving authorizers who are not the author."""
        self.assertEqual(next(item for item in evaluate(ROOT, self._class_c()) if item.check_id == 'E5').status, 'pass')

    def test_duplicate_reviews_count_once(self):
        """Repeated approvals by one account must count as one approval."""
        data = self._class_c()
        data['reviews'][1]['user']['login'] = AUTHORIZERS[0]
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E5').status, 'fail')

    def test_author_approval_is_excluded(self):
        """The PR author's own approval must never satisfy Class C approval."""
        data = self._class_c()
        data['pr']['user']['login'] = AUTHORIZERS[0]
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E5').status, 'fail')

    def test_stale_approval_is_excluded(self):
        """An approval for a commit other than the PR head must not count."""
        data = self._class_c()
        data['reviews'][1]['commit_id'] = 'oldsha'
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E5').status, 'fail')

    def test_approval_without_commit_identity_is_excluded(self):
        """An approval with no commit identity cannot establish review of the current diff."""
        data = self._class_c()
        data['reviews'][1].pop('commit_id')
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E5').status, 'fail')

    def test_authorization_after_pr_is_an_ordering_finding(self):
        """An authorization timestamp after implementation began must fail E1 ordering."""
        data = snapshot()
        data['pr']['created_at'] = '2026-09-10T10:00:00Z'
        data['issue_comments'][0]['created_at'] = '2026-09-10T11:00:00Z'
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E1').status, 'fail')

    def test_authorization_before_impact_analysis_is_an_ordering_finding(self):
        """Authorization must follow the completed Impact Analysis rather than merely precede implementation."""
        data = snapshot()
        data['pr']['created_at'] = '2026-09-10T12:00:00Z'
        data['issue_comments'][0]['created_at'] = '2026-09-10T10:00:00Z'
        data['issue_comments'][1]['created_at'] = '2026-09-10T11:00:00Z'
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E1').status, 'fail')

    def test_pr_author_cannot_authorize_own_change(self):
        """The implementer must not satisfy the Change Request authorization requirement."""
        data = snapshot()
        data['pr']['user']['login'] = AUTHORIZERS[0]
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E1').status, 'fail')

    def test_class_c_requires_two_distinct_cr_authorizers_before_implementation(self):
        """Class C implementation cannot start after only one Change Request authorization."""
        data = snapshot()
        data['pr']['labels'] = [{'name': 'class-c'}]
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E1').status, 'fail')
        data['issue_comments'].append({'user': {'login': AUTHORIZERS[1]}, 'body': 'Authorized: proceed.'})
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E1').status, 'pass')

    def test_rejection_text_does_not_count_as_authorization(self):
        """An authorizer saying a change is not authorized must not satisfy E1."""
        data = snapshot()
        data['issue_comments'][0]['body'] = 'Rejected: this is not authorized.'
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E1').status, 'fail')

    def test_emergency_requires_two_approvals_and_short_form_ia(self):
        """Emergency E7 must require two current approvals and all three short-form IA subjects."""
        data = snapshot()
        data['pr']['labels'].append({'name': 'emergency'})
        data['issue_comments'][1]['body'] = (
            'What changed: tooling\nWhat it could affect: process\nTests: change-control'
        )
        data['reviews'] = [
            {'user': {'login': AUTHORIZERS[0]}, 'state': 'APPROVED', 'commit_id': 'abc123'},
            {'user': {'login': AUTHORIZERS[1]}, 'state': 'APPROVED', 'commit_id': 'abc123'},
        ]
        data['check_runs'] = [{'name': 'change-control', 'conclusion': 'success', 'head_sha': 'abc123'}]
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E7').status, 'pass')

    def test_ambiguous_change_request_link_fails(self):
        """A bare issue number without Closes or Refs syntax must not be guessed as the CR."""
        data = snapshot()
        data['pr']['body'] = 'Issue #17'
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E1').status, 'fail')

    def test_multiple_change_request_links_fail(self):
        """Multiple candidate CR links must fail rather than selecting one."""
        data = snapshot()
        data['pr']['body'] = 'Closes #17\nRefs #18'
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E1').status, 'fail')

    def test_needs_change_request_label_exempts_e1(self):
        """External PRs awaiting a maintainer CR must produce a pending E1 result."""
        data = snapshot()
        data['pr']['labels'].append({'name': 'needs-change-request'})
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E1').status, 'pending')


class EvidenceAndCommentTests(unittest.TestCase):
    def test_e6_requires_at_least_one_named_test(self):
        """An IA that names no specific test must fail the plan-versus-execution check."""
        result = next(item for item in evaluate(ROOT, snapshot()) if item.check_id == 'E6')
        self.assertEqual(result.status, 'fail')

    def test_e6_missing_workflow_evidence_names_test(self):
        """A test named by the IA without head-SHA check evidence must fail and be named."""
        data = snapshot()
        data['issue_comments'][1]['body'] += '\n# 7. Verification plan for this change\n`missing-check`\n'
        result = next(item for item in evaluate(ROOT, data) if item.check_id == 'E6')
        self.assertEqual(result.status, 'fail')
        self.assertIn('missing-check', result.message)

    def test_e6_ignores_evidence_for_other_sha(self):
        """Evidence attached to an older commit must not satisfy the IA plan."""
        data = snapshot()
        data['issue_comments'][1]['body'] += '\n# 7. Verification plan for this change\n`host-check`\n'
        data['check_runs'] = [{'name': 'host-check', 'conclusion': 'success', 'head_sha': 'oldsha'}]
        self.assertEqual(next(item for item in evaluate(ROOT, data) if item.check_id == 'E6').status, 'fail')

    def test_e6_does_not_accept_a_substring_check_name(self):
        """A short IA test token must not match an unrelated longer check-run name."""
        data = snapshot()
        data['issue_comments'][1]['body'] += '\n# 7. Verification plan for this change\n`host`\n'
        data['check_runs'] = [{'name': 'host-check', 'conclusion': 'success', 'head_sha': 'abc123'}]
        result = next(item for item in evaluate(ROOT, data) if item.check_id == 'E6')
        self.assertEqual(result.status, 'fail')
        self.assertIn('host', result.message)

    def test_e6_reads_plain_table_cells(self):
        """Plain test names in the IA verification table must be checked, not only backticked names."""
        data = snapshot()
        data['issue_comments'][1]['body'] = data['issue_comments'][1]['body'].replace(
            '| Tests that validate the change itself |  |',
            '| Tests that validate the change itself | missing-table-check |',
        )
        result = next(item for item in evaluate(ROOT, data) if item.check_id == 'E6')
        self.assertEqual(result.status, 'fail')
        self.assertIn('missing-table-check', result.message)

    def test_upsert_comment_updates_existing_comment(self):
        """An existing bot report must be updated instead of appending another comment."""
        calls = []
        upsert_comment(
            lambda method, path, body=None: calls.append((method, path, body)),
            9,
            'report',
            [{'id': 44, 'body': '<!-- change-control -->old'}],
        )
        self.assertEqual(calls[0][0:2], ('PATCH', 'repos/{repo}/issues/comments/44'))

    def test_upsert_comment_creates_when_absent(self):
        """A bot report must be created exactly once when no marker exists."""
        calls = []
        upsert_comment(lambda method, path, body=None: calls.append((method, path, body)), 9, 'report', [])
        self.assertEqual(
            calls, [('POST', 'repos/{repo}/issues/9/comments', {'body': '<!-- change-control -->\nreport'})]
        )


class WireFormatTests(unittest.TestCase):
    def _copy_headers(self, directory):
        include = Path(directory) / 'pstop_c/pstop/include/pstop'
        include.mkdir(parents=True)
        source = ROOT / 'pstop_c/pstop/include/pstop'
        for name in HEADER_NAMES:
            shutil.copy2(source / name, include / name)
        return Path(directory)

    def test_signature_stable_across_comment_only_change(self):
        """Adding a C comment to a watched header must not alter its signature."""
        with tempfile.TemporaryDirectory() as directory:
            root = self._copy_headers(directory)
            before = compute_signature(root)
            path = root / 'pstop_c/pstop/include/pstop/protocol.h'
            path.write_text(path.read_text(encoding='utf-8') + '\n/* comment only */\n', encoding='utf-8')
            self.assertEqual(compute_signature(root).aggregate, before.aggregate)

    def test_comment_only_change_needs_no_wire_labels(self):
        """A comment-only watched-header edit must pass the declaration check without wire-break labels."""
        with tempfile.TemporaryDirectory() as directory:
            root = self._copy_headers(directory)
            shutil.copy2(WIRE_EXPECTED, root / 'wire_format.sha256')
            path = root / 'pstop_c/pstop/include/pstop/protocol.h'
            path.write_text(path.read_text(encoding='utf-8') + '\n/* comment only */\n', encoding='utf-8')
            code, _ = check_wire_format(root, root / 'wire_format.sha256', set(), [str(path)])
            self.assertEqual(code, 0)

    def test_signature_changes_on_field_addition(self):
        """Adding a structure field must alter the aggregate wire signature."""
        with tempfile.TemporaryDirectory() as directory:
            root = self._copy_headers(directory)
            before = compute_signature(root)
            path = root / 'pstop_c/pstop/include/pstop/pstop_msg.h'
            path.write_text(
                path.read_text(encoding='utf-8').replace('uint16_t checksum;', 'uint32_t added;\nuint16_t checksum;'),
                encoding='utf-8',
            )
            self.assertNotEqual(compute_signature(root).aggregate, before.aggregate)

    def test_signature_changes_on_constant_change(self):
        """Changing a protocol constant must alter the aggregate wire signature."""
        with tempfile.TemporaryDirectory() as directory:
            root = self._copy_headers(directory)
            before = compute_signature(root)
            path = root / 'pstop_c/pstop/include/pstop/pstop_msg.h'
            path.write_text(path.read_text(encoding='utf-8').replace('0xADU', '0xAEU'), encoding='utf-8')
            self.assertNotEqual(compute_signature(root).aggregate, before.aggregate)

    def test_signature_changes_on_message_size_change(self):
        """Changing PSTOP_MESSAGE_SIZE must alter the signature and reported literal."""
        with tempfile.TemporaryDirectory() as directory:
            root = self._copy_headers(directory)
            before = compute_signature(root)
            path = root / 'pstop_c/pstop/include/pstop/config.h'
            path.write_text(path.read_text(encoding='utf-8').replace('48U', '49U'), encoding='utf-8')
            after = compute_signature(root)
            self.assertEqual(after.message_size, '49U')
            self.assertNotEqual(after.aggregate, before.aggregate)

    def test_signature_changes_on_version_change(self):
        """Changing PSTOP_VERSION must alter both the reported version and aggregate signature."""
        with tempfile.TemporaryDirectory() as directory:
            root = self._copy_headers(directory)
            before = compute_signature(root)
            path = root / 'pstop_c/pstop/include/pstop/config.h'
            path.write_text(path.read_text(encoding='utf-8').replace('0x02U', '0x03U'), encoding='utf-8')
            after = compute_signature(root)
            self.assertEqual(after.version, '0x03U')
            self.assertNotEqual(after.aggregate, before.aggregate)

    def test_check_exits_one_on_mismatch_and_names_the_headers(self):
        """A stale expected signature must fail and name each changed header."""
        with tempfile.TemporaryDirectory() as directory:
            root = self._copy_headers(directory)
            shutil.copy2(WIRE_EXPECTED, root / 'wire_format.sha256')
            path = root / 'pstop_c/pstop/include/pstop/protocol.h'
            path.write_text(path.read_text(encoding='utf-8') + '\nint changed;\n', encoding='utf-8')
            code, message = check_wire_format(root, root / 'wire_format.sha256', {'wire-break', 'class-c'}, [])
            self.assertEqual(code, 1)
            self.assertIn('protocol.h', message)

    def test_expected_update_requires_both_labels(self):
        """Changing the expected signature cannot pass without wire-break and class-c labels."""
        code, message = check_wire_format(
            ROOT, WIRE_EXPECTED, {'wire-break'}, ['tools/change_control/wire_format.sha256']
        )
        self.assertEqual(code, 1)
        self.assertIn('class-c', message)

    def test_initial_expected_signature_is_not_a_wire_break(self):
        """Adding the first reviewed signature snapshot must not claim the existing wire format changed."""
        code, _ = check_wire_format(
            ROOT,
            WIRE_EXPECTED,
            set(),
            ['tools/change_control/wire_format.sha256'],
            expectation_preexisted=False,
        )
        self.assertEqual(code, 0)

    def test_initial_snapshot_cannot_hide_a_header_change(self):
        """A wire-header edit accompanying the first snapshot must still require Class C declaration."""
        code, message = check_wire_format(
            ROOT,
            WIRE_EXPECTED,
            {'wire-break'},
            ['pstop_c/pstop/include/pstop/protocol.h', 'tools/change_control/wire_format.sha256'],
            expectation_preexisted=False,
        )
        self.assertEqual(code, 1)
        self.assertIn('class-c', message)

    def test_wire_cli_exposes_guard_exit_codes(self):
        """The public wire CLI must return 0 for a match, 1 for policy mismatch, and 2 when it cannot run."""
        clean = subprocess.run(
            [sys.executable, '-m', 'tools.change_control.wire_format', 'check', '--root', str(ROOT)],
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
        finding = subprocess.run(
            [
                sys.executable,
                '-m',
                'tools.change_control.wire_format',
                'check',
                '--root',
                str(ROOT),
                '--changed-file',
                'tools/change_control/wire_format.sha256',
                '--labels',
                'wire-break',
            ],
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
        unable = subprocess.run(
            [sys.executable, '-m', 'tools.change_control.wire_format', 'check', '--root', '/does/not/exist'],
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual((clean.returncode, finding.returncode, unable.returncode), (0, 1, 2))

    def test_mismatch_explains_coordinated_rollout(self):
        """A wire mismatch must explicitly require coordinated remote and machine rollout."""
        with tempfile.TemporaryDirectory() as directory:
            root = self._copy_headers(directory)
            shutil.copy2(WIRE_EXPECTED, root / 'wire_format.sha256')
            path = root / 'pstop_c/pstop/include/pstop/constants.h'
            path.write_text(path.read_text(encoding='utf-8').replace('10U', '11U'), encoding='utf-8')
            _, message = check_wire_format(root, root / 'wire_format.sha256', set(), [])
            self.assertIn('remote and machine', message.lower())


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

        def api(method, path, body=None):
            if method == 'GET':
                return [{'id': 12, 'body': '<!-- coverage-delta -->\nold'}]
            writes.append((method, path, body))
            return {}

        upsert_coverage_comment(api, 'acme/project', 7, 'new')
        self.assertEqual(writes[0][0:2], ('PATCH', 'repos/acme/project/issues/comments/12'))

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


class CliTests(unittest.TestCase):
    def test_gh_api_flattens_all_paginated_list_pages(self):
        """Policy evaluation must see every item returned across GitHub list pages."""
        with tempfile.TemporaryDirectory() as directory:
            data = Path(directory) / 'gh.json'
            data.write_text(
                json.dumps({'responses': {'GET items': {'__pages__': [[{'id': 1}], [{'id': 2}]]}}}),
                encoding='utf-8',
            )
            with mock.patch.dict(os.environ, {'FAKE_GH_DATA': str(data)}):
                api = GhApi(f'{sys.executable} {ROOT / "tools/change_control/fixtures/fake_gh.py"}', 'acme/project')
                self.assertEqual(api('GET', 'items', paginate=True), [{'id': 1}, {'id': 2}])

    def test_gh_api_merges_all_paginated_collection_pages(self):
        """Check and workflow evidence must include every GitHub response page."""
        with tempfile.TemporaryDirectory() as directory:
            data = Path(directory) / 'gh.json'
            pages = [{'check_runs': [{'id': 1}]}, {'check_runs': [{'id': 2}]}]
            data.write_text(
                json.dumps({'responses': {'GET checks': {'__pages__': pages}}}),
                encoding='utf-8',
            )
            with mock.patch.dict(os.environ, {'FAKE_GH_DATA': str(data)}):
                api = GhApi(f'{sys.executable} {ROOT / "tools/change_control/fixtures/fake_gh.py"}', 'acme/project')
                self.assertEqual(
                    api('GET', 'checks', paginate=True, collection_key='check_runs'),
                    {'check_runs': [{'id': 1}, {'id': 2}]},
                )

    def test_workflows_disable_writes_for_fork_pull_requests(self):
        """Fork pull requests must still run checks without attempting unavailable comment or label writes."""
        change = (ROOT / '.github/workflows/change-control.yml').read_text(encoding='utf-8')
        coverage = (ROOT / '.github/workflows/coverage-delta.yml').read_text(encoding='utf-8')
        wire = (ROOT / '.github/workflows/wire-break.yml').read_text(encoding='utf-8')
        self.assertIn('CAN_COMMENT', change)
        self.assertIn('--no-comment', change)
        self.assertIn('CAN_COMMENT', coverage)
        self.assertIn('--no-comment', coverage)
        self.assertIn('CAN_LABEL', wire)

    def test_warn_mode_exits_zero_with_findings(self):
        """Warn mode must report findings while returning success to the caller."""
        result = self._run_cli('warn', {'responses': {}})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('mode: warn', result.stdout)

    def test_enforce_mode_exits_one_with_findings(self):
        """Enforce mode must return one for the same findings that warn mode tolerates."""
        result = self._run_cli('enforce', {'responses': {}})
        self.assertEqual(result.returncode, 1, result.stderr)

    def test_mode_file_rejects_unknown_value(self):
        """An invalid mode must return two rather than silently defaulting to warn."""
        result = self._run_cli('maybe', {'responses': {}})
        self.assertEqual(result.returncode, 2)

    def test_gh_api_failure_returns_two(self):
        """A failed gh subprocess must make the checker unable to run, not create policy findings."""
        result = self._run_cli('warn', {'exit_code': 1, 'stderr': 'API unavailable'})
        self.assertEqual(result.returncode, 2)
        self.assertIn('API unavailable', result.stderr)

    def test_partial_json_returns_two(self):
        """A partial GitHub response must fail closed as an execution error."""
        responses = self._responses()
        responses['GET repos/acme/project/pulls/7'] = {'body': 'Closes #17'}
        result = self._run_cli('warn', {'responses': responses})
        self.assertEqual(result.returncode, 2)

    def test_no_comment_avoids_write_api(self):
        """Fork-safe change-control checks must not call the comment API when writes are unavailable."""
        result, calls = self._run_cli('warn', {'responses': self._responses()}, no_comment=True, record_calls=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('POST repos/acme/project/issues/7/comments', calls)

    def test_warn_mode_survives_comment_permission_failure(self):
        """Warn-mode findings remain visible in logs when GitHub denies advisory comment writes."""
        responses = self._responses()
        responses.pop('POST repos/acme/project/issues/7/comments')
        result = self._run_cli('warn', {'responses': responses})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('comment publication warning', result.stderr)

    def _responses(self):
        data = snapshot()
        data['pr']['labels'] = []
        return {
            'GET repos/acme/project/pulls/7': data['pr'],
            'GET repos/acme/project/pulls/7/files': data['files'],
            'GET repos/acme/project/issues/17': data['issue'],
            'GET repos/acme/project/issues/17/comments': data['issue_comments'],
            'GET repos/acme/project/pulls/7/reviews': data['reviews'],
            'GET repos/acme/project/commits/abc123/check-runs': {'check_runs': data['check_runs']},
            'GET repos/acme/project/actions/runs?head_sha=abc123': {'workflow_runs': data['workflow_runs']},
            'GET repos/acme/project/issues/7/comments': [],
            'POST repos/acme/project/issues/7/comments': {},
        }

    def _run_cli(self, mode, fake_data, no_comment=False, record_calls=False):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'docs/process').mkdir(parents=True)
            (root / 'docs/process/enforcement-mode').write_text(mode + '\n', encoding='utf-8')
            (root / 'docs/process/templates').mkdir()
            shutil.copy2(
                ROOT / 'docs/process/templates/IMPACT_ANALYSIS.md', root / 'docs/process/templates/IMPACT_ANALYSIS.md'
            )
            (root / 'docs/safety').mkdir(parents=True)
            shutil.copy2(ROOT / 'docs/safety/SAFETY_REQUIREMENTS.md', root / 'docs/safety/SAFETY_REQUIREMENTS.md')
            (root / '.github/ISSUE_TEMPLATE').mkdir(parents=True)
            shutil.copy2(FORM, root / '.github/ISSUE_TEMPLATE/change-request.yml')
            data_path = root / 'gh.json'
            data_path.write_text(
                json.dumps(
                    fake_data
                    if fake_data.get('exit_code')
                    else {'responses': fake_data.get('responses') or self._responses()}
                ),
                encoding='utf-8',
            )
            environment = os.environ.copy()
            environment['FAKE_GH_DATA'] = str(data_path)
            environment['PYTHONPATH'] = str(ROOT)
            calls_path = root / 'gh-calls.txt'
            if record_calls:
                environment['FAKE_GH_CALLS'] = str(calls_path)
            command = [
                sys.executable,
                '-m',
                'tools.change_control',
                '--root',
                str(root),
                '--repository',
                'acme/project',
                '--pr',
                '7',
                '--gh',
                f'{sys.executable} {ROOT / "tools/change_control/fixtures/fake_gh.py"}',
            ]
            if no_comment:
                command.append('--no-comment')
            result = subprocess.run(
                command,
                cwd=ROOT,
                env=environment,
                check=False,
                capture_output=True,
                text=True,
            )
            if record_calls:
                calls = calls_path.read_text(encoding='utf-8') if calls_path.exists() else ''
                return result, calls
            return result


if __name__ == '__main__':
    unittest.main(verbosity=2)
