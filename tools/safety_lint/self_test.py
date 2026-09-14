#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Specification tests for the checked-in safety traceability linter."""

import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
FIXTURE = Path(__file__).with_name('fixtures') / 'repository'

sys.path.insert(0, str(REPO))

from tools.safety_lint.__main__ import _load_baseline  # noqa: E402
from tools.safety_lint.checks import (  # noqa: E402
    SRS_STATUSES,
    TRACE_STATUSES,
    apply_baseline,
    check_numeric_coverage_claims,
    check_summary_prose,
    run_checks,
)
from tools.safety_lint.coverage import compute_coverage  # noqa: E402
from tools.safety_lint.model import LintError  # noqa: E402
from tools.safety_lint.parse_srs import expand_allocations, parse_srs, split_row  # noqa: E402
from tools.safety_lint.parse_system_definition import parse_system_definition  # noqa: E402
from tools.safety_lint.parse_traceability import parse_traceability  # noqa: E402
from tools.safety_lint.render import render_traceability  # noqa: E402
from tools.safety_lint.runner import analyze  # noqa: E402


class FixtureRepo(unittest.TestCase):
    """Copy a caller-visible markdown repository for each adversarial test."""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        shutil.copytree(FIXTURE, self.root, dirs_exist_ok=True)
        self.replace(
            'docs/safety/TRACEABILITY.md',
            '- **(a) SRs with ≥1 passing verifying test: 2 / 2 = 100 %**\n'
            '- **Strict, fully-verified only: 1 / 2 = 50.0 %.**\n'
            '- **(b) Safety functions F-xx traced to ≥1 SR: 1 / 1 = 100 %.**\n'
            '  Excluding the two declared-non-safety functions: 1 / 1 = 100 %.\n',
            'Coverage values are owned by the generated regions above.\n',
        )

    def tearDown(self):
        self.temp.cleanup()

    def replace(self, relative, old, new):
        path = self.root / relative
        path.write_text(path.read_text(encoding='utf-8').replace(old, new), encoding='utf-8')

    def findings(self):
        return run_checks(analyze(self.root))

    def assert_check(self, check_id, subject=None):
        matches = [f for f in self.findings() if f.check_id == check_id]
        if subject is not None:
            matches = [f for f in matches if f.subject == subject]
        self.assertTrue(matches, f'expected {check_id} {subject or ""}')


class ParserTests(FixtureRepo):
    def test_parses_all_sr_areas(self):
        """The canonical SRS exposes at least one requirement in every declared area."""
        areas = {row.area for row in parse_srs(REPO / 'docs/safety/SAFETY_REQUIREMENTS.md')}
        self.assertEqual(areas, {'SYS', 'R', 'H', 'M', 'I'})

    def test_sr_ids_unique(self):
        """Every canonical requirement ID identifies exactly one SRS row."""
        rows = parse_srs(REPO / 'docs/safety/SAFETY_REQUIREMENTS.md')
        self.assertEqual(len({row.sr_id for row in rows}), len(rows))

    def test_allocated_to_range_expansion(self):
        """Compact function ranges expand to every member without dropping other allocations."""
        self.assertEqual(
            set(expand_allocations('F-R-01..05, F-H-03')),
            {'F-R-01', 'F-R-02', 'F-R-03', 'F-R-04', 'F-R-05', 'F-H-03'},
        )

    def test_allocated_to_slash_expansion(self):
        """Compact slash allocations expand to complete function IDs."""
        self.assertEqual(expand_allocations('F-M-03/04'), ('F-M-03', 'F-M-04'))

    def test_status_longest_match_wins(self):
        """A partial SRS status is never inflated to a fully satisfied status."""
        rows = parse_srs(self.root / 'docs/safety/SAFETY_REQUIREMENTS.md')
        self.assertEqual(rows[1].status, 'Partially satisfied')

    def test_status_vocabularies_are_defined_longest_match_first(self):
        """Checks defines both closed status vocabularies in longest-match-first order."""
        self.assertEqual(SRS_STATUSES, ('Partially satisfied', 'Residual-accepted', 'Satisfied', 'Gap'))
        self.assertEqual(TRACE_STATUSES, ('Partially-verified', 'Residual-accepted', 'Unverified-gap', 'Verified'))

    def test_pipe_inside_code_span_does_not_split_cell(self):
        """A pipe inside an inline code span remains part of its markdown cell."""
        self.assertEqual(split_row('| a | `b|c` | d |'), ['a', '`b|c`', 'd'])

    def test_every_trace_row_has_sr_id(self):
        """Malformed matrix rows cannot silently disappear as non-requirement content."""
        self.replace('docs/safety/TRACEABILITY.md', '| SR-R-01 |', '| BAD-ID |')
        with self.assertRaises(LintError):
            parse_traceability(self.root)

    def test_test_shorthand_expands_to_existing_paths(self):
        """Every documented shorthand expands to its exact existing repository path."""
        self.replace(
            'docs/safety/TRACEABILITY.md',
            'test_unique_probe.py',
            'EV, MR[B/D], HIL10, HIL20, HIL30, JL, REQ 2_02/2_03',
        )
        rows, _, issues = parse_traceability(self.root)
        self.assertEqual(
            set(rows[1].test_refs),
            {
                'firmware/test/test_estop_verdict.c',
                'tools/pstop_multi_remote_test.py',
                'tools/hil/test_10_button.py',
                'tools/hil/test_20_discordance.py',
                'tools/hil/test_30_power_cycle.py',
                'ros2/protective_stop_machine/test/test_json_lite.cpp',
                'pstop_c/pstop/test/src/pstop/requirements/req_2_02_test.c',
                'pstop_c/pstop/test/src/pstop/requirements/req_2_03_test.c',
            },
        )
        self.assertFalse([i for i in issues if i.kind == 'missing-shorthand'])

    def test_compact_hil_slash_form_resolves_both_paths(self):
        """Compact HIL20/30 notation resolves both independently existing HIL sources."""
        self.replace('docs/safety/TRACEABILITY.md', 'test_unique_probe.py', 'HIL20/30')
        rows, _, _ = parse_traceability(self.root)
        self.assertEqual(
            rows[1].test_refs,
            ('tools/hil/test_20_discordance.py', 'tools/hil/test_30_power_cycle.py'),
        )

    def test_mr_group_selector_yields_single_path(self):
        """MR group selectors identify one test source, not one source per group letter."""
        rows, _, _ = parse_traceability(self.root)
        self.assertEqual(rows[0].test_refs.count('tools/pstop_multi_remote_test.py'), 1)

    def test_req_slash_form_yields_two_paths(self):
        """A slash-separated REQ citation resolves each independent requirement test."""
        rows, _, _ = parse_traceability(self.root)
        self.assertEqual(len([p for p in rows[0].test_refs if 'req_' in p]), 2)

    def test_no_test_marker_detected_alongside_real_tests(self):
        """A scoped NO TEST note does not erase real evidence cited in the same cell."""
        rows, _, _ = parse_traceability(self.root)
        self.assertTrue(rows[0].has_no_test_marker and rows[0].test_refs)

    def test_reverse_map_flags_declared_non_safety(self):
        """An explicit non-safety reverse-map declaration remains distinguishable from a hole."""
        _, reverse, _ = parse_traceability(REPO)
        self.assertTrue(reverse['F-R-08'].declared_non_safety)

    def test_system_definition_function_set_nonempty(self):
        """The authoritative function decomposition yields a nonempty function set."""
        self.assertTrue(parse_system_definition(REPO / 'docs/safety/SYSTEM_DEFINITION.md'))

    def test_unique_repository_wide_stem_resolves(self):
        """A repository-wide unique test stem resolves without directory guessing."""
        decoy = self.root / 'tools/safety_lint/fixtures/repository/tests/test_unique_probe.py'
        decoy.parent.mkdir(parents=True)
        decoy.write_text('# synthetic fixture, not project evidence\n', encoding='utf-8')
        self.replace('docs/safety/TRACEABILITY.md', 'test_unique_probe.py', 'test_unique_probe')
        rows, _, issues = parse_traceability(self.root)
        self.assertIn('tests/test_unique_probe.py', rows[1].test_refs)
        self.assertFalse([i for i in issues if i.literal == 'test_unique_probe'])

    def test_missing_bare_test_stem_is_not_resolved(self):
        """A missing bare test stem produces an issue but never a resolved evidence path."""
        self.replace('docs/safety/TRACEABILITY.md', 'test_unique_probe.py', 'test_deleted_source')
        rows, _, issues = parse_traceability(self.root)
        self.assertFalse(rows[1].test_refs)
        self.assertTrue([issue for issue in issues if issue.literal == 'test_deleted_source'])

    def test_ambiguous_repository_wide_stem_is_finding(self):
        """An ambiguous test stem is reported instead of selecting the nearest file."""
        (self.root / 'other').mkdir()
        (self.root / 'other/test_unique_probe.py').write_text('# duplicate\n', encoding='utf-8')
        self.replace('docs/safety/TRACEABILITY.md', 'test_unique_probe.py', 'test_unique_probe')
        _, _, issues = parse_traceability(self.root)
        self.assertTrue([i for i in issues if i.kind == 'ambiguous' and i.literal == 'test_unique_probe'])

    def test_hil_report_must_name_sr(self):
        """An evidence report counts only when its content names the cited requirement."""
        (self.root / 'docs/evidence.md').write_text('# HIL evidence for another SR\n', encoding='utf-8')
        self.replace('docs/safety/TRACEABILITY.md', 'test_unique_probe.py', 'docs/evidence.md')
        _, _, issues = parse_traceability(self.root)
        self.assertTrue([i for i in issues if i.kind == 'report-does-not-name-sr'])

    def test_prose_does_not_infer_evidence(self):
        """Words describing a successful test never become a test-file citation."""
        self.replace('docs/safety/TRACEABILITY.md', 'test_unique_probe.py', 'bench test passed 8/8')
        rows, _, _ = parse_traceability(self.root)
        self.assertFalse(rows[1].test_refs)

    def test_unique_production_source_basename_is_not_test_evidence(self):
        """A unique production source basename cannot satisfy a verifying-test citation."""
        self.replace('docs/safety/TRACEABILITY.md', 'test_unique_probe.py', 'code.c')
        rows, _, _ = parse_traceability(self.root)
        self.assertFalse(rows[1].test_refs)

    def test_bare_test_filename_with_line_suffix_resolves_file(self):
        """A bare test filename citation may carry a line suffix without changing its target."""
        self.replace('docs/safety/TRACEABILITY.md', 'test_unique_probe.py', 'test_unique_probe.py:10')
        rows, _, _ = parse_traceability(self.root)
        self.assertEqual(rows[1].test_refs, ('tests/test_unique_probe.py',))

    def test_explicit_test_path_with_line_range_resolves_file(self):
        """An explicit test path may carry a line range without changing its target."""
        self.replace('docs/safety/TRACEABILITY.md', 'test_unique_probe.py', 'tests/test_unique_probe.py:10-20')
        rows, _, _ = parse_traceability(self.root)
        self.assertEqual(rows[1].test_refs, ('tests/test_unique_probe.py',))

    def test_attached_test_symbol_uses_preceding_file_citation(self):
        """A parenthesized test symbol attached to file:line does not become a missing stem."""
        self.replace(
            'docs/safety/TRACEABILITY.md',
            'test_unique_probe.py',
            '`test_unique_probe.py:10` (`test_named_case`)',
        )
        rows, _, issues = parse_traceability(self.root)
        self.assertEqual(rows[1].test_refs, ('tests/test_unique_probe.py',))
        self.assertFalse([issue for issue in issues if issue.literal == 'test_named_case'])

    def test_real_machine_test_line_citations_resolve_without_opening_symbols(self):
        """SR-SYS-09 file:line citations resolve while their attached symbols add no findings."""
        rows, _, issues = parse_traceability(REPO)
        row = next(row for row in rows if row.sr_id == 'SR-SYS-09')
        self.assertIn('pstop_c/pstop/test/src/pstop/machine_test.c', row.test_refs)
        self.assertFalse([
            issue
            for issue in issues
            if issue.sr_id == 'SR-SYS-09'
            and issue.literal in {'test_bond_stop_ok_stop_only_operator', 'test_2_clients_stop_only_stop'}
        ])


class ConsistencyTests(FixtureRepo):
    def test_c1_flags_sr_set_mismatch(self):
        """C1 reports an SRS requirement omitted from the matrix."""
        self.replace(
            'docs/safety/TRACEABILITY.md',
            '| SR-R-01 | F-R-01 | code.c:1 | test_unique_probe.py | Test | **Verified** |\n',
            '',
        )
        self.assert_check('C1', 'SR-R-01')

    def test_c2_flags_unknown_trace_status(self):
        """C2 rejects matrix statuses outside the documented closed vocabulary."""
        self.replace('docs/safety/TRACEABILITY.md', '**Verified** |', '**Complete** |')
        with self.assertRaises(LintError):
            analyze(self.root)

    def test_c3_flags_verified_without_evidence(self):
        """C3 rejects a Verified row whose evidence cell resolves no test path."""
        self.replace('docs/safety/TRACEABILITY.md', 'test_unique_probe.py', 'NO TEST')
        self.assert_check('C3', 'SR-R-01')

    def test_c4_flags_missing_cited_test(self):
        """C4 identifies every SR affected by a cited test file removed from the tree."""
        (self.root / 'tests/test_unique_probe.py').unlink()
        self.assert_check('C4', 'SR-R-01')

    def test_missing_shorthand_removes_ref_reduces_coverage_and_triggers_c3_c4(self):
        """Deleting sole shorthand evidence removes coverage and reports both evidence checks."""
        self.replace('docs/safety/TRACEABILITY.md', 'test_unique_probe.py', 'HIL10')
        before = compute_coverage(analyze(self.root)).cited_tests
        (self.root / 'tools/hil/test_10_button.py').unlink()
        analysis = analyze(self.root)
        row = next(row for row in analysis.trace if row.sr_id == 'SR-R-01')
        findings = run_checks(analysis)
        self.assertEqual((row.test_refs, compute_coverage(analysis).cited_tests), ((), before - 1))
        self.assertTrue({finding.check_id for finding in findings if finding.subject == 'SR-R-01'} >= {'C3', 'C4'})

    def test_missing_req_removes_ref_and_triggers_c3_c4(self):
        """Deleting sole REQ evidence leaves a Partial row unresolved and reports C3 plus C4."""
        self.replace('docs/safety/TRACEABILITY.md', 'test_unique_probe.py', 'REQ 2_02')
        (self.root / 'pstop_c/pstop/test/src/pstop/requirements/req_2_02_test.c').unlink()
        analysis = analyze(self.root)
        row = next(row for row in analysis.trace if row.sr_id == 'SR-R-01')
        findings = run_checks(analysis)
        self.assertFalse(row.test_refs)
        self.assertTrue({finding.check_id for finding in findings if finding.subject == 'SR-R-01'} >= {'C3', 'C4'})

    def test_c5_flags_missing_code_file(self):
        """C5 rejects an explicit code citation whose repository file is absent."""
        self.replace('docs/safety/TRACEABILITY.md', 'code.c:1', 'missing.c:1')
        self.assert_check('C5', 'SR-SYS-01')

    def test_c6_flags_unknown_function(self):
        """C6 rejects allocations outside the authoritative function decomposition."""
        self.replace('docs/safety/SAFETY_REQUIREMENTS.md', 'F-R-01 |', 'F-X-99 |')
        self.assert_check('C6', 'F-X-99')

    def test_c7_flags_unknown_upstream_reference(self):
        """C7 warns when a requirement cites an absent hazard, goal, or DU identifier."""
        self.replace('docs/safety/SAFETY_REQUIREMENTS.md', 'SG-1, H-01, DU-1', 'SG-9, H-99, DU-9')
        self.assert_check('C7', 'SG-9')

    def test_c8_flags_reverse_map_disagreement(self):
        """C8 rejects a reverse map that disagrees with forward requirement allocation."""
        self.replace(
            'docs/safety/TRACEABILITY.md', '| F-R-01 | Sense | SR-SYS-01, SR-R-01 |', '| F-R-01 | Sense | SR-SYS-01 |'
        )
        self.assert_check('C8', 'F-R-01')

    def test_c9_flags_srs_matrix_allocation_disagreement(self):
        """C9 rejects differing SRS and matrix allocations for the same requirement."""
        self.replace('docs/safety/TRACEABILITY.md', '| SR-R-01 | F-R-01 |', '| SR-R-01 | F-R-02 |')
        self.assert_check('C9', 'SR-R-01')

    def test_baseline_suppresses_exact_matching_finding(self):
        """A justified baseline entry suppresses exactly one matching finding message."""
        self.replace('docs/safety/TRACEABILITY.md', 'test_unique_probe.py', 'NO TEST')
        finding = next(finding for finding in self.findings() if finding.check_id == 'C3')
        active, suppressed = apply_baseline(
            self.findings(),
            {('C3', 'SR-R-01', finding.message): {'reason': 'fixture', 'owner': 'test'}},
        )
        self.assertFalse([f for f in active if f.check_id == 'C3'])
        self.assertTrue([f for f in suppressed if f.check_id == 'C3'])

    def test_same_subject_new_finding_is_not_suppressed(self):
        """A new message on an already-baselined subject remains an active violation."""
        self.replace(
            'docs/safety/TRACEABILITY.md',
            'test_unique_probe.py',
            'test_missing_one, test_missing_two',
        )
        findings = self.findings()
        existing = next(finding for finding in findings if 'test_missing_one' in finding.message)
        active, _ = apply_baseline(
            findings,
            {('C4', 'SR-R-01', existing.message): {'reason': 'fixture', 'owner': 'test'}},
        )
        self.assertFalse([finding for finding in active if finding.check_id == 'BASELINE'])
        self.assertTrue([
            finding
            for finding in active
            if finding.check_id == 'C4' and finding.subject == 'SR-R-01' and 'test_missing_two' in finding.message
        ])

    def test_stale_exact_baseline_entry_is_an_error(self):
        """An exact baseline entry with no current finding fails the ratchet as stale."""
        active, _ = apply_baseline(
            self.findings(), {('C8', 'absent', 'old exact finding'): {'reason': 'fixture', 'owner': 'test'}}
        )
        self.assertTrue([f for f in active if f.check_id == 'BASELINE' and f.severity == 'error'])

    def test_duplicate_exact_baseline_entries_are_rejected(self):
        """The baseline loader rejects duplicate exact finding discriminators."""
        entry = {
            'check_id': 'C4',
            'subject': 'SR-R-01',
            'finding': 'missing fixture',
            'reason': 'fixture reason',
            'owner': 'test',
        }
        path = self.root / 'duplicate-baseline.json'
        path.write_text(json.dumps({'findings': [entry, entry]}), encoding='utf-8')
        with self.assertRaises(LintError):
            _load_baseline(path)

    def test_undocumented_srs_status_is_informational(self):
        """A valid status used by requirements but omitted from conventions is informationally visible."""
        findings = self.findings()
        self.assertTrue([f for f in findings if f.check_id == 'C2' and f.severity == 'info'])


class NumericCoverageOwnershipTests(unittest.TestCase):
    def test_reconciled_real_section_has_no_c10_finding(self):
        """The reconciled real summary keeps all numeric requirements coverage inside generated regions."""
        text = (REPO / 'docs/safety/TRACEABILITY.md').read_text(encoding='utf-8')
        self.assertEqual(check_numeric_coverage_claims(text), ())

    def test_marker_bounded_numeric_claims_are_ignored(self):
        """Generated coverage regions exclusively own every numeric claim they contain."""
        text = """## 3. Requirements coverage summary
<!-- BEGIN GENERATED: safety-lint headline -->
32 / 40 = 80.0 %; 17 Verified; 14 Partially-verified
<!-- END GENERATED: safety-lint headline -->
<!-- BEGIN GENERATED: safety-lint areas -->
22 / 27 = 81.5 %
<!-- END GENERATED: safety-lint areas -->
**Reading:** 56.8 % branch coverage
"""
        self.assertEqual(check_numeric_coverage_claims(text), ())

    def test_new_outside_claim_changes_exact_message(self):
        """Any added outside-marker ratio or percentage breaks an exact C10 baseline."""
        base = """## 3. Requirements coverage summary
Legacy coverage: 2 / 3 = 66.7 %.
**Reading:** details
"""
        changed = base.replace('**Reading:**', 'Another claim: 75 %.\n**Reading:**')
        before = check_numeric_coverage_claims(base)[0]
        after = check_numeric_coverage_claims(changed)[0]
        self.assertEqual(before.subject, after.subject)
        self.assertNotEqual(before.message, after.message)
        self.assertIn('75 %', after.message)

    def test_identifiers_date_and_reconciliation_delta_do_not_trigger(self):
        """Compact IDs, dates, and reconciliation deltas are not coverage values."""
        text = """## 3. Requirements coverage summary
Reconciled 2026-08-07: +4 for SR-M-01/03/05 and DU-1/2/3/4.
**Reading:** details
"""
        self.assertEqual(check_numeric_coverage_claims(text), ())

    def test_structural_percentages_after_reading_do_not_trigger(self):
        """Structural coverage in Reading is outside C10's requirements-summary scope."""
        text = """## 3. Requirements coverage summary
Coverage details are generated above.
**Reading:** MC-DC 100 %, branch 56.8 %, line ~89 %.
"""
        self.assertEqual(check_numeric_coverage_claims(text), ())

    def test_later_sections_are_not_scanned_without_reading_paragraph(self):
        """Coverage-like numbers in section 4 cannot become section-3 ownership findings."""
        text = """## 3. Requirements coverage summary
Coverage details are generated above.
## 4. Test-gap register
Historical result: 4 / 5 = 80 %.
"""
        self.assertEqual(check_numeric_coverage_claims(text), ())

    def test_clean_pointer_and_reconciliation_section_has_no_finding(self):
        """Pointers and nonnumeric history may remain outside generated coverage regions."""
        text = """## 3. Requirements coverage summary
<!-- BEGIN GENERATED: safety-lint headline -->
32 / 40 = 80.0 %
<!-- END GENERATED: safety-lint headline -->
See the generated headline and area table. Reconciled 2026-08-07: +4 for
SR-M-01/03/05 and DU-1/2/3/4 after evidence review.
**Reading:** structural coverage is discussed here at 100 %.
"""
        self.assertEqual(check_numeric_coverage_claims(text), ())


class CoverageRenderCliTests(FixtureRepo):
    def test_coverage_matches_committed_summary(self):
        """Real citation and status counts reproduce the ratified committed summary."""
        result = analyze(REPO)
        coverage = compute_coverage(result)
        self.assertEqual((coverage.total, coverage.cited_tests, coverage.verified), (40, 32, 17))

    def test_generated_block_is_idempotent(self):
        """Rendering an already rendered traceability document is byte-idempotent."""
        result = analyze(self.root)
        original = (self.root / 'docs/safety/TRACEABILITY.md').read_text(encoding='utf-8')
        once = render_traceability(original, compute_coverage(result))
        self.assertEqual(render_traceability(once, compute_coverage(result)), once)

    def test_check_mode_detects_stale_block(self):
        """Check mode returns one when a generated numeric region is stale."""
        self.assertEqual(self.run_cli('--write').returncode, 0)
        self.replace(
            'docs/safety/TRACEABILITY.md',
            'SRs with at least one cited verifying test: 2 / 2',
            'SRs with at least one cited verifying test: 1 / 2',
        )
        proc = self.run_cli('--check')
        self.assertEqual(proc.returncode, 1)

    def test_stale_check_names_exact_write_remedy(self):
        """Stale check output gives the exact command that refreshes generated regions."""
        proc = self.run_cli('--check')
        self.assertIn(
            'docs/safety/TRACEABILITY.md: generated regions are stale; run python3 -m tools.safety_lint --write',
            proc.stdout,
        )

    def test_render_does_not_touch_prose_outside_markers(self):
        """Rendering preserves hand-authored Reading prose byte-for-byte."""
        path = self.root / 'docs/safety/TRACEABILITY.md'
        before = path.read_text(encoding='utf-8').split('**Reading:**', 1)[1]
        after = render_traceability(path.read_text(encoding='utf-8'), compute_coverage(analyze(self.root))).split(
            '**Reading:**', 1
        )[1]
        self.assertEqual(after, before)

    def test_renderer_uses_unicode_greater_than_or_equal(self):
        """The generated area heading preserves the document's Unicode comparison symbol."""
        text = (self.root / 'docs/safety/TRACEABILITY.md').read_text(encoding='utf-8')
        rendered = render_traceability(text, compute_coverage(analyze(self.root)))
        self.assertIn('≥1 cited test %', rendered)
        self.assertNotIn('>=', rendered)

    def test_new_numeric_prose_outside_markers_is_a_c10_failure(self):
        """A newly introduced numeric claim outside generated markers remains a check failure."""
        text = (REPO / 'docs/safety/TRACEABILITY.md').read_text(encoding='utf-8')
        stale = text.replace('**Reading:**', 'Legacy claim: 21 / 27 = 77.8 %.\n\n**Reading:**')
        self.assertTrue(check_numeric_coverage_claims(stale))

    def test_missing_legacy_summary_claims_do_not_trigger_summary(self):
        """Removing superseded hand-authored headlines does not create SUMMARY errors."""
        coverage = compute_coverage(analyze(self.root))
        self.assertEqual(check_summary_prose('No hand-authored numeric coverage claims.\n', coverage), ())

    def test_cli_exit_code_zero_on_clean_tree(self):
        """A repository with no active or stale findings returns the clean exit code."""
        proc = self.run_cli()
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)

    def test_cli_exit_code_one_on_injected_error(self):
        """A parseable traceability parity defect returns the check-failed exit code."""
        self.replace(
            'docs/safety/TRACEABILITY.md',
            '| SR-R-01 | F-R-01 | code.c:1 | test_unique_probe.py | Test | **Verified** |\n',
            '',
        )
        self.assertEqual(self.run_cli().returncode, 1)

    def test_write_refuses_active_error_without_touching_traceability(self):
        """Write mode returns one and preserves bytes when consistency errors are active."""
        self.replace(
            'docs/safety/TRACEABILITY.md',
            '| SR-R-01 | F-R-01 | code.c:1 | test_unique_probe.py | Test | **Verified** |\n',
            '',
        )
        path = self.root / 'docs/safety/TRACEABILITY.md'
        before = path.read_bytes()
        proc = self.run_cli('--write')
        self.assertEqual((proc.returncode, path.read_bytes()), (1, before))

    def test_normal_cli_reports_citation_execution_limitation(self):
        """Normal CLI output explicitly says citation resolution does not verify execution."""
        proc = self.run_cli()
        self.assertIn('Citation limitation: resolution does not verify test execution or passing state.', proc.stdout)

    def test_cli_exit_code_two_on_missing_document(self):
        """A missing required safety document returns the cannot-run exit code."""
        (self.root / 'docs/safety/SAFETY_REQUIREMENTS.md').unlink()
        self.assertEqual(self.run_cli().returncode, 2)

    def test_workflow_has_no_path_filters(self):
        """CI runs on every pull request and every main push without path filtering."""
        text = (REPO / '.github/workflows/safety-lint.yml').read_text(encoding='utf-8')
        self.assertNotIn('paths:', text)
        self.assertIn('pull_request:', text)
        self.assertIn('branches: [main]', text)

    def run_cli(self, *args):
        return subprocess.run(
            [sys.executable, '-m', 'tools.safety_lint', '--root', str(self.root), *args],
            cwd=REPO,
            capture_output=True,
            text=True,
            check=False,
        )


if __name__ == '__main__':
    unittest.main(verbosity=2)
