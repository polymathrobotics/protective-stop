#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Spec-driven tests for wire-format change-control tooling."""

import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from tools.change_control.wire_format import WIRE_PATHS, check_wire_format, compute_signature  # noqa: E402

WIRE_EXPECTED = ROOT / 'tools/change_control/wire_format.sha256'
EXPECTED_WIRE_PATHS = (
    'pstop_c/pstop/include/pstop/config.h',
    'pstop_c/pstop/include/pstop/constants.h',
    'pstop_c/pstop/include/pstop/protocol.h',
    'pstop_c/pstop/include/pstop/protocol_data.h',
    'pstop_c/pstop/include/pstop/pstop_msg.h',
    'pstop_c/pstop/include/pstop/checksum.h',
    'pstop_c/pstop/include/pstop/device_id.h',
    'pstop_c/pstop/include/pstop/endian.h',
    'pstop_c/pstop/src/pstop/pstop_msg.c',
    'pstop_c/pstop/src/pstop/checksum.c',
    'pstop_c/pstop/src/pstop/endian.c',
)


class WireFormatTests(unittest.TestCase):
    def _copy_wire_files(self, directory):
        for relative in EXPECTED_WIRE_PATHS:
            destination = Path(directory) / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / relative, destination)
        return Path(directory)

    _copy_headers = _copy_wire_files

    def _run_wire_script(self, base_sha_marker, fake_git=False):
        environment = os.environ.copy()
        if base_sha_marker is None:
            environment.pop('PSTOP_BASE_SHA', None)
        else:
            environment['PSTOP_BASE_SHA'] = base_sha_marker
        marker = None
        temporary = None
        if fake_git:
            temporary = tempfile.TemporaryDirectory()
            directory = Path(temporary.name)
            marker = directory / 'git-called'
            fake = directory / 'git'
            fake.write_text(f'#!/usr/bin/env bash\ntouch "{marker}"\nexit 99\n', encoding='utf-8')
            fake.chmod(0o755)
            environment['PATH'] = f'{directory}:{environment["PATH"]}'
        result = subprocess.run(
            ['scripts/check_wire_format.sh'],
            cwd=ROOT,
            env=environment,
            check=False,
            capture_output=True,
            text=True,
        )
        git_called = marker.exists() if marker else False
        if temporary:
            temporary.cleanup()
        return result, git_called

    def test_wire_script_allows_unset_base_sha(self):
        """A local invocation with PSTOP_BASE_SHA unset must run the current-tree check."""
        result, _ = self._run_wire_script(None)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_wire_script_allows_empty_base_sha(self):
        """A local invocation with an empty PSTOP_BASE_SHA must run the current-tree check."""
        result, _ = self._run_wire_script('')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_wire_script_accepts_valid_full_base_sha(self):
        """An exact 40-character hexadecimal base SHA must reach the real git comparison seam."""
        head = subprocess.run(
            ['git', 'rev-parse', 'HEAD'], cwd=ROOT, check=True, capture_output=True, text=True
        ).stdout.strip()
        result, _ = self._run_wire_script(head)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_wire_script_rejects_short_base_sha_before_git(self):
        """A shortened base SHA must return cannot-run before invoking git."""
        result, git_called = self._run_wire_script('a' * 39, fake_git=True)
        self.assertEqual((result.returncode, git_called), (2, False))
        self.assertIn('cannot run', result.stderr.lower())

    def test_wire_script_rejects_nonhex_base_sha_before_git(self):
        """A 40-character nonhex base SHA must return cannot-run before invoking git."""
        result, git_called = self._run_wire_script('g' * 40, fake_git=True)
        self.assertEqual((result.returncode, git_called), (2, False))
        self.assertIn('PSTOP_BASE_SHA', result.stderr)

    def test_wire_script_rejects_option_like_base_sha_before_git(self):
        """An option-like base value must never be passed to git as a revision argument."""
        result, git_called = self._run_wire_script('--help', fake_git=True)
        self.assertEqual((result.returncode, git_called), (2, False))
        self.assertIn('PSTOP_BASE_SHA', result.stderr)

    def test_signature_stable_across_comment_only_change(self):
        """Adding a C comment to a watched header must not alter its signature."""
        with tempfile.TemporaryDirectory() as directory:
            root = self._copy_headers(directory)
            before = compute_signature(root)
            path = root / 'pstop_c/pstop/include/pstop/protocol.h'
            path.write_text(path.read_text(encoding='utf-8') + '\n/* comment only */\n', encoding='utf-8')
            self.assertEqual(compute_signature(root).aggregate, before.aggregate)

    def test_source_signatures_stay_stable_across_comment_only_changes(self):
        """Comments in each watched implementation file must not alter any wire signature."""
        with tempfile.TemporaryDirectory() as directory:
            root = self._copy_wire_files(directory)
            before = compute_signature(root)
            for relative in EXPECTED_WIRE_PATHS[-3:]:
                path = root / relative
                path.write_text(path.read_text(encoding='utf-8') + '\n/* comment only */\n', encoding='utf-8')
            after = compute_signature(root)
            self.assertEqual(after, before)

    def test_reordered_field_writes_change_pstop_message_source_and_aggregate_hashes(self):
        """Reordering adjacent encoded fields must change pstop_msg.c evidence and the aggregate."""
        relative = 'pstop_c/pstop/src/pstop/pstop_msg.c'
        with tempfile.TemporaryDirectory() as directory:
            root = self._copy_wire_files(directory)
            before = compute_signature(root)
            path = root / relative
            original = (
                '    write_uint32(msg->counter, data, &pos);\n    write_uint32(msg->received_counter, data, &pos);'
            )
            replacement = (
                '    write_uint32(msg->received_counter, data, &pos);\n    write_uint32(msg->counter, data, &pos);'
            )
            changed = path.read_text(encoding='utf-8').replace(original, replacement)
            self.assertNotEqual(changed, path.read_text(encoding='utf-8'))
            path.write_text(changed, encoding='utf-8')
            after = compute_signature(root)
            self.assertNotEqual(after.files[relative], before.files[relative])
            self.assertNotEqual(after.aggregate, before.aggregate)

    def test_crc_polynomial_change_changes_checksum_source_and_aggregate_hashes(self):
        """Changing the CRC polynomial must change checksum.c evidence and the aggregate."""
        relative = 'pstop_c/pstop/src/pstop/checksum.c'
        with tempfile.TemporaryDirectory() as directory:
            root = self._copy_wire_files(directory)
            before = compute_signature(root)
            path = root / relative
            path.write_text(path.read_text(encoding='utf-8').replace('0x8D95U', '0x8D96U'), encoding='utf-8')
            after = compute_signature(root)
            self.assertNotEqual(after.files[relative], before.files[relative])
            self.assertNotEqual(after.aggregate, before.aggregate)

    def test_byte_order_change_changes_endian_source_and_aggregate_hashes(self):
        """Changing one byte-order operation must change endian.c evidence and the aggregate."""
        relative = 'pstop_c/pstop/src/pstop/endian.c'
        with tempfile.TemporaryDirectory() as directory:
            root = self._copy_wire_files(directory)
            before = compute_signature(root)
            path = root / relative
            path.write_text(
                path.read_text(encoding='utf-8').replace(
                    'bytes[3] = (uint8_t)(value & 0xFFU);', 'bytes[2] = (uint8_t)(value & 0xFFU);'
                ),
                encoding='utf-8',
            )
            after = compute_signature(root)
            self.assertNotEqual(after.files[relative], before.files[relative])
            self.assertNotEqual(after.aggregate, before.aggregate)

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

    def test_source_mismatch_diagnostic_names_repository_path(self):
        """A source mismatch diagnostic must identify the changed repository-relative path."""
        relative = 'pstop_c/pstop/src/pstop/checksum.c'
        with tempfile.TemporaryDirectory() as directory:
            root = self._copy_wire_files(directory)
            shutil.copy2(WIRE_EXPECTED, root / 'wire_format.sha256')
            path = root / relative
            path.write_text(path.read_text(encoding='utf-8').replace('0x8D95U', '0x8D96U'), encoding='utf-8')
            code, message = check_wire_format(root, root / 'wire_format.sha256', {'wire-break', 'class-c'}, [])
            self.assertEqual(code, 1)
            self.assertIn(relative, message)

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

    def test_initial_snapshot_cannot_hide_a_source_change(self):
        """A watched source edit accompanying the first snapshot must still require both declarations."""
        relative = 'pstop_c/pstop/src/pstop/endian.c'
        code, message = check_wire_format(
            ROOT,
            WIRE_EXPECTED,
            {'wire-break'},
            [relative, 'tools/change_control/wire_format.sha256'],
            expectation_preexisted=False,
        )
        self.assertEqual(code, 1)
        self.assertIn('class-c', message)

    def test_workflow_initial_bootstrap_matcher_watches_all_wire_files(self):
        """The reviewable workflow matcher must include every watched header and implementation path."""
        workflow = (ROOT / '.github/workflows/wire-break.yml').read_text(encoding='utf-8')
        self.assertEqual(WIRE_PATHS, EXPECTED_WIRE_PATHS)
        for relative in EXPECTED_WIRE_PATHS:
            self.assertIn(relative, workflow)

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
