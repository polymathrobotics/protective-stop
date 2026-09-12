# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Compute and enforce the normalized pstop_c public wire-header signature."""

import argparse
import hashlib
import re
import sys
from dataclasses import dataclass
from pathlib import Path

HEADER_NAMES = (
    'config.h',
    'constants.h',
    'protocol.h',
    'protocol_data.h',
    'pstop_msg.h',
    'checksum.h',
    'device_id.h',
    'endian.h',
)
HEADER_PREFIX = 'pstop_c/pstop/include/pstop/'


@dataclass(frozen=True)
class WireSignature:
    """Per-header evidence and the aggregate normalized wire signature."""

    version: str
    message_size: str
    headers: dict
    aggregate: str


def _strip_comments(text):
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.DOTALL)
    text = re.sub(r'//[^\n]*', ' ', text)
    return text


def _literal(text, name):
    match = re.search(rf'^\s*#\s*define\s+{name}\s+(\S+)', _strip_comments(text), re.MULTILINE)
    if not match:
        raise ValueError(f'{name} not found in config.h')
    return match.group(1)


def compute_signature(root):
    """Hash comment-free, whitespace-collapsed headers plus explicit protocol literals."""
    include = Path(root) / HEADER_PREFIX
    normalized = {}
    for name in HEADER_NAMES:
        path = include / name
        if not path.is_file():
            raise FileNotFoundError(path)
        normalized[name] = ' '.join(_strip_comments(path.read_text(encoding='utf-8')).split())
    config = (include / 'config.h').read_text(encoding='utf-8')
    version = _literal(config, 'PSTOP_VERSION')
    message_size = _literal(config, 'PSTOP_MESSAGE_SIZE')
    header_hashes = {name: hashlib.sha256(normalized[name].encode()).hexdigest() for name in HEADER_NAMES}
    payload = ''.join(f'{name}\0{normalized[name]}\0' for name in HEADER_NAMES)
    payload += f'PSTOP_VERSION\0{version}\0PSTOP_MESSAGE_SIZE\0{message_size}\0'
    return WireSignature(version, message_size, header_hashes, hashlib.sha256(payload.encode()).hexdigest())


def read_expected(path):
    """Read the reviewable line-oriented wire signature record."""
    values = {}
    headers = {}
    for raw in Path(path).read_text(encoding='utf-8').splitlines():
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) != 2:
            raise ValueError(f'invalid expected signature line: {raw}')
        key, value = parts
        if key in HEADER_NAMES:
            headers[key] = value
        else:
            values[key] = value
    if set(headers) != set(HEADER_NAMES) or not {'PSTOP_VERSION', 'PSTOP_MESSAGE_SIZE', 'aggregate'} <= set(values):
        raise ValueError('expected signature lacks version, size, aggregate, or per-header hashes')
    return WireSignature(values['PSTOP_VERSION'], values['PSTOP_MESSAGE_SIZE'], headers, values['aggregate'])


def render_signature(signature):
    """Render deterministic expected-signature content suitable for code review."""
    lines = [
        '# Normalized pstop_c wire headers; comments stripped and whitespace collapsed.',
        f'# Corresponds to PSTOP_VERSION {signature.version}.',
        f'PSTOP_VERSION {signature.version}',
        f'PSTOP_MESSAGE_SIZE {signature.message_size}',
    ]
    lines.extend(f'{name} {signature.headers[name]}' for name in HEADER_NAMES)
    lines.append(f'aggregate {signature.aggregate}')
    return '\n'.join(lines) + '\n'


def check_wire_format(root, expected_path, labels, changed_files, expectation_preexisted=True):
    """Return guard exit code and explanation for current headers, labels, and changed paths."""
    try:
        current = compute_signature(root)
        expected = read_expected(expected_path)
    except (OSError, ValueError) as error:
        return 2, f'wire-format: cannot run: {error}'
    mismatched = [name for name in HEADER_NAMES if current.headers[name] != expected.headers[name]]
    if current.version != expected.version or current.message_size != expected.message_size:
        if 'config.h' not in mismatched:
            mismatched.append('config.h')
    expectation_change = expectation_preexisted and 'tools/change_control/wire_format.sha256' in changed_files
    initial_header_change = not expectation_preexisted and any(path.startswith(HEADER_PREFIX) for path in changed_files)
    signature_changed = bool(mismatched or current.aggregate != expected.aggregate)
    required = {'wire-break', 'class-c'} if expectation_change or initial_header_change or signature_changed else set()
    missing_labels = sorted(required - set(labels))
    rollout = 'Remote and machine must be released and deployed together for a coordinated rollout.'
    if signature_changed:
        names = ', '.join(sorted(set(mismatched))) or 'aggregate signature'
        return 1, f'wire-format mismatch in: {names}. Update the reviewed expectation in this PR. {rollout}'
    if missing_labels:
        return (
            1,
            f'wire-format or expectation changed; required PR labels missing: {", ".join(missing_labels)}. {rollout}',
        )
    return (
        0,
        f'wire-format signature matches PSTOP_VERSION {current.version}, PSTOP_MESSAGE_SIZE {current.message_size}',
    )


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=('check', 'snapshot'))
    parser.add_argument('--root', default='.')
    parser.add_argument('--expected', default='tools/change_control/wire_format.sha256')
    parser.add_argument('--labels', default='')
    parser.add_argument('--changed-file', action='append', default=[])
    parser.add_argument('--initial-expectation', action='store_true')
    args = parser.parse_args(argv)
    if args.command == 'snapshot':
        try:
            print(render_signature(compute_signature(args.root)), end='')
            return 0
        except (OSError, ValueError) as error:
            print(f'wire-format: cannot run: {error}', file=sys.stderr)
            return 2
    labels = {label.strip() for label in args.labels.split(',') if label.strip()}
    code, message = check_wire_format(
        args.root,
        Path(args.root) / args.expected,
        labels,
        args.changed_file,
        expectation_preexisted=not args.initial_expectation,
    )
    print(message, file=sys.stderr if code == 2 else sys.stdout)
    return code


if __name__ == '__main__':
    sys.exit(main())
