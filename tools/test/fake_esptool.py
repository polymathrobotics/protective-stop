#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""A shim standing in for the real esptool binary, for tests only.

Run as `[sys.executable, __file__, *args]` in place of esptool; it never opens
a port. All behavior is driven by environment variables:

FAKE_ESPTOOL_VERSION       (default "5.4.0")     what `version` reports
FAKE_ESPTOOL_VERSION_RC    (default "0")         exit code of `version`
FAKE_ESPTOOL_ARGV_LOG      (path, optional)      JSONL log of every non-version
                                                  invocation: one line per call,
                                                  {"argv": [...], "cwd": "..."},
                                                  appended, never truncated
FAKE_ESPTOOL_FAIL_FIRST    (default "0")         the first N non-version
                                                  invocations (by position in
                                                  the argv log) exit 1
FAKE_ESPTOOL_MAC           (default "aa:bb:cc:dd:ee:ff")   what `read-mac` reports

`version` invocations are never written to the argv log and are never made to
fail by FAKE_ESPTOOL_FAIL_FIRST — only FAKE_ESPTOOL_VERSION_RC controls that —
so a caller resolving the command (which always probes `version` first) does
not perturb an invocation count a test is asserting on.

Rejects the v4 subcommand/flag spellings (`read_mac`, `write_flash`,
`erase_flash`, `default_reset`, `hard_reset`) with an argparse-style usage
error, the way a real v5 esptool does.
"""

import json
import os
import re
import sys
from pathlib import Path

KNOWN_SUBCOMMANDS = ('read-mac', 'write-flash', 'erase-flash')
V4_SPELLINGS = ('read_mac', 'write_flash', 'erase_flash', 'default_reset', 'hard_reset')


def _version_lines(version):
    """Two output lines matching real esptool's `version` shape, or the raw
    value twice if it does not parse as a major.minor number."""
    m = re.match(r'(\d+)\.', version)
    if not m:
        return [version, version]
    major = int(m.group(1))
    tag = 'esptool.py' if major < 5 else 'esptool'
    return [f'{tag} v{version}', version]


def _log_and_index(argv):
    """Append this invocation to the argv log (if configured) and return its
    0-based position among prior entries, for FAIL_FIRST."""
    path = os.environ.get('FAKE_ESPTOOL_ARGV_LOG')
    if not path:
        return 0
    p = Path(path)
    index = sum(1 for _ in p.open()) if p.exists() else 0
    with p.open('a') as f:
        f.write(json.dumps({'argv': argv, 'cwd': os.getcwd()}) + '\n')
    return index


def _should_fail(index):
    n = int(os.environ.get('FAKE_ESPTOOL_FAIL_FIRST', '0') or '0')
    return index < n


def _port_of(argv):
    if '-p' in argv:
        i = argv.index('-p')
        if i + 1 < len(argv):
            return argv[i + 1]
    return '/dev/ttyACM0'


def _offset_file_pairs(argv):
    """(addr, path) pairs from the positional args following write-flash."""
    pairs = []
    i = 0
    while i < len(argv):
        tok = argv[i]
        if re.fullmatch(r'0x[0-9a-fA-F]+', tok) and i + 1 < len(argv):
            pairs.append((int(tok, 16), argv[i + 1]))
            i += 2
        else:
            i += 1
    return pairs


def main(argv):
    hit = [tok for tok in argv if tok in V4_SPELLINGS]
    if hit:
        print('usage: esptool [-h] ...', file=sys.stderr)
        print(f'esptool: error: unrecognized arguments (pre-v5 spelling): {hit}', file=sys.stderr)
        return 2

    if 'version' in argv and not any(sub in argv for sub in KNOWN_SUBCOMMANDS):
        for ln in _version_lines(os.environ.get('FAKE_ESPTOOL_VERSION', '5.4.0')):
            print(ln)
        return int(os.environ.get('FAKE_ESPTOOL_VERSION_RC', '0') or '0')

    index = _log_and_index(argv)

    if 'read-mac' in argv:
        if _should_fail(index):
            print(
                'A fatal error occurred: Failed to connect to ESP32-S3: Timed out waiting for packet header',
                file=sys.stderr,
            )
            return 1
        mac = os.environ.get('FAKE_ESPTOOL_MAC', 'aa:bb:cc:dd:ee:ff')
        print(f'Serial port {_port_of(argv)}')
        print('Connecting....')
        print('Chip is ESP32-S3')
        print(f'MAC: {mac}')
        return 0

    if 'erase-flash' in argv:
        if _should_fail(index):
            print('A fatal error occurred: erase failed', file=sys.stderr)
            return 1
        print('Erasing flash (this may take a while)...')
        print('Chip erase completed successfully.')
        return 0

    if 'write-flash' in argv:
        if _should_fail(index):
            print('A fatal error occurred: Timed out waiting for packet header', file=sys.stderr)
            return 1
        steps = 4
        for addr, path in _offset_file_pairs(argv):
            try:
                size = os.path.getsize(path)
            except OSError:
                size = 0
            for i in range(steps + 1):
                off = addr + (size * i) // steps if size else addr
                print(f'Writing at 0x{off:08x}...')
        print('Hash of data verified.')
        print('Hard resetting via RTS pin...')
        return 0

    print('usage: esptool [-h] ...', file=sys.stderr)
    print(f'esptool: error: unrecognized subcommand in {argv}', file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
