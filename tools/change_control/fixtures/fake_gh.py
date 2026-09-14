#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Deterministic gh replacement used only by change-control tests."""

import json
import os
import sys
from pathlib import Path


def main():
    """Serve API responses from the JSON file named by FAKE_GH_DATA."""
    data = json.loads(Path(os.environ['FAKE_GH_DATA']).read_text(encoding='utf-8'))
    if data.get('exit_code'):
        print(data.get('stderr', 'fake gh failure'), file=sys.stderr)
        return int(data['exit_code'])
    args = sys.argv[1:]
    if not args or args[0] != 'api':
        return 2
    endpoint = args[1] if len(args) > 1 else ''
    method = 'GET'
    if '--method' in args:
        method = args[args.index('--method') + 1]
    key = f'{method} {endpoint}'
    calls = os.environ.get('FAKE_GH_CALLS')
    if calls:
        with Path(calls).open('a', encoding='utf-8') as stream:
            stream.write(key + '\n')
    response = data.get('responses', {}).get(key)
    if response is None:
        print(f'unconfigured fake gh request: {key}', file=sys.stderr)
        return 1
    if isinstance(response, dict) and '__pages__' in response:
        response = response['__pages__']
    elif '--slurp' in args:
        response = [response]
    if isinstance(response, str):
        print(response)
    else:
        print(json.dumps(response))
    return 0


if __name__ == '__main__':
    sys.exit(main())
