# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Parse the authoritative function decomposition."""

import re
from pathlib import Path

from .model import Function, LintError
from .parse_srs import split_row


def parse_system_definition(path):
    path = Path(path)
    if not path.is_file():
        raise LintError(f'required document missing: {path}')
    lines = path.read_text(encoding='utf-8').splitlines()
    in_section = False
    functions = {}
    for index, line in enumerate(lines):
        if line.startswith('## 4.'):
            in_section = True
        elif in_section and line.startswith('## '):
            break
        if not in_section or not line.lstrip().startswith('|'):
            continue
        cells = split_row(line)
        if len(cells) >= 2 and re.fullmatch(r'F-[A-Z]-\d{2}', cells[0].strip('*')):
            function_id = cells[0].strip('*')
            functions[function_id] = Function(function_id, cells[1], index + 1)
    if not functions:
        raise LintError(f'{path}: no functions found in section 4')
    return functions
