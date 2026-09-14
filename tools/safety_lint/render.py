# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Render named numeric regions without touching safety-argument prose."""

import re

from .model import LintError


def _percent(numerator, denominator):
    if denominator == 0:
        return '0 %'
    value = 100 * numerator / denominator
    return '100 %' if value == 100 else f'{value:.1f} %'


def _replace(text, name, content):
    pattern = re.compile(
        rf'(<!-- BEGIN GENERATED: safety-lint {re.escape(name)} -->\n).*?(\n<!-- END GENERATED: safety-lint {re.escape(name)} -->)',
        re.DOTALL,
    )
    if len(pattern.findall(text)) != 1:
        raise LintError(f'TRACEABILITY.md requires exactly one generated marker pair named {name}')
    return pattern.sub(lambda match: match.group(1) + content + match.group(2), text)


def render_traceability(text, coverage):
    headline = (
        f'- **SRs with at least one cited verifying test: {coverage.cited_tests} / {coverage.total} = '
        f'{_percent(coverage.cited_tests, coverage.total)}**‡\n'
        f'- **Strict, fully-verified only: {coverage.verified} / {coverage.total} = '
        f'{_percent(coverage.verified, coverage.total)}**\n'
        f'- **Functions traced to at least one SR: {coverage.functions_traced} / {coverage.functions_total} = '
        f'{_percent(coverage.functions_traced, coverage.functions_total)}**\n'
        f'- **Functions traced excluding declared non-safety functions: {coverage.safety_functions_traced} / '
        f'{coverage.safety_functions_total} = {_percent(coverage.safety_functions_traced, coverage.safety_functions_total)}**\n'
        '\n'
        '‡ Citation resolution, not test execution or passing state, is checked by the linter.'
    )
    lines = [
        '| Area | Count | Verified | Partially-verified | Unverified-gap | Residual-accepted | ≥1 cited test % | Fully-verified % |',
        '|---|---|---|---|---|---|---|---|',
    ]
    for area in ('SYS', 'R', 'H', 'M', 'I'):
        data = coverage.areas[area]
        cited_percent = _percent(data['cited'], data['count'])
        if area == 'M':
            cited_percent += '†'
        lines.append(
            f'| SR-{area} | {data["count"]} | {data["Verified"]} | {data["Partially-verified"]} | '
            f'{data["Unverified-gap"]} | {data["Residual-accepted"]} | {cited_percent} | '
            f'{_percent(data["Verified"], data["count"])} |'
        )
    lines.append(
        f'| **Total** | **{coverage.total}** | **{sum(v["Verified"] for v in coverage.areas.values())}** | '
        f'**{sum(v["Partially-verified"] for v in coverage.areas.values())}** | '
        f'**{sum(v["Unverified-gap"] for v in coverage.areas.values())}** | '
        f'**{sum(v["Residual-accepted"] for v in coverage.areas.values())}** | '
        f'**{_percent(coverage.cited_tests, coverage.total)}** | **{_percent(coverage.verified, coverage.total)}** |'
    )
    return _replace(_replace(text, 'headline', headline), 'areas', '\n'.join(lines))
