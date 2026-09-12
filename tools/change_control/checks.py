# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Evaluate existence and ordering of modification records without judging adequacy."""

import re
from dataclasses import dataclass
from pathlib import Path

from tools.safety_lint.parse_srs import parse_srs

from .issue_form import parse_issue_form

AUTHORIZERS = ('iliabaranov', 'rajasimman-madhivanan', 'davidt315')
CLASS_RANK = {'A': 1, 'B': 2, 'C': 3}
COMMENT_MARKER = '<!-- change-control -->'


@dataclass(frozen=True)
class CheckResult:
    """One caller-visible policy result."""

    check_id: str
    status: str
    message: str


def load_mode(root):
    """Read the auditable mode switch and reject every value except warn or enforce."""
    path = Path(root) / 'docs/process/enforcement-mode'
    try:
        value = path.read_text(encoding='utf-8')
    except OSError as error:
        raise RuntimeError(f'cannot read enforcement mode: {error}') from error
    if value not in ('warn\n', 'enforce\n'):
        raise RuntimeError('enforcement-mode must contain exactly warn or enforce followed by a newline')
    return value.strip()


def _headings(path):
    return [
        line.strip() for line in Path(path).read_text(encoding='utf-8').splitlines() if re.match(r'^#(?:\s|\d)', line)
    ]


def impact_analysis_complete(text, headings):
    """Check that headings occur in order and each has non-whitespace content beneath it."""
    positions = []
    cursor = 0
    for heading in headings:
        match = re.search(rf'(?m)^{re.escape(heading)}\s*$', text[cursor:])
        if not match:
            return False, [heading]
        start = cursor + match.start()
        end = cursor + match.end()
        positions.append((heading, start, end))
        cursor = end
    missing = []
    for index, (heading, _, end) in enumerate(positions):
        next_start = positions[index + 1][1] if index + 1 < len(positions) else len(text)
        if not text[end:next_start].strip():
            missing.append(heading)
    return not missing, missing


def minimum_class(paths, wire_changed=False):
    """Return the mechanical classification floor for changed repository paths."""
    floor = 'C' if wire_changed else 'A'
    for path in paths:
        if (
            path.startswith('pstop_c/')
            or path in ('firmware/main/main.c', 'machn/main/main.c')
            or path.startswith('docs/safety/')
        ):
            candidate = 'C'
        elif path.startswith(('components/', 'common/')) or path in (
            'firmware/sdkconfig.defaults',
            'machn/sdkconfig.defaults',
        ):
            candidate = 'B'
        elif path.startswith('.github/workflows/') or path.startswith('scripts/'):
            candidate = 'B'
        else:
            candidate = 'A'
        if CLASS_RANK[candidate] > CLASS_RANK[floor]:
            floor = candidate
    return floor


def _labels(entity):
    return {label['name'] if isinstance(label, dict) else label for label in entity.get('labels', [])}


def _cr_number(body):
    matches = re.findall(
        r'(?im)^\s*(?:closes|refs)\s+(?:(?:https://github\.com/[^/]+/[^/]+/issues/)?#?)(\d+)\s*$',
        body or '',
    )
    return int(matches[0]) if len(set(matches)) == 1 else None


def _issue_fields_complete(root, body):
    form = parse_issue_form(Path(root) / '.github/ISSUE_TEMPLATE/change-request.yml')
    missing = []
    for field in form['body']:
        if not field['required']:
            continue
        label = field['label']
        match = re.search(rf'(?ms)^###\s+{re.escape(label)}\s*$\n(.*?)(?=^###\s|\Z)', body or '')
        if not match or not match.group(1).strip() or match.group(1).strip() == '_No response_':
            missing.append(field['id'])
    return missing


def _find_ia(comments, headings):
    for comment in comments:
        body = comment.get('body', '')
        if headings and headings[0] in body:
            return body
    return ''


def _find_short_ia(comments):
    for comment in comments:
        body = comment.get('body', '')
        if all(re.search(phrase, body, re.IGNORECASE) for phrase in ('what changed', 'what it could affect', 'tests')):
            return body
    return ''


def _cited_sr_tokens(text):
    return set(re.findall(r'\bSR-[A-Z]+-[0-9A-Za-z]+(?:-[0-9A-Za-z]+)*\b', text))


def _is_authorization(comment, author=''):
    login = comment.get('user', {}).get('login', '').lower()
    return (
        login in AUTHORIZERS
        and login != author.lower()
        and bool(re.search(r'(?im)^\s*(?:decision:\s*)?authori[sz]ed\b', comment.get('body', '')))
    )


def _named_tests(text):
    sections = re.findall(r'(?ms)^# 7\. Verification plan for this change\s*$\n(.*?)(?=^# 8\.|\Z)', text)
    names = []
    for section in sections:
        names.extend(re.findall(r'`([^`]+)`', section))
        for line in section.splitlines():
            if line.lstrip().startswith(('-', '+')) and ':' in line:
                value = line.split(':', 1)[1].strip()
                if value and value.lower() not in ('none', 'n/a'):
                    names.append(value)
            if line.strip().startswith('|'):
                cells = [cell.strip() for cell in line.strip().strip('|').split('|')]
                if len(cells) >= 2 and cells[0] not in ('Purpose', '---'):
                    value = cells[1]
                    if value and value not in ('---', 'None', 'N/A'):
                        names.extend(part.strip() for part in re.split(r'<br\s*/?>|,', value) if part.strip())
    if not sections:
        match = re.search(r'(?im)^\s*(?:which\s+)?tests(?:\s+will\s+be\s+run)?\s*:\s*(.+)$', text)
        if match:
            names.extend(part.strip(' `') for part in match.group(1).split(',') if part.strip(' `'))
    return list(dict.fromkeys(name.strip() for name in names if name.strip()))


def _evidence_matches(name, evidence_name):
    """Match an IA entry to one complete check or workflow name, never a substring."""
    planned = ' '.join(name.casefold().split())
    observed = ' '.join(evidence_name.casefold().split())
    return bool(planned and observed and planned == observed)


def _approvers(data, require_head=True):
    head = data['pr']['head']['sha']
    author = data['pr']['user']['login'].lower()
    latest = {}
    for review in data.get('reviews', []):
        login = review.get('user', {}).get('login', '').lower()
        if login:
            latest[login] = review
    return {
        login
        for login, review in latest.items()
        if login in AUTHORIZERS
        and login != author
        and review.get('state') == 'APPROVED'
        and (not require_head or review.get('commit_id') == head)
    }


def evaluate(root, data):
    """Evaluate E1-E7 against a complete, synthetic-or-live GitHub state snapshot."""
    root = Path(root)
    pr = data['pr']
    labels = _labels(pr)
    cr_number = _cr_number(pr.get('body', ''))
    issue = data.get('issue', {})
    issue_labels = _labels(issue)
    comments = data.get('issue_comments', [])
    headings = _headings(root / 'docs/process/templates/IMPACT_ANALYSIS.md')
    ia = _find_ia(comments, headings)
    short_ia = _find_short_ia(comments)
    emergency = 'emergency' in labels
    if emergency and not ia:
        ia = short_ia
    results = []

    if 'needs-change-request' in labels:
        results.append(CheckResult('E1', 'pending', 'maintainer Change Request required before review begins'))
    else:
        author = pr['user']['login']
        required_authorizers = 2 if 'class-c' in labels or emergency else 1
        authorization_comments = {}
        for comment in comments:
            if _is_authorization(comment, author):
                authorization_comments[comment['user']['login'].lower()] = comment
        authorized_comment = len(authorization_comments) >= required_authorizers
        authorization_times = [
            comment.get('created_at') for comment in authorization_comments.values() if comment.get('created_at')
        ]
        ia_times = [
            comment.get('created_at')
            for comment in comments
            if comment.get('body', '') == ia and comment.get('created_at')
        ]
        before_implementation = not pr.get('created_at') or (
            len(authorization_times) >= required_authorizers and max(authorization_times) <= pr['created_at']
        )
        after_analysis = not ia_times or (
            len(authorization_times) >= required_authorizers and max(ia_times) <= min(authorization_times)
        )
        ordered = before_implementation and after_analysis
        missing_fields = _issue_fields_complete(root, issue.get('body', '')) if cr_number else ['change-request-link']
        okay = (
            cr_number is not None
            and 'change-request' in issue_labels
            and 'status:authorized' in issue_labels
            and authorized_comment
            and ordered
            and not missing_fields
        )
        detail = (
            f'authorized Change Request has {required_authorizers} distinct pre-implementation authorizer(s)'
            if okay
            else f'Change Request missing, ambiguous, incomplete, or lacks {required_authorizers} distinct pre-implementation authorizer(s) ({", ".join(missing_fields)})'
        )
        results.append(CheckResult('E1', 'pass' if okay else 'fail', detail))

    if emergency and ia == short_ia and short_ia:
        complete, empty = True, []
    else:
        complete, empty = impact_analysis_complete(ia, headings) if ia else (False, ['Impact Analysis'])
    results.append(
        CheckResult(
            'E2',
            'pass' if complete else 'fail',
            'all IA sections exist and are nonblank; content truth and adequacy are not assessed'
            if complete
            else f'IA sections missing or blank: {", ".join(empty)}; content truth and adequacy are not assessed',
        )
    )

    canonical = {requirement.sr_id for requirement in parse_srs(root / 'docs/safety/SAFETY_REQUIREMENTS.md')}
    cited = _cited_sr_tokens(ia)
    invalid = sorted(cited - canonical)
    results.append(
        CheckResult(
            'E3',
            'fail' if invalid else 'pass',
            f'invalid requirement IDs: {", ".join(invalid)}' if invalid else 'all cited requirement IDs exist',
        )
    )

    class_labels = sorted(label for label in labels if re.fullmatch(r'class-[abc]', label))
    paths = [item['filename'] for item in data.get('files', [])]
    wire_changed = any(path.startswith('pstop_c/pstop/include/pstop/') for path in paths)
    floor = minimum_class(paths, wire_changed)
    if len(class_labels) != 1:
        results.append(CheckResult('E4', 'fail', 'exactly one class-a, class-b, or class-c label is required'))
        selected = None
    else:
        selected = class_labels[0][-1].upper()
        under = CLASS_RANK[selected] < CLASS_RANK[floor]
        results.append(
            CheckResult(
                'E4',
                'fail' if under else 'pass',
                f'Class {selected}; mechanical floor Class {floor}; checks existence/order, not classification adequacy',
            )
        )

    approvers = _approvers(data)
    if selected == 'C':
        status = 'pass' if len(approvers) >= 2 else 'fail'
        results.append(
            CheckResult('E5', status, f'current distinct non-author approving authorizers: {len(approvers)}/2')
        )
    else:
        results.append(CheckResult('E5', 'not-applicable', 'two-review requirement applies to Class C'))

    names = _named_tests(ia)
    head = pr['head']['sha']
    evidence = {
        item.get('name', '')
        for item in data.get('check_runs', [])
        if item.get('head_sha', head) == head and item.get('conclusion') == 'success'
    }
    evidence.update(
        item.get('name', item.get('path', ''))
        for item in data.get('workflow_runs', [])
        if item.get('head_sha') == head and item.get('conclusion') == 'success'
    )
    missing_tests = [name for name in names if not any(_evidence_matches(name, item) for item in evidence)]
    explanation = 'check-run/workflow evidence cannot prove commands or tests inside a job executed'
    if not names:
        e6_status = 'fail'
        e6_message = f'IA verification plan names no specific tests; {explanation}'
    elif missing_tests:
        e6_status = 'fail'
        e6_message = f'missing head-SHA evidence: {", ".join(missing_tests)}; {explanation}'
    else:
        e6_status = 'pass'
        e6_message = f'all named evidence matched; {explanation}'
    results.append(
        CheckResult(
            'E6',
            e6_status,
            e6_message,
        )
    )

    if emergency:
        short_form = bool(ia and names and short_ia)
        status = 'pass' if len(approvers) >= 2 and short_form else 'fail'
        results.append(
            CheckResult(
                'E7',
                status,
                'emergency path requires two approvals and short-form IA; retrospective due within five working days of release',
            )
        )
    else:
        results.append(CheckResult('E7', 'not-applicable', 'PR is not labelled emergency'))
    return results


def render_report(mode, results):
    """Render one deterministic PR comment with the active mode visible."""
    lines = [f'mode: {mode}', '', '| Check | Result | Explanation |', '|---|---|---|']
    for item in results:
        message = item.message.replace('|', '\\|')
        lines.append(f'| {item.check_id} | {item.status} | {message} |')
    lines.extend([
        '',
        'These checks verify artifact existence and ordering only, not truth, adequacy, or safety sufficiency.',
    ])
    return '\n'.join(lines)


def upsert_comment(api, pr_number, report, comments):
    """Create or update at most one marker-owned report comment."""
    body = f'{COMMENT_MARKER}\n{report}'
    existing = next((comment for comment in comments if COMMENT_MARKER in comment.get('body', '')), None)
    if existing:
        api('PATCH', f'repos/{{repo}}/issues/comments/{existing["id"]}', {'body': body})
    else:
        api('POST', f'repos/{{repo}}/issues/{pr_number}/comments', {'body': body})
