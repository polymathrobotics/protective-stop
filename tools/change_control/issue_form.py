# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Parse and validate the constrained YAML used by the Change Request issue form."""

import re
from pathlib import Path


def _scalar(value):
    value = value.strip()
    if not value:
        return ''
    if value in ('true', 'false'):
        return value == 'true'
    if value.isdigit():
        return int(value)
    if value.startswith('[') and value.endswith(']'):
        return [part.strip().strip('\'"') for part in value[1:-1].split(',') if part.strip()]
    return value.strip('\'"')


def parse_issue_form(path):
    """Return issue-form controls from the repository's deliberately limited YAML subset."""
    lines = Path(path).read_text(encoding='utf-8').splitlines()
    if any('\t' in line for line in lines):
        raise ValueError('tabs are not valid indentation')
    document = {'body': []}
    item = None
    section = None
    options = False
    in_body = False
    for number, raw in enumerate(lines, 1):
        if not raw.strip() or raw.lstrip().startswith('#') or raw.strip() == '---':
            continue
        indent = len(raw) - len(raw.lstrip(' '))
        text = raw.strip()
        if indent == 0:
            match = re.fullmatch(r'([a-z_]+):(?:\s*(.*))?', text)
            if not match:
                raise ValueError(f'{path}:{number}: unsupported top-level YAML')
            key, value = match.groups()
            if key == 'body':
                if value:
                    raise ValueError(f'{path}:{number}: body must be a sequence')
                in_body = True
            else:
                if in_body:
                    raise ValueError(f'{path}:{number}: top-level key after body')
                document[key] = _scalar(value or '')
            continue
        if not in_body:
            raise ValueError(f'{path}:{number}: unexpected indentation')
        if indent == 2 and text.startswith('- type: '):
            item = {'type': _scalar(text[8:]), 'required': False, 'options': []}
            document['body'].append(item)
            section = None
            options = False
            continue
        if item is None:
            raise ValueError(f'{path}:{number}: body entry must begin with type')
        if indent == 4 and re.fullmatch(r'(id|attributes|validations):.*', text):
            key, value = text.split(':', 1)
            if key == 'id':
                item['id'] = _scalar(value)
                section = None
            else:
                if value.strip():
                    raise ValueError(f'{path}:{number}: {key} must be a mapping')
                section = key
            options = False
            continue
        if indent == 6 and section in ('attributes', 'validations'):
            if ':' not in text:
                raise ValueError(f'{path}:{number}: expected mapping value')
            key, value = text.split(':', 1)
            if section == 'validations' and key == 'required':
                item['required'] = _scalar(value)
            elif section == 'attributes' and key == 'options':
                if value.strip():
                    raise ValueError(f'{path}:{number}: options must be a sequence')
                options = True
            elif section == 'attributes':
                item[key] = _scalar(value)
                options = False
            else:
                raise ValueError(f'{path}:{number}: unsupported validation')
            continue
        if indent == 8 and options and text.startswith('- '):
            item['options'].append(_scalar(text[2:]))
            continue
        if indent >= 8 and section == 'attributes':
            continue
        raise ValueError(f'{path}:{number}: unsupported indentation or YAML construct')
    validate_issue_form(document)
    return document


def validate_issue_form(document):
    """Reject forms that GitHub could silently replace with a blank issue page."""
    for key in ('name', 'description', 'title', 'labels'):
        if not document.get(key):
            raise ValueError(f'issue form missing top-level {key}')
    body = document.get('body')
    if not isinstance(body, list) or not body:
        raise ValueError('issue form body must contain controls')
    ids = []
    for field in body:
        if field.get('type') not in ('input', 'textarea', 'dropdown'):
            raise ValueError(f'unsupported issue form field type: {field.get("type")}')
        if not field.get('id') or not field.get('label'):
            raise ValueError('every issue form field needs id and label')
        ids.append(field['id'])
        if field['type'] == 'dropdown':
            if not field.get('options') or not isinstance(field.get('default'), int):
                raise ValueError('dropdown needs options and integer default')
            if not 0 <= field['default'] < len(field['options']):
                raise ValueError('dropdown default is outside options')
    if len(ids) != len(set(ids)):
        raise ValueError('duplicate issue form field id')
    return document
