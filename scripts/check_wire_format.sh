#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
#
# Process guard for unannounced pstop_c wire breaks. No SR or DU is claimed:
# this checks coordinated release process, not runtime safety behavior.
set -uo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT" || exit 2

args=(check --root "$ROOT" --labels "${PSTOP_PR_LABELS:-}")
if [ -n "${PSTOP_BASE_SHA:-}" ]; then
    if [[ ! "$PSTOP_BASE_SHA" =~ ^[0-9A-Fa-f]{40}$ ]]; then
        echo "wire-format: cannot run: PSTOP_BASE_SHA must be exactly 40 ASCII hexadecimal characters" >&2
        exit 2
    fi
    if ! git cat-file -e "$PSTOP_BASE_SHA:tools/change_control/wire_format.sha256" 2>/dev/null; then
        args+=(--initial-expectation)
    fi
    if ! changed=$(git diff --name-only "$PSTOP_BASE_SHA"...HEAD 2>/dev/null); then
        echo "wire-format: cannot run: unable to compare PSTOP_BASE_SHA" >&2
        exit 2
    fi
    while IFS= read -r path; do
        [ -z "$path" ] || args+=(--changed-file "$path")
    done <<< "$changed"
fi
python3 -m tools.change_control.wire_format "${args[@]}"
