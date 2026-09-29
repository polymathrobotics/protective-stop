#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
#
# Process configuration helper; it guards no SR or DU and changes labels only
# when explicitly run by an authorized repository administrator.
set -uo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT" || exit 2
REPOSITORY="${PSTOP_REPOSITORY:-polymathrobotics/protective-stop}"

if [ "$#" -gt 1 ] || { [ "$#" -eq 1 ] && [ "$1" != "--dry-run" ]; }; then
    echo "usage: scripts/sync_labels.sh [--dry-run]" >&2
    exit 2
fi
dry_run=false
[ "${1:-}" = "--dry-run" ] && dry_run=true
if [ "$dry_run" = false ]; then
    command -v gh >/dev/null 2>&1 || { echo "sync-labels: gh not found" >&2; exit 2; }
fi

python3 - "$ROOT/tools/change_control/labels.json" <<'PY' | while IFS=$'\t' read -r name color description; do
import json
import sys

for label in json.load(open(sys.argv[1], encoding='utf-8')):
    print(label['name'], label['color'], label['description'], sep='\t')
PY
    if [ "$dry_run" = true ]; then
        printf 'would sync label: %s (%s)\n' "$name" "$color"
    else
        gh label create "$name" --repo "$REPOSITORY" --color "$color" --description "$description" --force || exit 1
    fi
done
