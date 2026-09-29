#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
#
# Export the schematic to relay-board-schematic.pdf, the human-readable copy that
# is committed next to it for reviewers. Re-run after ANY change to the .kicad_sch
# and commit both together.
# Usage: export_pdf.sh [path-to-kicad-cli]      (default: kicad-cli on PATH)
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
kicad_cli="${1:-kicad-cli}"
"$kicad_cli" sch export pdf -o "$here/relay-board-schematic.pdf" "$here/relay-board.kicad_sch"
echo "wrote $here/relay-board-schematic.pdf"
