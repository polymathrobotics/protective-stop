#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
# Copy KiCad's stock symbol and footprint libraries out of the KiCad AppImage.
#
# The MCP server and build_symbols.py need real files on disk; an AppImage only
# exposes them while mounted. Usage: extract_kicad_libs.sh <KiCad.AppImage> [dest]
set -euo pipefail
appimage="${1:?usage: $0 <KiCad.AppImage> [dest]}"
dest="${2:-$HOME/Applications/kicad10-libs}"
log="$(mktemp)"
"$appimage" --appimage-mount >"$log" 2>&1 &
mount_pid=$!
trap 'kill "$mount_pid" 2>/dev/null || true; rm -f "$log"' EXIT
for _ in $(seq 1 20); do
  mnt="$(head -1 "$log" 2>/dev/null || true)"
  [ -d "$mnt/share/kicad/symbols" ] && break
  sleep 0.5
done
[ -d "$mnt/share/kicad/symbols" ] || { echo "could not mount $appimage" >&2; exit 1; }
rm -rf "$dest"
mkdir -p "$dest"
cp -r "$mnt/share/kicad/symbols" "$dest/symbols"
cp -r "$mnt/share/kicad/footprints" "$dest/footprints"
echo "libraries copied to $dest (set KICAD_SYMBOL_LIBS=$dest/symbols)"
