#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
#
# Build one app from two copies of the tree at different paths and require
# byte-identical flashable outputs and ELF.
#
# Usage: scripts/check_reproducible.sh firmware|machn [extra idf.py args...]
# Needs a sourced ESP-IDF. KEEP_WORK=1 keeps both trees for diffing.
# Exit: 0 = identical, 1 = outputs differ or a build failed.
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
APP=${1:?usage: check_reproducible.sh firmware|machn [idf.py args...]}
shift

WORK=$(mktemp -d)
if [ -n "${KEEP_WORK:-}" ]; then
  echo "work dir: $WORK"
else
  trap 'rm -rf "$WORK"' EXIT
fi

for copy in a bb; do
  mkdir -p "$WORK/$copy"
  tar -C "$ROOT" -c --exclude='./*/build' --exclude='./*/build-*' --exclude='./*/sdkconfig' \
    --exclude='./*/sdkconfig.old' --exclude='./*/sdkconfig.credentials' --exclude='./tools/.venv' . |
    tar -C "$WORK/$copy" -x
  (cd "$WORK/$copy/$APP" && idf.py -DPROJECT_VER=reproducible-check "$@" build >"$WORK/$copy.log" 2>&1) || {
    tail -30 "$WORK/$copy.log"
    exit 1
  }
done

# flasher_args.json omits the bootloader in Secure Boot builds.
outputs() {
  (cd "$1" && {
    python3 -c 'import json; print("\n".join(json.load(open("flasher_args.json"))["flash_files"].values()))'
    ls ./*.elf bootloader/bootloader.bin bootloader/bootloader.elf
  } | sed 's#^\./##' | sort -u)
}

status=0
while IFS= read -r f; do
  a=$(sha256sum "$WORK/a/$APP/build/$f" | cut -d' ' -f1)
  b=$(sha256sum "$WORK/bb/$APP/build/$f" | cut -d' ' -f1)
  if [ "$a" = "$b" ]; then
    echo "same    $a  $f"
  else
    echo "DIFFERS $f"
    status=1
  fi
done < <(outputs "$WORK/a/$APP/build")
exit $status
