#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
#
# Build pstop_c with BullseyeCoverage and produce a covsrc summary or
# (with --html) a covhtml report at build/coverage-html/.
#
# Usage:
#   scripts/run-coverage.sh           # covsrc summary
#   scripts/run-coverage.sh --html    # HTML report

set -euo pipefail

mode="summary"
case "${1:-}" in
  ""|--summary) mode="summary" ;;
  --html)       mode="html" ;;
  -h|--help)
    sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
    ;;
  *)
    echo "run-coverage.sh: unknown argument: $1" >&2
    exit 2
    ;;
esac

if ! command -v cov01 >/dev/null 2>&1; then
  cat >&2 <<'EOF'
run-coverage.sh: cov01 not found on PATH.

BullseyeCoverage is required to produce coverage reports. Install it and
ensure /usr/local/BullseyeCoverage/bin (or the equivalent on your system)
is on PATH.
EOF
  exit 1
fi

# Locate pstop_c root from this script's location, regardless of CWD.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pstop_c_dir="$(cd "${script_dir}/.." && pwd)"
build_dir="${pstop_c_dir}/build"

# Restore cov01 state if a build failure aborts the CMake target mid-flight.
trap 'cov01 --off >/dev/null 2>&1 || true' EXIT

cmake -S "${pstop_c_dir}" -B "${build_dir}" -DPSTOP_ENABLE_COVERAGE=ON

case "${mode}" in
  summary) cmake --build "${build_dir}" --target coverage ;;
  html)    cmake --build "${build_dir}" --target coverage-html ;;
esac
