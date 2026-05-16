#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
#
# Configure + build pstop_c with BullseyeCoverage instrumentation, run the
# Unity test suite, and produce a covsrc summary (default) or an HTML report
# (--html). Used by devs locally and by .github/workflows/pstop_c_coverage.yml.
#
# Build artifacts land in pstop_c/build/ — the same dir the regular build
# uses, but the coverage targets force `cmake --build --clean-first` so a
# stale non-instrumented tree gets rebuilt safely.
#
# Usage:
#   scripts/run-coverage.sh           # covsrc summary
#   scripts/run-coverage.sh --html    # covsrc + HTML report at build/coverage-html/

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

# Belt-and-suspenders: the CMake target also runs `cov01 --off` on success,
# but a build failure mid-target would leave cov01 in the "on" state. This
# trap guarantees we restore the user's Bullseye state on any exit path.
trap 'cov01 --off >/dev/null 2>&1 || true' EXIT

cmake -S "${pstop_c_dir}" -B "${build_dir}" -DPSTOP_ENABLE_COVERAGE=ON

case "${mode}" in
  summary) cmake --build "${build_dir}" --target coverage ;;
  html)    cmake --build "${build_dir}" --target coverage-html ;;
esac
