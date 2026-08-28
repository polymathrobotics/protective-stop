#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
#
# Build pstop_c once with BullseyeCoverage, then measure each test suite against
# that build. Prints a per-file table per suite and writes it to
# build/coverage-summary-<suite>.txt.
#
# Usage:
#   scripts/run-coverage.sh                        # both suites, summary
#   scripts/run-coverage.sh --html                 # both suites, HTML reports
#   scripts/run-coverage.sh --suite requirements   # one suite
#
# Suites:
#   unit          pstop_test               build/coverage-html-unit/
#   requirements  pstop_requirements_test  build/coverage-html-requirements/

set -euo pipefail

mode="summary"
suites="unit requirements"
target="coverage"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --summary) mode="summary"; shift ;;
    --html)    mode="html";    shift ;;
    --suite)
      case "${2:-}" in
        unit|requirements) suites="$2"; target="coverage-$2"; shift 2 ;;
        both)              suites="unit requirements"; target="coverage"; shift 2 ;;
        *)
          echo "run-coverage.sh: --suite needs unit, requirements, or both" >&2
          exit 2
          ;;
      esac
      ;;
    -h|--help) sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *)
      echo "run-coverage.sh: unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

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

[[ "${mode}" == "html" ]] && target="${target}-html"
cmake --build "${build_dir}" --target "${target}"

# Per-file table, for the reader and for the CI job summary.
for suite in ${suites}; do
  case "${suite}" in
    unit)         covfile="pstop_unit.cov" ;;
    requirements) covfile="pstop_requirements.cov" ;;
  esac
  echo
  echo "== ${suite} =="
  (
    cd "${build_dir}"
    COVFILE="${covfile}" covsrc \
      "${pstop_c_dir}"/pstop/src/pstop/*.c \
      "${pstop_c_dir}"/transport/src/transport/udp/*.c
  ) | sed "s|${pstop_c_dir}/||" | tee "${build_dir}/coverage-summary-${suite}.txt"
done
