#!/bin/bash
# MISRA C:2012 check for the pstop firmware we own, using the free cppcheck
# misra addon. Covers firmware/main + firmware/components/dcs_support.
# EXCLUDES components/pstop/upstream (certified pstop_c, checked upstream) and
# ESP-IDF/managed components (third-party platform).
#
# This is the engineering pre-check; formal certification evidence needs a
# licensed MISRA checker (rule texts + compliance report). See
# docs/MISRA_COMPLIANCE_2026-07-21.md for the deviation register.
#
# Install (Ubuntu):  sudo apt-get install -y cppcheck
# Run:               ./tools/misra_check.sh [main|dcs|all]   (default: all)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FW="$ROOT/firmware"
SEL="${1:-all}"

if ! command -v cppcheck >/dev/null; then
  echo "cppcheck not found — install with: sudo apt-get install -y cppcheck"; exit 1
fi

INC="-I $FW/main -I $FW/components/dcs_support/include -I $FW/components/dcs_support/src"
COMMON="--addon=misra --std=c11 --quiet --enable=warning $INC"

run_one() {  # $1 = file
  echo "=== MISRA: $1 ==="
  cppcheck $COMMON "$1" 2>&1 | grep -E "misra-c2012" | \
    sed -E 's#.*/([^/]+):([0-9]+):[0-9]+: style: misra violation.*\[misra-(c2012-[0-9.]+)\]#\1:\2 \3#' | \
    sort -t' ' -k2 | uniq -c | sort -rn
}

case "$SEL" in
  main) run_one "$FW/main/main.c" ;;
  dcs)  for f in "$FW"/components/dcs_support/src/*.c; do run_one "$f"; done ;;
  all)  run_one "$FW/main/main.c"
        for f in "$FW"/components/dcs_support/src/*.c; do run_one "$f"; done ;;
  *) echo "usage: $0 [main|dcs|all]"; exit 2 ;;
esac

echo
echo "NOTE: cppcheck 2.7's misra addon crashes on dcs_net_liveness.c (its"
echo "rule-7.4 checker has a NoneType bug); that file gets a manual pass."
echo "Residual findings are the documented deviation register — see"
echo "docs/MISRA_COMPLIANCE_2026-07-21.md."
