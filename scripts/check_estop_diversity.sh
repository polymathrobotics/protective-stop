#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
#
# SR-R-03 / FMEA DU-3 — verify Option B verdict DIVERSITY survives the optimizer.
#
# estop_decide() forms the OK/STOP verdict by two INDEPENDENT expressions
# selected on core_id: core 0 by arithmetic image + table lookup (k_estop_msg),
# core 1 by a branchless boolean (moveqz/movnez). They are logically identical,
# so an optimizer *could* collapse them to one implementation — which would make
# the dual-core diversity illusory (common-mode β≈1). This check disassembles
# the shipped function and asserts BOTH distinct implementations are still
# present in the object code. If either vanishes, the optimizer collapsed the
# diversity and this FAILS.
#
# Usage: scripts/check_estop_diversity.sh [path/to/pstop_remote.elf]
#        (source esp-idf export.sh first, or have the xtensa objdump on PATH)
set -uo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
ELF="${1:-$ROOT/firmware/build/pstop_remote.elf}"
OD=$(command -v xtensa-esp32s3-elf-objdump 2>/dev/null || \
     ls ~/.espressif/tools/xtensa-esp-elf/*/xtensa-esp-elf/bin/xtensa-esp32s3-elf-objdump 2>/dev/null | head -1)

[ -n "$OD" ] && [ -x "$OD" ] || { echo "check_estop_diversity: xtensa objdump not found (source esp-idf export.sh)"; exit 2; }
[ -f "$ELF" ] || { echo "check_estop_diversity: ELF not found: $ELF (build firmware first)"; exit 2; }

# Extract just the estop_decide function body.
DIS=$("$OD" -d "$ELF" | awk '/<estop_decide>:/{f=1} f{print} /^$/{if(f)c++} c>1{exit}')
[ -n "$DIS" ] || { echo "check_estop_diversity: estop_decide symbol not found (inlined? LTO?)"; exit 1; }

core0=$(printf '%s\n' "$DIS" | grep -cE 'k_estop_msg')            # core 0: table lookup
core1=$(printf '%s\n' "$DIS" | grep -cE '\b(moveqz|movnez)\b')    # core 1: boolean cmov

echo "estop_decide Option-B diversity (object code):"
echo "  core-0 arithmetic-image + table-load (k_estop_msg refs): $core0"
echo "  core-1 boolean conditional-move (moveqz/movnez):         $core1"

if [ "$core0" -ge 1 ] && [ "$core1" -ge 1 ]; then
  echo "PASS: both diverse verdict paths present — Option B intact at -O2."
  exit 0
fi
echo "FAIL: a diverse verdict path is MISSING — the optimizer may have collapsed"
echo "      Option B (DU-3). Add a compiler barrier / opaque per-core seed."
exit 1
