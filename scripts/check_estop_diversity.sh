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
# shellcheck disable=SC2012,SC2015  # ls-glob for toolchain path; guard uses && || intentionally
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

# Core-0 selects the codeword through the k_estop_msg TABLE (data-indexed load).
# Core-1 forms it by a boolean that materialises the codewords DIRECTLY — with the
# old {0,1} codewords the compiler emitted a conditional-move (moveqz/movnez); with
# the far-Hamming v1 codewords (0x55/0x92) it emits a branch + `movi` of those
# immediates. Either way the diversity signature is "core-1 does NOT go through the
# table — it materialises the codeword constants itself". Derive the codeword values
# from the header so this survives any future codeword change; a genuine collapse
# (core-1 folded into core-0's table) has neither cmov nor the codeword immediates
# and still FAILS.
HDR="$(dirname "$0")/../pstop_c/pstop/include/pstop/pstop_msg.h"
okhex=$(grep -oE 'PSTOP_MESSAGE_OK[[:space:]]+0x[0-9A-Fa-f]+' "$HDR" 2>/dev/null | grep -oE '0x[0-9A-Fa-f]+' | head -1)
sthex=$(grep -oE 'PSTOP_MESSAGE_STOP[[:space:]]+0x[0-9A-Fa-f]+' "$HDR" 2>/dev/null | grep -oE '0x[0-9A-Fa-f]+' | head -1)
okd=$((${okhex:-0x55})); std=$((${sthex:-0x92}))

core0=$(printf '%s\n' "$DIS" | grep -cE 'k_estop_msg')            # core 0: table lookup
core1=$(printf '%s\n' "$DIS" | grep -cE "\b(moveqz|movnez)\b|movi(\.n)?[[:space:]]+a[0-9]+,[[:space:]]*(${okd}|${std})\b")  # core 1: cmov OR direct codeword immediates

echo "estop_decide Option-B diversity (object code):"
echo "  core-0 arithmetic-image + table-load (k_estop_msg refs): $core0"
echo "  core-1 boolean cmov / direct codeword immediates ($okd,$std): $core1"

if [ "$core0" -ge 1 ] && [ "$core1" -ge 1 ]; then
  echo "PASS: both diverse verdict paths present — Option B intact at -O2."
  exit 0
fi
echo "FAIL: a diverse verdict path is MISSING — the optimizer may have collapsed"
echo "      Option B (DU-3). Add a compiler barrier / opaque per-core seed."
exit 1
