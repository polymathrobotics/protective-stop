<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# DU-3 / SR-R-03 — Option B diversity survives the optimizer (object-code check)

**Status: RESOLVED (2026-08-02).** Diversity confirmed real at the shipped
object-code level; a repeatable CI guard now protects it.

## The concern
`estop_decide()` (firmware/main/estop_verdict.c) forms the OK/STOP verdict by
two **independent expressions** selected on `core_id`:
- core 0 — arithmetic image + table lookup: `k_estop_msg[((rb_hi^1)|rb_lo)&1]`
- core 1 — branchless boolean: `(rb_hi==1 && rb_lo==0) ? OK : STOP`

They are logically identical, so a sufficiently smart optimizer could **collapse
them to a single implementation**. If it did, both lockstep cores would execute
the *same* machine code for the verdict and the dual-core diversity against a
common-mode interpretation fault would be **illusory** (β≈1). FMEA flagged this
as **DU-3** and the requirements set as the **SR-R-03 / SC 3** gate.

## The check + result
The firmware ships at `-O2` (`CONFIG_COMPILER_OPTIMIZATION_PERF`, xtensa
gcc 14.2). Disassembling `estop_decide` from `firmware/build/pstop_remote.elf`:

- A **real runtime branch on `core_id`** (`bnez.n a3, …`) selects the path.
- **Core 0 path:** `xor` / `or` / `extui` / `l32r …k_estop_msg` / `l8ui` —
  the arithmetic-image computation + **table load**.
- **Core 1 path:** `moveqz` / `movnez` — a **branchless conditional-move**
  boolean, no table.
- The debounce / STOP-override / priming code *after* the selection is shared —
  by design; the diversity claim is only over the verdict *selection*.

The two selection paths are **distinct instruction sequences** — the optimizer
did **not** collapse them. Option B diversity is real at the object level.

## Standing guard (so a future toolchain change can't silently collapse it)
`scripts/check_estop_diversity.sh` disassembles `estop_decide` and asserts BOTH
distinct implementations are still present (a `k_estop_msg` table load AND
`moveqz`/`movnez`). It **fails** if either vanishes. Wired into CI in
`.github/workflows/firmware-build.yml` (runs inside the ESP-IDF action, where the
xtensa objdump is on PATH), so a compiler/flag change that merges the paths
breaks the build rather than silently degrading the safety claim.

## Residual
This proves *object-code* non-identity, which is the SC 3 concern for a
common-mode *systematic* fault in the verdict expression. It does not add
hardware/ISA diversity (still one toolchain, two identical Xtensa cores) — that
remains a documented β input in the FMEDA (β for shared silicon is the
least-defensible term). Downstream shared code (encode / `memcmp` / priming) is
covered by DU-5/DU-7, not by this check.
