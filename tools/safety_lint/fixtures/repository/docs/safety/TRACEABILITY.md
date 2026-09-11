<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Traceability

Test-file shorthand: `EV` = `tests/ev.c`; `MR` = `tools/pstop_multi_remote_test.py`; `HIL10` = `tools/hil/test_10_button.py`; `HIL20` = `tools/hil/test_20_discordance.py`; `HIL30` = `tools/hil/test_30_power_cycle.py`; `JL` = `tests/json_lite.cpp`; `REQ n_nn` = `pstop_c/pstop/test/src/pstop/requirements/req_n_nn_test.c`.

| SR | Alloc F-xx | Code (file:line) | Verifying test(s) | Method | Status |
|---|---|---|---|---|---|
| SR-SYS-01 | F-R-01 | code.c:1 | MR[B/D/E/F], REQ 2_02/2_03, **NO TEST** for latency | Test | **Partially-verified** |
| SR-R-01 | F-R-01 | code.c:1 | test_unique_probe.py | Test | **Verified** |

<!-- BEGIN GENERATED: safety-lint headline -->
stale
<!-- END GENERATED: safety-lint headline -->
<!-- BEGIN GENERATED: safety-lint areas -->
stale
<!-- END GENERATED: safety-lint areas -->

- **(a) SRs with ≥1 passing verifying test: 2 / 2 = 100 %**
- **Strict, fully-verified only: 1 / 2 = 50.0 %.**
- **(b) Safety functions F-xx traced to ≥1 SR: 1 / 1 = 100 %.**
  Excluding the two declared-non-safety functions: 1 / 1 = 100 %.

**Reading:** hand-authored prose.

| F-xx | Function | SRs touching it |
|---|---|---|
| F-R-01 | Sense | SR-SYS-01, SR-R-01 |
