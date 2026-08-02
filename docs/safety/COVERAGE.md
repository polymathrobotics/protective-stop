<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Protective-Stop — Coverage Report

Two coverage regimes, both driven toward the SIL 3 bar:
1. **Structural** — statement + branch now (gcov / gcovr); **MC/DC** via GCC-14
   `-fcondition-coverage` (installed 2026-08-02) — being rolled in.
2. **Requirements-driven** — % of safety requirements (`SAFETY_REQUIREMENTS.md`)
   with a verifying test, and % of safety-relevant lines traced to a requirement
   (`TRACEABILITY.md`, pending).

`pstop_c` is **pre-qualified** — it carries its own Bullseye coverage + CI and is
excluded from the numbers below.

_Baselines as of 2026-08-02. Numbers are first measurements, not targets met._

## 1. Structural coverage — baselines

| Codebase | Tool | Line | Branch | MC/DC | Driver |
|---|---|---|---|---|---|
| **Host machine** `machine_app_runner.c` | gcov-11 / gcovr | **70.4%** (311/442) | **56.8%** (187/329) | pending (gcc-14 re-run) | `tools/pstop_multi_remote_test.py` (34/34 invariant checks) |
| **ROS2 machine** (hand-written) | gcov-11 / gcovr | see per-module | — | — | `test_json_lite` (4/4) only |
| **Remote firmware** | on-target ESP32 gcov | not started (task #8) | | | transport decision pending |

### ROS2 per-module (the gap is stark and expected)
| Module | Lines | Line cov | Branch cov | Note |
|---|---|---|---|---|
| `json_lite.hpp` | 119 | **84.0%** | 53.4% | only unit-tested module (functions 100%) |
| `software_backend.cpp` | 222 | **0%** | — | no unit test executes it |
| `hardware_backend.cpp` | 180 | **0%** | — | no unit test executes it |
| `machine_bridge_node.cpp` | 334 | **0%** | — | no unit test executes it |
| `main.cpp` | 24 | **0%** | — | no unit test executes it |

Only **2 of 114** instrumented translation units execute under the current test
suite. The node + both backends have been HW-tested end-to-end but have **no unit
tests** — that is the primary structural-coverage debt (task #10).

## 2. Requirements coverage — pending
Awaiting `TRACEABILITY.md` (task #9). Input ready: `SAFETY_REQUIREMENTS.md` marks
18/39 requirements Satisfied-in-code (cited), 11 Gap, 8 Partial, 2
Residual-accepted. Requirements coverage will report, per requirement, whether a
**requirement-traced test** (`SR-<area>-nn-k` style, parallel to `pstop_c`'s
`req_*` tests) exists and passes.

## 3. How to reproduce

**Host** (statement+branch now; add `-fcondition-coverage` + `gcov-14` for MC/DC):
```
cd host && gcc -O0 -g --coverage -I../pstop_c/pstop/include -I../pstop_c/transport/include \
  -o machine_app_runner machine_app_runner.c ../pstop_c/pstop/src/pstop/*.c \
  ../pstop_c/transport/src/transport/udp/udp_transport.c
python3 ../tools/pstop_multi_remote_test.py
gcovr --filter 'host/machine_app_runner\.c' --print-summary
```

**ROS2** (separate `build-cov`, does not disturb the normal build):
```
cd ros2 && source /opt/ros/humble/setup.bash
colcon build --packages-up-to protective_stop_machine --build-base build-cov \
  --install-base install-cov --cmake-args -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-O0 -g --coverage" -DCMAKE_EXE_LINKER_FLAGS="--coverage"
colcon test --packages-select protective_stop_machine --build-base build-cov
gcovr --root protective_stop_machine build-cov/protective_stop_machine \
  --filter '.*/protective_stop_machine/(src|include)/.*'
```

**Firmware:** on-target gcov — see task #8 (transport TBD).

## 4. Notes on MC/DC
GCC 14.3 (`-fcondition-coverage`) covers host + ROS2 in the same gcov/gcovr flow.
ROS2 caveat: gcc-14 against Humble's gcc-11 prebuilt libraries carries C++ ABI
risk — to be validated before making gcc-14 the ROS2 default; branch coverage on
gcc-11 is unaffected. On-target firmware MC/DC depends on the ESP-IDF Xtensa
toolchain's gcc version (likely no `-fcondition-coverage`) — branch coverage
on-target, MC/DC via a host-compiled logic harness is the likely split.
