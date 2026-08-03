<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Protective-Stop — Coverage Report

Two coverage regimes, both driven toward the SIL 3 bar:
1. **Structural** — statement + branch now (gcov / gcovr); **MC/DC** via GCC-14
   `-fcondition-coverage` (installed 2026-08-02) — being rolled in.
2. **Requirements-driven** — % of safety requirements (`SAFETY_REQUIREMENTS.md`)
   with a verifying test, and % of safety-relevant lines traced to a requirement
   (`TRACEABILITY.md`, pending).

**Coverage-tooling policy (OD-1, settled 2026-08-02, user-confirmed).** Two tools
by design, and they never measure the same code: `pstop_c` keeps its own
**Bullseye** coverage + CI as pre-qualified evidence; **every other in-scope
codebase uses GCC-14 `gcov`/`gcovr`** (line + branch + `-fcondition-coverage`
MC/DC), the free flow already wired into `scripts/coverage.sh` and CI. Re-tooling
the pre-qualified library would add cost without adding evidence, and because the
two tools cover disjoint code there is no cross-tool comparability concern.

`pstop_c` is **pre-qualified** — it carries its own Bullseye coverage + CI and is
excluded from the numbers below.

_Baselines as of 2026-08-02. Numbers are first measurements, not targets met._

## 1. Structural coverage — baselines

| Codebase | Tool | Line | Branch | MC/DC | Driver |
|---|---|---|---|---|---|
| **Host machine** `machine_app_runner.c` | gcov-14 / gcovr | **71.4%** (332/465) | **57.9%** (202/349) | **58.3%** (189/324) | `pstop_multi_remote_test.py` (34/34) + `test_config_floor.py` (7/7, SR-H-03) |
| **ROS2 machine** (hand-written) | gcov-11 / gcovr | **~89%** (624/703) | see per-module | — | 6 gtest suites (56 tests): `test_json_lite`, `test_timing_floors`, `test_hardware_parse`, `test_hardware_backend`, `test_lifecycle`, `test_node_runtime` |
| **Remote firmware — decision core** `estop_verdict.c` | **host harness, gcc-14** | **100%** (26/26) | **100%** (36/36) | **100% MC/DC** (36/36) | `firmware/test/test_estop_verdict` (39/39) |

**Firmware = hybrid (decided 2026-08-02).** On-target gcov is infeasible — the
ESP32-S3 JTAG pins *are* the E-stop loop pins (GPIO39–42) and USB-Serial-JTAG is
displaced by the USB-NCM tether, and ESP-IDF gcov needs JTAG. So the SIL-critical
decision logic (both-phase sampling → Option A+B verdict → debounce → priming)
was extracted from `main.c` into the pure, HAL-free `estop_verdict.c` and covered
on the host with GCC-14 (`-fcondition-coverage` = MC/DC). The extraction is a
behaviour-preserving refactor: firmware rebuilds + links clean, and the on-target
path is validated functionally (the A/B HIL bench runs). _On-bench smoke of this
specific build is pending a direct tailnet path (bench currently DERP-relayed)._

### ROS2 per-module — updated 2026-08-02 (executor + HTTP-seam suites added)
Re-measured with gcovr 8.6 (line/branch counts differ slightly from the earlier
snapshot — 8.6 counts inline/header lines differently — so the `json_lite.hpp`
and `timing_floors.hpp` rows, untouched this round, are restated in the same
tool for consistency). `2026-08-02 was:` shows the prior figure where a module
was driven up this round.

| Module | Lines | Line cov | Branch cov | Note |
|---|---|---|---|---|
| `json_lite.hpp` | 183 | **76.5%** | 56.8% | unit-tested (`test_json_lite`); functions 100% |
| `software_backend.cpp` | 120 | **75.0%** (was 65.0%) | **40.0%** (was 34.0%) | `test_lifecycle` + `test_node_runtime` now run the machine loop live (start/activate/publish/stop); residual is pstop_c protocol callbacks + remote-enumeration, reachable only with live UDP remote traffic |
| `hardware_backend.cpp` | 142 | **98.6%** (was 24.6%) | **59.5%** (was 22.4%) | `test_hardware_parse` (parse) + new `test_hardware_backend` drives `http_get`/`http_post`/`write_cb`/poll/start/stop/`configure` against a loopback HTTP stub + a closed port (SR-M-03). Only the two `curl_easy_init()==NULL` OOM guards remain |
| `timing_floors.hpp` | 13 | **100%** | 65.6% | `test_timing_floors` (8/8) — runtime-config safety envelope (SR-M-01) |
| `machine_bridge_node.cpp` | 245 | **98.4%** (was 43.3%) | **49.8%** (was 27.7%) | `test_lifecycle` + new `test_node_runtime` spin an executor: `publish_tick`, every `diagnostics` summary branch (OK/WARN/ERROR, driven off canned `/state.json`), `on_set_parameters` accept/reject, the `~/configure_machine` service (apply/reject/backend-refused), and `on_error`/`on_shutdown`/activate-start-failure (SR-M-01/03/07). Functions 100% |
| `main.cpp` | 11 | **0%** | — | entry point only (deliberately excluded) |

Hand-written ROS2 is now **~89% line** (624/703 across `src`+`include`, excluding
the entry-point `main.cpp`) over **6 test suites (56 tests)** — up from ~53% line
/ 4 suites (27 tests) at the start of this round. The remaining structural gaps
are honest and bounded:
- **software_backend.cpp** — the pstop_c C callbacks (`cb_remote_details`,
  `cb_log`), the UDP message-processing branch in `run()`, and the
  remote-enumeration loop in `rebuild_snapshot()` fire only when a live pstop
  *remote* bonds over UDP. Covered functionally by the host `pstop_multi_remote`
  E2E, not by these in-process node tests (no remote peer stood up).
- **hardware_backend.cpp** — only the `curl_easy_init()==NULL` allocation-failure
  guards (2 lines) are uncovered; not reachable without malloc fault injection.
- **machine_bridge_node.cpp** — the residual lines are unreachable guards: the
  `publish_tick` null-backend early-return (the timer only exists while a backend
  does) and the `handle_configure` "no backend" branch (the service only exists
  once a backend is built). Branch % is bounded by rclcpp-internal conditionals
  in the message/QoS templates, not node logic.
- Branch coverage still trails line coverage because the libcurl 2xx/auth
  option-setting and the rclcpp publish/QoS machinery expand many
  compiler-visible conditionals that a functional test does not fan out.

## 2. Requirements coverage — baseline (`TRACEABILITY.md`)
Two definitions, reported both ways to stay honest:

| Metric | Value |
|---|---|
| SRs with ≥1 passing verifying test | **27/39 = 69.2%** |
| SRs **strictly fully-verified** | **15/39 = 38.5%** |
| Safety functions (F-xx) traced to ≥1 SR | **22/27 = 81.5%** (88% excl. 2 declared-non-safety) |

Status of the 39 SRs: **15 Verified · 11 Partial · 11 Unverified-gap · 2 Residual-accepted**
(SR-M-01 + SR-M-03 verified 2026-08-02 by the new ROS 2 executor/HTTP-seam suites).
`SR-R` (remote firmware) is the weak spot — 8/15 unverified, holding 5 of the 6
top DU gaps. Full matrix + test-gap register + function→SR reverse map in
`TRACEABILITY.md`. SIL3/PLe stays **allocated, not achieved**.

**P0 test/implementation gaps** (drive task #10): ~~SR-H-03 host config-floor
(DU-4)~~ **DONE 2026-08-02** (`cfg_validate` + `test_config_floor.py`, 7/7),
SR-R-09 GPIO re-verify (DU-1), SR-H-04 frozen-clock (DU-2),
SR-R-03 diversity anti-common-cause **non-identity** (note: `test_estop_verdict.c`
proves diversity *equivalence*, not the fault-injection *divergence* B needs —
that's the DU-3 gap), SR-R-08 memcmp self-test (DU-7), SR-R-12/SR-I-01
golden-vector encode (DU-5).

## 3. How to reproduce

**One-shot:** `scripts/coverage.sh` runs all three in-scope codebases (gcc-14 for
host + firmware-core MC/DC; colcon for ROS2) and writes
`docs/safety/coverage/SUMMARY.md`. CI runs the same script in the `ros:humble`
container on every push/PR (`.github/workflows/coverage.yml`) and posts the
summary. `SKIP_ROS2=1` for a host+firmware-only run. Manual per-codebase steps:

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
