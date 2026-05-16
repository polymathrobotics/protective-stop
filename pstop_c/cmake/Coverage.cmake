# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
#
# BullseyeCoverage targets for pstop_c. Only included when
# PSTOP_ENABLE_COVERAGE=ON (see top-level CMakeLists.txt).
#
# These targets auto-toggle `cov01 --on/--off` so devs don't manage that
# state by hand. `--clean-first` forces a full rebuild after `cov01 --on`,
# guaranteeing every object in the test binary is instrumented even when a
# stale non-coverage build dir is reused.
#
# Caveat: if a target step fails mid-flight, `cov01 --off` does not run and
# the user's Bullseye state stays "on". `scripts/run-coverage.sh` adds a
# shell `trap` to recover from that on the wrapper path.
#
# Targets:
#   coverage       — wipe .cov, instrument + build, run tests, import region
#                    selections, print covsrc summary.
#   coverage-html  — same flow, plus a covhtml report in build/coverage-html/.

set(_pstop_regions_file "${CMAKE_SOURCE_DIR}/.bullseye/covselect.txt")
set(_pstop_html_dir     "${CMAKE_BINARY_DIR}/coverage-html")

set(_pstop_cov_preamble
  COMMAND ${CMAKE_COMMAND} -E rm -f ${PSTOP_COVFILE}
  COMMAND ${BULLSEYE_COV01} --on
  # COVFILE must be set at BUILD time too: the compiler wrappers write the
  # static function/region table to it during compilation. Without this, the
  # build writes static data to the default ./test.cov, the runtime then
  # writes hit counts to our PSTOP_COVFILE, and reports come out empty
  # because PSTOP_COVFILE has no function definitions to attribute hits to.
  # Build pstop_test (drags in pstop) with a clean tree so every object is
  # instrumented. Then build transport_udp separately: nothing in pstop_test
  # links it, so a bare pstop_test build would skip it and it would silently
  # never appear in the coverage report. Building it forces instrumentation
  # to land in the .cov; runtime hits stay zero until tests exercise it,
  # which is the honest answer for the current test gap.
  COMMAND ${CMAKE_COMMAND} -E env COVFILE=${PSTOP_COVFILE}
          ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR}
          --target pstop_test --clean-first
  COMMAND ${CMAKE_COMMAND} -E env COVFILE=${PSTOP_COVFILE}
          ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR}
          --target transport_udp
  COMMAND ${CMAKE_COMMAND} -E env COVFILE=${PSTOP_COVFILE}
          $<TARGET_FILE:pstop_test>
  COMMAND ${CMAKE_COMMAND} -E env COVFILE=${PSTOP_COVFILE}
          ${BULLSEYE_COVSELECT} --import ${_pstop_regions_file}
)

add_custom_target(coverage
  ${_pstop_cov_preamble}
  COMMAND ${CMAKE_COMMAND} -E env COVFILE=${PSTOP_COVFILE}
          ${BULLSEYE_COVSRC} -q
  COMMAND ${BULLSEYE_COV01} --off
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMENT "BullseyeCoverage: running tests + summary"
  VERBATIM
)

add_custom_target(coverage-html
  ${_pstop_cov_preamble}
  COMMAND ${CMAKE_COMMAND} -E env COVFILE=${PSTOP_COVFILE}
          ${BULLSEYE_COVHTML} --srcdir ${CMAKE_SOURCE_DIR} ${_pstop_html_dir}
  COMMAND ${BULLSEYE_COV01} --off
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMENT "BullseyeCoverage: running tests + HTML report at ${_pstop_html_dir}"
  VERBATIM
)
