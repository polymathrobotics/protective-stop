# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
#
# BullseyeCoverage targets, enabled by PSTOP_ENABLE_COVERAGE.
#
#   coverage       — covsrc summary
#   coverage-html  — covsrc + HTML report at build/coverage-html/

set(_pstop_regions_file "${CMAKE_SOURCE_DIR}/.bullseye/covselect.txt")
set(_pstop_html_dir     "${CMAKE_BINARY_DIR}/coverage-html")

set(_pstop_cov_preamble
  COMMAND ${CMAKE_COMMAND} -E rm -f ${PSTOP_COVFILE}
  COMMAND ${BULLSEYE_COV01} --on
  # COVFILE must be set at build time too — otherwise compile writes the
  # static function table to ./test.cov and our .cov ends up empty.
  COMMAND ${CMAKE_COMMAND} -E env COVFILE=${PSTOP_COVFILE}
          ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR}
          --target pstop_test --clean-first
  # transport_udp isn't linked by pstop_test, so build it explicitly or it
  # silently drops out of the report.
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
