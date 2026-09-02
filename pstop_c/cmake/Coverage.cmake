# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
#
# BullseyeCoverage targets, enabled by PSTOP_ENABLE_COVERAGE.
#
#   coverage                    — every suite, covsrc summary
#   coverage-html               — every suite, covsrc + HTML report
#   coverage-unit               — pstop_test only
#   coverage-requirements       — pstop_requirements_test only
#   <suite>-html                — that suite's HTML report
#
# One instrumented build writes PSTOP_COVFILE; each suite then runs against its
# own copy of that baseline, so the suites share a compile and a denominator.

set(_pstop_regions_file "${CMAKE_SOURCE_DIR}/.bullseye/covselect.txt")

get_filename_component(_pstop_base_covfile "${PSTOP_COVFILE}" NAME)

add_custom_target(coverage-build
  COMMAND ${CMAKE_COMMAND} -E rm -f ${PSTOP_COVFILE}
  COMMAND ${BULLSEYE_COV01} --on
  # COVFILE must be set at build time too — otherwise compile writes the static
  # function table to ./test.cov and our .cov ends up empty. Absolute here so
  # every compiler invocation writes the one file whatever its directory.
  COMMAND ${CMAKE_COMMAND} -E env COVFILE=${PSTOP_COVFILE}
          ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --clean-first
  COMMAND ${CMAKE_COMMAND} -E env COVFILE=${_pstop_base_covfile}
          ${BULLSEYE_COVSELECT} --import ${_pstop_regions_file}
  COMMAND ${BULLSEYE_COV01} --off
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMENT "BullseyeCoverage: instrumented build + baseline covfile"
  VERBATIM
)

# pstop_add_coverage_suite(SUFFIX <s> TEST <target> COVFILE <name>)
#
# Generates coverage${SUFFIX} and coverage${SUFFIX}-html.
function(pstop_add_coverage_suite)
  cmake_parse_arguments(ARG "" "SUFFIX;TEST;COVFILE;REGIONS" "TARGETS" ${ARGN})

  # Optional per-suite reporting selection, narrowing the baseline's regions.
  # covhtml and argument-less covsrc honour it; covsrc reports whatever files it
  # is handed, so TARGETS below has to scope the per-file table to match.
  set(_select "")
  if(ARG_REGIONS)
    set(_select
      COMMAND ${CMAKE_COMMAND} -E env COVFILE=${ARG_COVFILE}
              ${BULLSEYE_COVSELECT} --import ${CMAKE_SOURCE_DIR}/${ARG_REGIONS}
    )
  endif()

  # The sources this suite reports on, taken from the library targets themselves
  # rather than restated as globs. run-coverage.sh reads the list back.
  set(_srcs "")
  foreach(_target IN LISTS ARG_TARGETS)
    get_target_property(_target_dir ${_target} SOURCE_DIR)
    get_target_property(_target_srcs ${_target} SOURCES)
    foreach(_src IN LISTS _target_srcs)
      if(NOT IS_ABSOLUTE "${_src}")
        set(_src "${_target_dir}/${_src}")
      endif()
      string(APPEND _srcs "${_src}\n")
    endforeach()
  endforeach()
  file(GENERATE
    OUTPUT  ${CMAKE_BINARY_DIR}/coverage-sources${ARG_SUFFIX}.txt
    CONTENT "${_srcs}"
  )

  # covsrc/covhtml read an absolute COVFILE as an empty report — the imported
  # regions stop matching — so the covfile is named relative to the build dir,
  # which is every command's working directory.
  #
  # The test is a plain path, not $<TARGET_FILE:>: that generator expression
  # adds an implicit dependency, so make would build it outside coverage-build
  # and link uninstrumented objects against the instrumented library.
  set(_run
    COMMAND ${CMAKE_COMMAND} -E copy ${PSTOP_COVFILE} ${ARG_COVFILE}
    ${_select}
    COMMAND ${CMAKE_COMMAND} -E env COVFILE=${ARG_COVFILE}
            ${CMAKE_BINARY_DIR}/pstop/${ARG_TEST}
  )

  add_custom_target(coverage${ARG_SUFFIX}
    ${_run}
    COMMAND ${CMAKE_COMMAND} -E env COVFILE=${ARG_COVFILE}
            ${BULLSEYE_COVSRC} -q
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "BullseyeCoverage: running ${ARG_TEST} + summary"
    VERBATIM
  )

  add_custom_target(coverage${ARG_SUFFIX}-html
    ${_run}
    COMMAND ${CMAKE_COMMAND} -E env COVFILE=${ARG_COVFILE}
            ${BULLSEYE_COVHTML} --srcdir ${CMAKE_SOURCE_DIR}
            ${CMAKE_BINARY_DIR}/coverage-html${ARG_SUFFIX}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "BullseyeCoverage: running ${ARG_TEST} + HTML report"
    VERBATIM
  )

  add_dependencies(coverage${ARG_SUFFIX} coverage-build)
  add_dependencies(coverage${ARG_SUFFIX}-html coverage-build)
endfunction()

pstop_add_coverage_suite(
  SUFFIX  "-unit"
  TEST    pstop_test
  COVFILE pstop_unit.cov
  TARGETS pstop transport_udp
)

pstop_add_coverage_suite(
  SUFFIX  "-requirements"
  TEST    pstop_requirements_test
  COVFILE pstop_requirements.cov
  REGIONS .bullseye/covselect-requirements.txt
  TARGETS pstop
)

add_custom_target(coverage)
add_dependencies(coverage coverage-unit coverage-requirements)

add_custom_target(coverage-html)
add_dependencies(coverage-html coverage-unit-html coverage-requirements-html)
