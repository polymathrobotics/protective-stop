# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
#
# Security profile overlay for firmware/ and machn/; see docs/SECURITY_PROFILES.md.

set(PSTOP_PROFILE "secure-fe" CACHE STRING "Security profile: secure-fe, secure or dev")
# Index = how many of Secure Boot and flash encryption the profile turns on.
set(_pstop_profiles dev secure secure-fe)
list(FIND _pstop_profiles "${PSTOP_PROFILE}" _pstop_level)
if(_pstop_level EQUAL -1)
    message(FATAL_ERROR "PSTOP_PROFILE must be secure-fe, secure or dev (got '${PSTOP_PROFILE}')")
endif()

set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;${CMAKE_CURRENT_LIST_DIR}/${PSTOP_PROFILE}.defaults")
if(EXISTS "${CMAKE_SOURCE_DIR}/sdkconfig.credentials")
    list(APPEND SDKCONFIG_DEFAULTS "sdkconfig.credentials")
endif()

# SDKCONFIG_DEFAULTS only apply when sdkconfig is created, so an existing one keeps its profile.
set(_pstop_sdkconfig "${CMAKE_SOURCE_DIR}/sdkconfig")
if(SDKCONFIG)
    get_filename_component(_pstop_sdkconfig "${SDKCONFIG}" ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}")
endif()
if(EXISTS "${_pstop_sdkconfig}")
    file(STRINGS "${_pstop_sdkconfig}" _pstop_on REGEX "^CONFIG_(SECURE_BOOT|SECURE_FLASH_ENC_ENABLED)=y$")
    list(LENGTH _pstop_on _pstop_n)
    if(NOT _pstop_n EQUAL _pstop_level)
        message(FATAL_ERROR "${_pstop_sdkconfig} is from another profile: delete it and the build directory, "
            "then build again with -DPSTOP_PROFILE=${PSTOP_PROFILE}")
    endif()
endif()
