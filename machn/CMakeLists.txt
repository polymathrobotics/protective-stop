# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)

# ESP32-S3 pstop MACHINE node ("machn"). Same platform baseline as the
# remote (../firmware) — microlink, dcs_support, vendored pstop_c — with a
# machine-role main: dual-core RX lockstep driving two series relays.
# *** REQUIRES ESP-IDF v5.5 OR NEWER (see ../firmware/CMakeLists.txt). ***

set(EXTRA_COMPONENT_DIRS
    "../components"
    "../components/microlink/components"
    "../firmware/components"
)

# Secrets are flashed per device (tools/provision_secrets.py), never compiled in.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.credentials")
    message(WARNING "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.credentials is ignored: "
        "secrets are no longer compiled in. Move its values to tools/credentials.env "
        "(see tools/credentials.env.example) and delete it.")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(machn_machine)

# Identity prefix: machn-01<mac24> instead of pstop-01<mac24>.
idf_component_get_property(dcs_lib dcs_support COMPONENT_LIB)
if(dcs_lib)
    target_compile_definitions(${dcs_lib} PRIVATE
        DCS_ID_PREFIX="machn"
        DCS_PAGE_TITLE="PSTOP Machine"
        DCS_PAGE_ICON="%F0%9F%A4%96"    # robot, percent-encoded (favicon data URI)
        DCS_PAGE_EMOJI="\\xF0\\x9F\\xA4\\x96" # robot, literal UTF-8 (page heading)
        DCS_PAGE_MACHINE=1

    )
endif()

# Same lwIP pbuf-pool override as the remote (see rationale there).
idf_component_get_property(lwip_lib lwip COMPONENT_LIB)
if(lwip_lib)
    target_compile_definitions(${lwip_lib} PRIVATE PBUF_POOL_SIZE=48)
endif()

# Same panic-banner tee as the remote (panic_log.c lives in dcs_support).
target_link_options(${CMAKE_PROJECT_NAME}.elf PRIVATE "-Wl,--wrap=panic_print_char")
