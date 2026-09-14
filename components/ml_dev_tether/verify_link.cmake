# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0

# Included by each project AFTER project() creates the final ELF target.
add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD
    COMMAND ${PYTHON} "${CMAKE_CURRENT_LIST_DIR}/../../scripts/check_usb_tx_link.py"
        --objdump "${CMAKE_OBJDUMP}" --elf "$<TARGET_FILE:${CMAKE_PROJECT_NAME}.elf>"
    VERBATIM
)
