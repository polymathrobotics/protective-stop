# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
idf_component_register(
    SRCS "main.c" "estop_verdict.c"
    INCLUDE_DIRS "."
    REQUIRES dcs_support pstop
    PRIV_REQUIRES hal soc
)
