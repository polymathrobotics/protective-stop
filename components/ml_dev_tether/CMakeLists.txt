# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
idf_component_register(
    SRCS "src/ml_dev_tether.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_netif esp_event lwip
)
