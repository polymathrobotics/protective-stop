# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
idf_component_register(
    SRCS "src/ml_dev_tether.c" "src/ml_usb_tx.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_netif esp_event lwip
    PRIV_REQUIRES espressif__esp_tinyusb espressif__tinyusb
)

# Route NCM copies for project-owned requests without relying on Espressif's
# private packet_t layout. The wrapper forwards every other reference.
target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=tud_network_xmit_cb")
include("${CMAKE_CURRENT_LIST_DIR}/tinyusb_defer_patch.cmake")
