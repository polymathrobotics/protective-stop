# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
idf_component_register(
    SRCS
        "src/dcs_support.c"
        "src/dcs_identity.c"
        "src/dcs_nvs.c"
        "src/dcs_safety.c"
        "src/dcs_boot.c"
        "src/dcs_eth.c"
        "src/dcs_wifi.c"
        "src/dcs_net_supervisor.c"
        "src/dcs_rgb.c"
        "src/dcs_pstop_ring.c"
        "src/dcs_net_liveness.c"
        "src/dcs_net_inet.c"
        "src/dcs_telemetry.c"
        "src/dcs_admin_pages.c"
        "src/dcs_health.c"
        "src/dcs_health_logic.c"
        "src/panic_log.c"
    INCLUDE_DIRS
        "include"
        # Non-certified aux-channel schema shared by remote + machn (repo-root
        # common/). Exposed publicly so both ESP32 apps that REQUIRE dcs_support
        # can include "pstop_aux_channel.h". See common/pstop_aux_channel.h.
        "${CMAKE_CURRENT_LIST_DIR}/../../../common"
    PRIV_INCLUDE_DIRS
        "src"
    REQUIRES espcoredump
        spi_flash
        microlink
        ml_dev_tether
        pstop
        esp_http_server
        esp_netif
        esp_eth
        esp_event
        esp_timer
        esp_wifi
        nvs_flash
        app_update
        driver
        lwip
        wireguard_lwip
)
