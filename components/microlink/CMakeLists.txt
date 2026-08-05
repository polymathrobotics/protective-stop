# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
set(SRCS
    "src/microlink.c"
    "src/ml_coord.c"
    "src/ml_derp.c"
    "src/ml_net_io.c"
    "src/ml_wg_mgr.c"
    "src/ml_stun.c"
    "src/ml_noise.c"
    "src/ml_h2.c"
    "src/ml_udp.c"
    "src/ml_peer_nvs.c"
    "src/ml_perf_bench.c"
    "src/nacl_box.c"
    "src/x25519.c"
    "src/ml_zerocopy.c"
    "src/ml_config_httpd.c"
    "src/ml_app.c"
)

set(REQUIRES
    nvs_flash
    esp_wifi
    esp_netif
    esp_event
    esp_timer
    lwip
    mbedtls
    json
    driver
    esp_driver_tsens
    esp_http_server
)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS
        "include"
        "src"
    REQUIRES ${REQUIRES}
    PRIV_REQUIRES
        wireguard_lwip
        app_update
        esp_http_client
)

# Crypto speedup (from fast-crypto, 2026-08-04): compile the ALREADY-VETTED,
# byte-identical-output primitives at -O3 (project default is -Og). Measured
# ~2.5x on X25519 on real ESP32-S3 silicon; SAME code, output verified identical
# vs -Og on-device. No new crypto in the safety path — just the compiler flag.
# X25519 is handshake-only (WG handshake/rekey + disco cold-contact), so this is
# a latency win on bond setup, not per-packet. See docs/CRYPTO_SPEEDUP_REPORT.md.
set_source_files_properties(
    "src/x25519.c"
    "src/nacl_box.c"
    PROPERTIES COMPILE_OPTIONS "-O3")
