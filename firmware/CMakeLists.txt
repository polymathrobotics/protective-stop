cmake_minimum_required(VERSION 3.16)

# *** REQUIRES ESP-IDF v5.5 OR NEWER. ***
# ml_dev_tether (USB-CDC-NCM) pulls espressif/esp_tinyusb ^2.0.0 which
# needs the IDF 5.5 TinyUSB ↔ USB-Serial-JTAG mode-switch fix on the
# ESP32-S3. Source ~/esp-idf-5.5/export.sh (not ~/esp-idf which is the
# tree's 5.3). See README.md for the full upgrade rationale.

set(EXTRA_COMPONENT_DIRS
    "../components"
    "../components/microlink/components"
    "components"
)

# Merge git-ignored credentials on top of defaults if present.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.credentials")
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.credentials")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(pstop_remote)

# Override lwIP's pbuf pool. IDF 5.5 doesn't expose PBUF_POOL_SIZE via
# Kconfig (the upstream lwip Kconfig wiring is missing), so the only way
# to grow it is to inject the macro into the lwip component's compile
# definitions. lwIP default is 16 — easy to exhaust on a 3-netif chip
# (USB-NCM + WiFi STA + WG) under sustained Tailscale chatter. 48 ×
# ~256 B = ~12 KiB of internal RAM, well within budget.
idf_component_get_property(lwip_lib lwip COMPONENT_LIB)
if(lwip_lib)
    target_compile_definitions(${lwip_lib} PRIVATE PBUF_POOL_SIZE=48)
endif()

# Wrap panic_print_char at the final link so calls from anywhere
# (including IDF's panic.c) route through main/panic_log.c's __wrap, which
# tees each byte into the RTC_NOINIT ringbuffer before forwarding to the
# real handler. Without this the panic banner + backtrace go to the
# console only — which on this board is nowhere.
target_link_options(${CMAKE_PROJECT_NAME}.elf PRIVATE "-Wl,--wrap=panic_print_char")
