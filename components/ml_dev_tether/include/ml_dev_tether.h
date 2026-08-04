// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ml_dev_tether.h
 * @brief Opt-in USB-CDC-NCM tether for ESP32-S3 development workflow.
 *
 * Brings up the on-board USB-OTG controller as an NCM (Network Control Model)
 * Ethernet device, attaches it to lwIP as an esp_netif with DHCP client, and
 * waits up to `timeout_ms` for a DHCP lease from the host. If the lease
 * arrives, the function returns ESP_OK and the netif is set as the system
 * default route — giving the ESP internet via the connected host machine.
 *
 * Intended use: pass `ml_dev_tether_try_start` as ml_app_config_t.try_alt_network
 * so the ESP prefers the USB host when present and falls back to WiFi otherwise.
 *
 * Host-side setup (Linux): NetworkManager "shared" connection profile on the
 * new usbN interface, or dnsmasq + iptables MASQUERADE. See README in this
 * component directory.
 *
 * Caveats:
 *   - Composite USB descriptor (NCM + CDC). USB-Serial-JTAG console is
 *     replaced by a CDC ACM, so flashing/logging via /dev/ttyACM* still works
 *     but esptool needs --before no_reset_no_sync on some boards.
 *   - Only ESP32-S3 (and other targets with USB-OTG) supported.
 */

#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

  /**
 * @brief Bring up USB-NCM and wait for DHCP lease from host.
 *
 * @param timeout_ms  How long to wait for a DHCP lease before giving up.
 * @return  ESP_OK            on success (default route set to USB netif)
 *          ESP_ERR_TIMEOUT   USB enumerated but no DHCP within timeout
 *          ESP_ERR_NOT_FOUND USB host not present / not enumerating
 *          ESP_FAIL          init failure
 */
  esp_err_t ml_dev_tether_try_start(uint32_t timeout_ms);

  /**
 * @brief Tear down the tether (closes netif, disables TinyUSB if owned).
 * Optional — for cleanup paths. Currently a no-op if not started.
 */
  void ml_dev_tether_stop(void);

  /**
 * @brief Set the unit number for the "PSTOPxx" USB product string.
 * @param n  1..99 to force a specific number; 0 (default) derives it from the
 *           chip ID. Must be called before the first try_start — the USB
 *           descriptor is fixed at TinyUSB install.
 */
  void ml_dev_tether_set_unit_number(uint8_t n);

#ifdef __cplusplus
}
#endif
