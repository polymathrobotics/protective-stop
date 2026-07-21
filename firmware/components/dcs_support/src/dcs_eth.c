// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dcs_eth.c
 * @brief W5500 SPI Ethernet bring-up for the Waveshare ESP32-S3-ETH board.
 *
 * This is the highest-priority network path. The board is PoE-powered and
 * normally lives on a wired LAN, so Ethernet should win the default route
 * over USB-NCM and WiFi whenever its link is up.
 *
 * Design notes:
 *   - The netif is created with route_prio = DCS_ETH_ROUTE_PRIO (128), above
 *     USB-NCM (110, bumped in ml_dev_tether) and WiFi STA (100, IDF default).
 *     esp_netif's own default-route selection therefore agrees with the
 *     dcs_net_supervisor's explicit 1 Hz pin.
 *   - dcs_eth_start() is non-blocking: it brings up SPI + the W5500 driver +
 *     the netif and starts link detection, then returns. It does NOT tear the
 *     netif down on a missing link — the cable can be plugged in later and the
 *     supervisor will promote Ethernet to the default route automatically.
 *   - dcs_eth_wait_for_ip() blocks for the boot path so dcs_boot can decide
 *     whether Ethernet came up fast enough to skip the WiFi connect.
 *
 * Pin map (Waveshare ESP32-S3-ETH schematic):
 *   MOSI=GPIO11  MISO=GPIO12  SCLK=GPIO13  CS=GPIO14  INT=GPIO10  RST=GPIO9
 */

#include <stdatomic.h>
#include <string.h>

#include "dcs_internal.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_eth_netif_glue.h"
#include "esp_eth_phy.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_netif_defaults.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char * TAG = "dcs_eth";

/* --- Waveshare ESP32-S3-ETH W5500 pin map --- */
#define DCS_ETH_SPI_HOST SPI2_HOST
#define DCS_ETH_PIN_MOSI 11
#define DCS_ETH_PIN_MISO 12
#define DCS_ETH_PIN_SCLK 13
#define DCS_ETH_PIN_CS 14
#define DCS_ETH_PIN_INT 10
#define DCS_ETH_PIN_RST 9
#define DCS_ETH_SPI_CLOCK_MHZ 20 /* conservative; W5500 tolerates up to ~33 */
#define DCS_ETH_PHY_ADDR 1

static esp_netif_t * s_eth_netif = NULL;
static esp_eth_handle_t s_eth_handle = NULL;
static SemaphoreHandle_t s_got_ip_sem = NULL;
static atomic_bool s_link_up = false;
static atomic_bool s_has_ip = false;
static atomic_bool s_enabled = false; /* admin toggle: eth driver running */

/* === Event handlers ======================================================= */

static void on_eth_event(void * arg, esp_event_base_t base, int32_t id, void * data)
{
  (void)arg;
  (void)base;
  (void)data;
  switch (id) {
    case ETHERNET_EVENT_CONNECTED:
      atomic_store(&s_link_up, true);
      ESP_LOGI(TAG, "W5500 link UP");
      break;
    case ETHERNET_EVENT_DISCONNECTED:
      atomic_store(&s_link_up, false);
      atomic_store(&s_has_ip, false); /* DHCP lease is no longer valid */
      ESP_LOGW(TAG, "W5500 link DOWN");
      break;
    default:
      break;
  }
}

static void on_eth_got_ip(void * arg, esp_event_base_t base, int32_t id, void * data)
{
  (void)arg;
  (void)base;
  (void)id;
  ip_event_got_ip_t * evt = (ip_event_got_ip_t *)data;
  if ((evt == NULL) || (evt->esp_netif != s_eth_netif)) {
    return;
  }
  atomic_store(&s_has_ip, true);
  ESP_LOGI(TAG, "W5500 got IP " IPSTR " gw " IPSTR, IP2STR(&evt->ip_info.ip), IP2STR(&evt->ip_info.gw));
  if (s_got_ip_sem != NULL) {
    (void)xSemaphoreGive(s_got_ip_sem);
  }
}

/* === Public-ish (component-internal) API ================================== */

esp_err_t dcs_eth_start(void)
{
  if (s_eth_netif != NULL) {
    ESP_LOGW(TAG, "already started");
    return ESP_OK;
  }

  /* esp_netif + default event loop may not exist yet if we run before
     * ml_app's wifi_init. Both are idempotent (INVALID_STATE == already up). */
  esp_err_t err = esp_netif_init();
  if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
    ESP_LOGE(TAG, "esp_netif_init: %s", esp_err_to_name(err));
    return err;
  }
  err = esp_event_loop_create_default();
  if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
    ESP_LOGE(TAG, "esp_event_loop_create_default: %s", esp_err_to_name(err));
    return err;
  }

  s_got_ip_sem = xSemaphoreCreateBinary();
  if (s_got_ip_sem == NULL) {
    return ESP_ERR_NO_MEM;
  }

  /* Cleanup-ladder state, declared before the first goto so the fail_*
     * labels can reference them regardless of how far bring-up got. */
  bool spi_owned = false;
  esp_eth_mac_t * mac = NULL;
  esp_eth_phy_t * phy = NULL;

  /* GPIO ISR service for the W5500 INT line. Shared process-wide. */
  err = gpio_install_isr_service(0);
  if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
    ESP_LOGE(TAG, "gpio_install_isr_service: %s", esp_err_to_name(err));
    goto fail_sem;
  }

  /* --- SPI bus --- */
  spi_bus_config_t buscfg = {
    .miso_io_num = DCS_ETH_PIN_MISO,
    .mosi_io_num = DCS_ETH_PIN_MOSI,
    .sclk_io_num = DCS_ETH_PIN_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
  };
  err = spi_bus_initialize(DCS_ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
  if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
    ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
    goto fail_spi; /* spi_owned still false → no-op there, then frees sem */
  }
  /* We own the SPI bus (and must free it on a later failure) only if WE
     * initialized it — INVALID_STATE means another component owns it. */
  spi_owned = (err == ESP_OK);

  /* --- W5500 MAC + PHY --- */
  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.phy_addr = DCS_ETH_PHY_ADDR;
  phy_config.reset_gpio_num = DCS_ETH_PIN_RST;

  spi_device_interface_config_t spi_devcfg = {
    .mode = 0,
    .clock_speed_hz = DCS_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
    .queue_size = 20,
    .spics_io_num = DCS_ETH_PIN_CS,
  };

  eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(DCS_ETH_SPI_HOST, &spi_devcfg);
  w5500_config.int_gpio_num = DCS_ETH_PIN_INT; /* interrupt-driven RX */
  w5500_config.poll_period_ms = 0;

  mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
  phy = esp_eth_phy_new_w5500(&phy_config);
  if ((mac == NULL) || (phy == NULL)) {
    ESP_LOGE(TAG, "failed to create W5500 MAC/PHY");
    err = ESP_FAIL;
    goto fail_driver; /* dels whichever of mac/phy is non-NULL */
  }

  esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
  err = esp_eth_driver_install(&eth_cfg, &s_eth_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_eth_driver_install: %s", esp_err_to_name(err));
    s_eth_handle = NULL; /* ensure the ladder doesn't uninstall a non-handle */
    goto fail_driver;
  }

  /* W5500 has no factory MAC — set one derived from the chip's eFuse. */
  uint8_t eth_mac[6];
  (void)esp_read_mac(eth_mac, ESP_MAC_ETH);
  err = esp_eth_ioctl(s_eth_handle, ETH_CMD_S_MAC_ADDR, eth_mac);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "set MAC: %s (continuing)", esp_err_to_name(err));
  }

  /* --- esp_netif (Ethernet class, DHCP client, top route priority) --- */
  esp_netif_inherent_config_t base_cfg = ESP_NETIF_INHERENT_DEFAULT_ETH();
  base_cfg.route_prio = DCS_ETH_ROUTE_PRIO; /* 128 — above USB-NCM/WiFi */
  base_cfg.if_key = "DCS_ETH";
  base_cfg.if_desc = "eth_w5500";
  esp_netif_config_t netif_cfg = {
    .base = &base_cfg,
    .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
  };
  s_eth_netif = esp_netif_new(&netif_cfg);
  if (s_eth_netif == NULL) {
    ESP_LOGE(TAG, "esp_netif_new failed");
    err = ESP_FAIL;
    goto fail_driver;
  }

  /* Attach driver to netif (the glue auto-registers the ETH_EVENT ->
     * esp_netif_action_* handlers that start DHCP on link-up). */
  err = esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth_handle));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_netif_attach: %s", esp_err_to_name(err));
    goto fail_netif;
  }

  /* Our own handlers: link state + IP for telemetry/boot gating. */
  (void)esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, on_eth_event, NULL);
  (void)esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, on_eth_got_ip, NULL);

  err = esp_eth_start(s_eth_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_eth_start: %s", esp_err_to_name(err));
    (void)esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, on_eth_event);
    (void)esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP, on_eth_got_ip);
    goto fail_netif;
  }

  atomic_store(&s_enabled, true);
  ESP_LOGI(
    TAG,
    "W5500 Ethernet started (SPI%d cs=%d int=%d rst=%d @%dMHz, "
    "MAC %02x:%02x:%02x:%02x:%02x:%02x, route_prio=%d)",
    (int)DCS_ETH_SPI_HOST + 1,
    DCS_ETH_PIN_CS,
    DCS_ETH_PIN_INT,
    DCS_ETH_PIN_RST,
    DCS_ETH_SPI_CLOCK_MHZ,
    eth_mac[0],
    eth_mac[1],
    eth_mac[2],
    eth_mac[3],
    eth_mac[4],
    eth_mac[5],
    DCS_ETH_ROUTE_PRIO);
  return ESP_OK;

  /* Failure cleanup ladder — free exactly what THIS call created, in
     * reverse order, leaving shared/idempotent resources (esp_netif_init,
     * default event loop, GPIO ISR service) alone. Boot-path failure is
     * fatal (the safety chain reboots), but a clean unwind means a retry —
     * or a partially-shared SPI bus — isn't left in a half-initialized
     * state, and no handle/semaphore leaks. */
fail_netif:
  if (s_eth_netif != NULL) {
    esp_netif_destroy(s_eth_netif); /* releases the glue's driver reference */
    s_eth_netif = NULL; /* don't leave a phantom "DCS_ETH" netif */
  }
fail_driver:
  /* Per the IDF contract, uninstall does NOT free mac/phy — the caller dels
     * them, and only after the netif (which holds a driver reference) is gone. */
  if (s_eth_handle != NULL) {
    (void)esp_eth_driver_uninstall(s_eth_handle);
    s_eth_handle = NULL;
  }
  if (phy != NULL) {
    (void)phy->del(phy);
  }
  if (mac != NULL) {
    (void)mac->del(mac);
  }
fail_spi:
  if (spi_owned) {
    (void)spi_bus_free(DCS_ETH_SPI_HOST);
  }
fail_sem:
  vSemaphoreDelete(s_got_ip_sem);
  s_got_ip_sem = NULL;
  return err;
}

bool dcs_eth_wait_for_ip(uint32_t timeout_ms)
{
  if (s_got_ip_sem == NULL) {
    return false;
  }
  if (atomic_load(&s_has_ip)) {
    return true;
  }
  return xSemaphoreTake(s_got_ip_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

bool dcs_eth_link_up(void)
{
  return atomic_load(&s_link_up);
}

bool dcs_eth_has_ip(void)
{
  return atomic_load(&s_has_ip);
}

bool dcs_eth_is_enabled(void)
{
  return atomic_load(&s_enabled);
}

/* Admin toggle: stop/start the W5500 driver. Stopping drops the link so the
 * netif loses its lease and dcs_net_supervisor demotes Ethernet → failover to
 * the next interface; starting re-acquires DHCP and the supervisor promotes it
 * back. Returns ESP_ERR_INVALID_STATE if the driver was never installed. */
esp_err_t dcs_eth_set_enabled(bool on)
{
  if (s_eth_handle == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  if (on == atomic_load(&s_enabled)) {
    return ESP_OK;
  }
  esp_err_t err = on ? esp_eth_start(s_eth_handle) : esp_eth_stop(s_eth_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "eth %s: %s", on ? "start" : "stop", esp_err_to_name(err));
    return err;
  }
  if (!on) {
    atomic_store(&s_link_up, false);
    atomic_store(&s_has_ip, false);
  }
  atomic_store(&s_enabled, on);
  ESP_LOGW(TAG, "Ethernet %s via admin toggle", on ? "ENABLED" : "DISABLED");
  return ESP_OK;
}
