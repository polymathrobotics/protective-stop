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
#include "esp_rom_sys.h" /* esp_rom_delay_us — datasheet-correct RST pulse width */
#include "esp_timer.h" /* esp_timer_get_time — watchdog silence/timeout math   */
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

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

/* MAC/PHY object handles, stored on a successful bring-up so the health
 * watchdog's recovery ladder can del + recreate them across a full driver
 * reinstall (esp_eth_driver_uninstall does NOT free them — the caller does).
 * Left NULL on the boot fail path (which reboots) — only the success path
 * publishes them here, so eth_teardown() never double-frees. */
static esp_eth_mac_t * s_mac = NULL;
static esp_eth_phy_t * s_phy = NULL;

/* Serialises Ethernet lifecycle transitions (admin enable/disable vs the
 * watchdog's stop/start/reinstall ladder) so the two can't race the driver
 * FSM. The safety comparator/send loop never takes this — it only ever does
 * lock-free sendto/recvfrom on its own socket, untouched by recovery. */
static SemaphoreHandle_t s_eth_lock = NULL;
static bool s_wd_started = false; /* watchdog task spawned exactly once */

/* === W5500 SPI-Ethernet health watchdog + bounded reset-recovery ==========
 *
 * Closes the deferred-P1 "unrecoverable SPI/PHY wedge" gap (ESP-IDF issues
 * #11845 "unrecoverable SPI errors, no ETH event", #12058 "errno 113 can't
 * recover", esp32.com "W5500 reset-timeout"): the W5500 can black-hole with NO
 * ETHERNET_EVENT_DISCONNECTED and the netif still reporting UP with a valid
 * lease, so the 1 Hz route supervisor never demotes Ethernet and the link stays
 * wedged until a human power-cycles. The system still fails safe throughout (the
 * machine STOPs on heartbeat silence) — this only adds SELF-HEALING.
 *
 * Detection (either signal, must PERSIST DCS_ETHWD_CONFIRM_TICKS ticks):
 *   (A) SPI probe — a periodic read-only esp_eth_ioctl(ETH_CMD_READ_PHY_REG)
 *       returns != ESP_OK (a wedged/timed-out bus surfaces here).
 *   (B) Black-hole — driver link UP + valid lease + Ethernet is the active route,
 *       yet pstop has seen NO reply from ANY peer for DCS_ETHWD_SILENCE_MS AND the
 *       uplink gateway is likewise silent (or never answered ICMP). The gateway
 *       cross-check discriminates a local W5500 wedge from a merely-unreachable
 *       peer: if the gateway still answers, our uplink is fine and we hold.
 *
 * Recovery ladder (escalates only if the prior rung fails to restore link +
 * an SPI probe pass within DCS_ETHWD_RUNG_TIMEOUT_MS):
 *   1. esp_eth_stop()/esp_eth_start() — cheapest; clears a stuck driver FSM /
 *      link timer (does NOT re-init the chip).
 *   2. Full driver+netif rebuild via the existing dcs_eth_start() path —
 *      esp_eth_driver_install() re-runs phy->reset_hw + mac->init (W5500 SW reset
 *      + VERSIONR verify) + phy->init, re-establishing SPI from scratch.
 *   3. A datasheet-correct hardware RST pulse on GPIO9 (assert >= T_RC 500 us,
 *      wait >= T_PL 1 ms for PLL lock) THEN the same rebuild — last resort for a
 *      PLL/logic wedge the IDF's built-in 100 us reset_hw pulse is too short to
 *      clear. Board schematic: RSTn->GPIO9 through a 20 ohm series R (R22), no
 *      external RC or pull-up, so the GPIO drives the pin cleanly.
 *
 * Runs on a NON-safety PSRAM-stack task (mirrors the supervisor/liveness tasks);
 * never blocks or touches the comparator/send loop; never changes fail-safe
 * semantics. After a full ladder it enters a cooldown so it can never thrash. */
#define DCS_ETHWD_POLL_MS 1000u /* 1 Hz health check                         */
#define DCS_ETHWD_CONFIRM_TICKS 3u /* consecutive suspicion ticks before acting */
#define DCS_ETHWD_SILENCE_MS 8000u /* pstop + gateway silent this long => wedge */
#define DCS_ETHWD_RUNG_TIMEOUT_MS 8000u /* per-rung wait for link + probe pass  */
#define DCS_ETHWD_COOLDOWN_MS 60000u /* hold-off after a full ladder (anti-thrash) */

/* W5500 RESET electrical timing (datasheet v1.1.0 sec 5.5.1, Fig.22 Reset Timing):
 *   T_RC (Reset Cycle Time)      min 500 us  — RSTn low pulse width
 *   T_PL (RSTn -> internal PLOCK) max 1 ms    — PLL lock after RSTn released
 * We use generous margins (1 ms assert, 5 ms PLL wait). */
#define DCS_ETHWD_RST_ASSERT_US 1000u
#define DCS_ETHWD_RST_PLL_MS 5u

/* PHYCFGR common register, reconstructed from the IDF-private w5500.h mapping
 * W5500_MAKE_MAP(0x002E, BSB_COM_REG=0) = ((0x2E)<<16 | (0)<<3). The W5500 MAC's
 * phy_reg_read accepts ONLY this register, and reading it drives a real SPI
 * transaction — so a wedged bus is observable as esp_eth_ioctl() != ESP_OK. */
#define DCS_W5500_REG_PHYCFGR (((uint32_t)0x002Eu << 16) | ((uint32_t)0x00u << 3))

/* Last-recovery reason codes published to /state.json (eth_rec_reason). */
#define DCS_ETHWD_REASON_NONE 0u
#define DCS_ETHWD_REASON_SPI 1u /* SPI probe failed          */
#define DCS_ETHWD_REASON_BLACKHOLE 2u /* link UP + lease but all peers silent */

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

/* === Health watchdog + bounded reset-recovery (non-safety) ================ */

/* Read-only SPI liveness probe. Reads PHYCFGR over SPI via the public ioctl;
 * a wedged/timed-out bus returns != ESP_OK. Serialised against the driver's own
 * link-timer reads by the SPI-master device lock, so it is safe to call
 * concurrently. Returns true (healthy) when no driver is installed — an absent
 * driver is "nothing to probe", not a fault. */
static bool eth_spi_probe_ok(void)
{
  esp_eth_handle_t h = s_eth_handle;
  if (h == NULL) {
    return true;
  }
  uint32_t val = 0u;
  esp_eth_phy_reg_rw_data_t rd = {.reg_addr = DCS_W5500_REG_PHYCFGR, .reg_value_p = &val};
  return esp_eth_ioctl(h, ETH_CMD_READ_PHY_REG, &rd) == ESP_OK;
}

/* Detector B — a black-holing but still-"UP" Ethernet link. True only when the
 * driver claims link UP with a valid lease, Ethernet is the active default route,
 * pstop has bonded at least once yet no peer has replied for DCS_ETHWD_SILENCE_MS,
 * AND the uplink gateway is also silent (or never answered ICMP). If the gateway
 * still answers, our uplink is fine and the silence is a peer/path problem — not a
 * W5500 wedge — so we return false (mirrors the liveness watchdog's cross-check). */
static bool eth_blackhole_suspected(void)
{
  if (!atomic_load(&s_link_up) || !atomic_load(&s_has_ip)) {
    return false; /* link/lease already gone — the supervisor handles that path */
  }
  if ((dcs_iface_t)atomic_load(&g_dcs_active_iface) != DCS_IFACE_ETH) {
    return false; /* Ethernet isn't carrying traffic right now */
  }
  uint64_t ps_last = atomic_load(&g_dcs_pstop_last_reply_ms);
  if (ps_last == 0u) {
    return false; /* pstop never bonded — silence is not evidence of a wedge */
  }
  uint64_t now_ms = (uint64_t)esp_timer_get_time() / 1000u;
  uint64_t ps_silent = (now_ms > ps_last) ? (now_ms - ps_last) : 0u;
  if (ps_silent < (uint64_t)DCS_ETHWD_SILENCE_MS) {
    return false; /* a peer replied recently — traffic is flowing */
  }
  uint32_t gw_replies = 0u;
  dcs_net_liveness_stats(NULL, NULL, &gw_replies, NULL);
  if ((gw_replies > 0u) && (dcs_net_liveness_gw_silent_ms() < DCS_ETHWD_SILENCE_MS)) {
    return false; /* gateway is answering — uplink is fine, don't reset the W5500 */
  }
  return true;
}

/* Immediately reflect "Ethernet suspect/down" to link-state consumers
 * (telemetry, boot gating). The supervisor's own demotion happens when the
 * recovery ladder brings the netif down via esp_eth_stop(). */
static void eth_force_demote(void)
{
  atomic_store(&s_link_up, false);
  atomic_store(&s_has_ip, false);
}

/* Datasheet-correct hardware reset pulse on GPIO9 (RSTn, active-low). Only used
 * on the last recovery rung, always while the driver is torn down (so the pin
 * isn't owned by the esp_eth phy). Assert >= T_RC 500 us, release, wait >= T_PL
 * 1 ms for PLL lock — both with margin. */
static void eth_hw_reset_pulse(void)
{
  gpio_config_t io = {
    .pin_bit_mask = (1ULL << DCS_ETH_PIN_RST),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  (void)gpio_config(&io);
  gpio_set_level(DCS_ETH_PIN_RST, 0); /* assert RSTn */
  esp_rom_delay_us(DCS_ETHWD_RST_ASSERT_US); /* >= 500 us (T_RC) */
  gpio_set_level(DCS_ETH_PIN_RST, 1); /* release RSTn */
  vTaskDelay(pdMS_TO_TICKS(DCS_ETHWD_RST_PLL_MS)); /* >= 1 ms (T_PL) PLL lock */
  ESP_LOGW(TAG, "W5500 hardware RST pulse issued on GPIO%d", DCS_ETH_PIN_RST);
}

/* Full teardown of everything dcs_eth_start() created, in reverse order, leaving
 * the SPI bus + GPIO ISR service (both shared/idempotent) alone so a subsequent
 * dcs_eth_start() re-inits them via their INVALID_STATE fast-path. Mirrors the
 * bring-up's fail ladder. Only frees s_mac/s_phy, which are non-NULL solely after
 * a successful bring-up — so this never double-frees. */
static void eth_teardown(void)
{
  atomic_store(&s_enabled, false);
  if (s_eth_handle != NULL) {
    (void)esp_eth_stop(s_eth_handle);
  }
  (void)esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, on_eth_event);
  (void)esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP, on_eth_got_ip);
  if (s_eth_netif != NULL) {
    esp_netif_destroy(s_eth_netif);
    s_eth_netif = NULL;
  }
  if (s_eth_handle != NULL) {
    (void)esp_eth_driver_uninstall(s_eth_handle);
    s_eth_handle = NULL;
  }
  if (s_phy != NULL) {
    (void)s_phy->del(s_phy);
    s_phy = NULL;
  }
  if (s_mac != NULL) {
    (void)s_mac->del(s_mac);
    s_mac = NULL;
  }
  if (s_got_ip_sem != NULL) {
    vSemaphoreDelete(s_got_ip_sem);
    s_got_ip_sem = NULL;
  }
  atomic_store(&s_link_up, false);
  atomic_store(&s_has_ip, false);
}

/* Wait up to DCS_ETHWD_RUNG_TIMEOUT_MS for a recovery rung to restore the link:
 * a fresh CONNECTED event (s_link_up) AND an SPI probe pass. Bounded loop. */
static bool eth_wait_restored(void)
{
  uint64_t deadline_ms = (uint64_t)esp_timer_get_time() / 1000u + (uint64_t)DCS_ETHWD_RUNG_TIMEOUT_MS;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(250));
    if (atomic_load(&s_link_up) && eth_spi_probe_ok()) {
      return true;
    }
    if (((uint64_t)esp_timer_get_time() / 1000u) >= deadline_ms) {
      return false;
    }
  }
}

/* Walk the escalating recovery ladder under the lifecycle lock. Returns true if
 * some rung restored the link. Per-rung success counts feed /state.json. */
static bool eth_run_recovery(void)
{
  bool restored = false;
  (void)xSemaphoreTake(s_eth_lock, portMAX_DELAY);

  eth_force_demote(); /* consumers see Ethernet down at once */

  /* Rung 1 — stop/start: cheapest, clears a stuck FSM / link timer. */
  if (s_eth_handle != NULL) {
    (void)esp_eth_stop(s_eth_handle);
    vTaskDelay(pdMS_TO_TICKS(50));
    if ((esp_eth_start(s_eth_handle) == ESP_OK) && eth_wait_restored()) {
      (void)atomic_fetch_add(&g_dcs_eth_rec_r1, 1u);
      restored = true;
    }
  }

  /* Rung 2 — full driver+netif rebuild (re-inits the chip over SPI). */
  if (!restored) {
    ESP_LOGW(TAG, "W5500 recovery: rung 2 (driver reinstall)");
    eth_teardown();
    if ((dcs_eth_start() == ESP_OK) && eth_wait_restored()) {
      (void)atomic_fetch_add(&g_dcs_eth_rec_r2, 1u);
      restored = true;
    }
  }

  /* Rung 3 — datasheet-correct HW RST pulse, then rebuild. */
  if (!restored) {
    ESP_LOGW(TAG, "W5500 recovery: rung 3 (hardware RST pulse + reinstall)");
    eth_teardown();
    eth_hw_reset_pulse();
    if ((dcs_eth_start() == ESP_OK) && eth_wait_restored()) {
      (void)atomic_fetch_add(&g_dcs_eth_rec_r3, 1u);
      restored = true;
    }
  }

  (void)xSemaphoreGive(s_eth_lock);
  return restored;
}

static void eth_watchdog_task(void * arg)
{
  (void)arg;
  uint32_t suspect_ticks = 0u;
  uint64_t cooldown_until_ms = 0u;
  ESP_LOGI(
    TAG,
    "W5500 health watchdog: probe+black-hole detect, confirm=%u ticks, silence=%ums, "
    "bounded reset ladder, cooldown=%ums",
    (unsigned)DCS_ETHWD_CONFIRM_TICKS,
    (unsigned)DCS_ETHWD_SILENCE_MS,
    (unsigned)DCS_ETHWD_COOLDOWN_MS);

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(DCS_ETHWD_POLL_MS));

    if ((s_eth_handle == NULL) || !atomic_load(&s_enabled)) {
      suspect_ticks = 0u; /* driver down (boot / admin-disabled) — nothing to guard */
      continue;
    }
    uint64_t now_ms = (uint64_t)esp_timer_get_time() / 1000u;
    if (now_ms < cooldown_until_ms) {
      suspect_ticks = 0u; /* post-ladder hold-off */
      continue;
    }

    uint32_t reason = DCS_ETHWD_REASON_NONE;
    if (!eth_spi_probe_ok()) {
      reason = DCS_ETHWD_REASON_SPI;
      (void)atomic_fetch_add(&g_dcs_eth_spi_err, 1u);
    } else if (eth_blackhole_suspected()) {
      reason = DCS_ETHWD_REASON_BLACKHOLE;
    } else {
      /* healthy */
    }

    if (reason == DCS_ETHWD_REASON_NONE) {
      suspect_ticks = 0u;
      continue;
    }

    suspect_ticks++;
    if (suspect_ticks < DCS_ETHWD_CONFIRM_TICKS) {
      continue; /* require persistence — no false trips on a transient glitch */
    }

    ESP_LOGE(
      TAG,
      "W5500 wedge CONFIRMED (reason=%s) — demoting Ethernet and running reset ladder",
      (reason == DCS_ETHWD_REASON_SPI) ? "spi-probe" : "black-hole");
    atomic_store(&g_dcs_eth_rec_reason, reason);

    bool restored = eth_run_recovery();

    suspect_ticks = 0u;
    cooldown_until_ms = ((uint64_t)esp_timer_get_time() / 1000u) + (uint64_t)DCS_ETHWD_COOLDOWN_MS;
    if (restored) {
      (void)atomic_fetch_add(&g_dcs_eth_recoveries, 1u);
      ESP_LOGW(TAG, "W5500 recovery SUCCEEDED — Ethernet restored");
    } else {
      ESP_LOGE(
        TAG,
        "W5500 recovery ladder EXHAUSTED — Ethernet stays demoted (USB-NCM/WiFi "
        "carries pstop; fail-safe intact); cooldown %us before re-arming",
        (unsigned)(DCS_ETHWD_COOLDOWN_MS / 1000u));
    }
  }
}

/* === Public-ish (component-internal) API ================================== */

esp_err_t dcs_eth_start(void)
{
  if (s_eth_netif != NULL) {
    ESP_LOGW(TAG, "already started");
    return ESP_OK;
  }

  /* Lifecycle lock, created once on the first bring-up (before any concurrent
   * admin toggle or watchdog recovery can run). */
  if (s_eth_lock == NULL) {
    s_eth_lock = xSemaphoreCreateMutex();
    if (s_eth_lock == NULL) {
      return ESP_ERR_NO_MEM;
    }
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
    /* Declare the MISO round-trip so the SPI driver places the sampling edge
     * with margin against trace/temperature delay (ESP-IDF SPI-Ethernet default).
     * At 20 MHz the freq limit is 80/(floor(20/12.5)+1) = 40 MHz, well clear —
     * pure signal-integrity margin, no behavioural change. */
    .input_delay_ns = 20,
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

  /* Publish the MAC/PHY objects for the watchdog's reinstall path, and spawn the
   * health watchdog exactly once. Recovery rungs 2/3 re-enter dcs_eth_start(),
   * so the s_wd_started guard keeps this to a single task. */
  s_mac = mac;
  s_phy = phy;
  if (!s_wd_started) {
    s_wd_started = true;
    /* PSRAM stack, non-safety: the watchdog does esp_eth ioctl/stop/start and,
     * on recovery, a full driver+netif rebuild (no flash/NVS, no ISR access) —
     * safe on an external-RAM stack. Prio 3 < supervisor(4), well below safety. */
    (void)dcs_task_spawn_psram(eth_watchdog_task, "eth_wd", 6144, NULL, 3, tskNO_AFFINITY);
  }

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
  /* Serialise against the watchdog's recovery ladder so an admin toggle can't
   * race a stop/start/reinstall in flight. */
  if (s_eth_lock != NULL) {
    (void)xSemaphoreTake(s_eth_lock, portMAX_DELAY);
  }
  if (on == atomic_load(&s_enabled)) {
    if (s_eth_lock != NULL) {
      (void)xSemaphoreGive(s_eth_lock);
    }
    return ESP_OK;
  }
  esp_err_t err = on ? esp_eth_start(s_eth_handle) : esp_eth_stop(s_eth_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "eth %s: %s", on ? "start" : "stop", esp_err_to_name(err));
    if (s_eth_lock != NULL) {
      (void)xSemaphoreGive(s_eth_lock);
    }
    return err;
  }
  if (!on) {
    atomic_store(&s_link_up, false);
    atomic_store(&s_has_ip, false);
  }
  atomic_store(&s_enabled, on);
  ESP_LOGW(TAG, "Ethernet %s via admin toggle", on ? "ENABLED" : "DISABLED");
  if (s_eth_lock != NULL) {
    (void)xSemaphoreGive(s_eth_lock);
  }
  return ESP_OK;
}
