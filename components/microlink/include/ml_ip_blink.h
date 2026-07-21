// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ml_ip_blink.h
 * @brief Blink the last octet of the chip's IPv4 address on an LED
 *
 * Displays each digit as blinks with pauses between digits.
 * For IP x.x.x.133: 1 blink, pause, 3 blinks, pause, 3 blinks, long pause, repeat.
 * Zero digit = one long blink (600ms vs 200ms for normal blinks).
 *
 * Picks whichever interface currently has an IPv4 lease (WiFi STA, USB-NCM
 * tether, AP, etc), preferring a "local" address over Tailscale (100.64/10
 * CGNAT). If no interface has an IP yet, the task quietly waits and retries
 * every 2 s — so it's safe to start the task at boot before networking is up.
 * Also re-reads the IP each cycle, so DHCP renewals or mode switches are
 * picked up without a reboot.
 *
 * Runs as a background FreeRTOS task. Call ml_ip_blink_start() once at boot.
 */

#pragma once

#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef ML_BLINK_GPIO
  #define ML_BLINK_GPIO 21
#endif

#define ML_BLINK_ON_MS 200
#define ML_BLINK_OFF_MS 200
#define ML_BLINK_ZERO_MS 600 /* long blink for zero digit */
#define ML_BLINK_DIGIT_GAP 600 /* pause between digits */
#define ML_BLINK_CYCLE_GAP 5000 /* pause between full cycles */
#define ML_BLINK_WAIT_MS 2000 /* poll interval while waiting for IP */

/* Active-low LED: 0 = ON, 1 = OFF */
#define LED_ON 0
#define LED_OFF 1

/* Pick the last octet of whichever netif currently has the most useful IPv4.
 * Returns 0 if nothing usable is up. A non-zero CGNAT (Tailscale 100.64/10)
 * address is used only as a fallback when no local address is available. */
static inline uint8_t ml_ip_blink_pick_octet(void)
{
  uint8_t fallback = 0; /* Tailscale CGNAT as last resort */

  /* esp_netif_next walks the registered netifs; passing NULL gets the first. */
  esp_netif_t * n = esp_netif_next_unsafe(NULL);
  while (n) {
    esp_netif_ip_info_t info;
    if (esp_netif_get_ip_info(n, &info) == ESP_OK && info.ip.addr != 0) {
      /* esp_ip4_addr_t.addr is stored in network byte order; on the
             * little-endian S3 that means byte 0 of the integer is the
             * first dotted octet (e.g. 10.42.0.106 -> 0x6A002A0A). */
      uint32_t addr = info.ip.addr;
      uint8_t first = (uint8_t)(addr & 0xFF);
      uint8_t second = (uint8_t)((addr >> 8) & 0xFF);
      uint8_t last = (uint8_t)((addr >> 24) & 0xFF);

      if (first == 127) {
        /* loopback — skip */
      } else if (first == 169 && second == 254) {
        /* link-local 169.254/16 — skip, no real connectivity */
      } else if (first == 100 && (second & 0xC0) == 0x40) {
        /* Tailscale CGNAT 100.64.0.0/10 — keep as fallback only */
        if (fallback == 0) fallback = last;
      } else {
        /* Preferred local IPv4 — return immediately */
        return last;
      }
    }
    n = esp_netif_next_unsafe(n);
  }
  return fallback;
}

static void ml_ip_blink_task(void * arg)
{
  int gpio = (int)(intptr_t)arg;

  gpio_config_t io = {
    .pin_bit_mask = 1ULL << gpio,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
  };
  gpio_config(&io);
  gpio_set_level(gpio, LED_OFF);

  uint8_t last_logged = 0;

  while (1) {
    uint8_t last_octet = ml_ip_blink_pick_octet();

    if (last_octet == 0) {
      /* No usable IP yet — wait and try again. */
      if (last_logged != 0) {
        ESP_LOGW("ip_blink", "Lost IP — waiting");
        last_logged = 0;
      }
      gpio_set_level(gpio, LED_OFF);
      vTaskDelay(pdMS_TO_TICKS(ML_BLINK_WAIT_MS));
      continue;
    }

    if (last_octet != last_logged) {
      ESP_LOGI("ip_blink", "Blinking last octet: %d (GPIO %d, active-low)", last_octet, gpio);
      last_logged = last_octet;
    }

    /* Extract digits */
    int digits[3];
    digits[0] = last_octet / 100;
    digits[1] = (last_octet / 10) % 10;
    digits[2] = last_octet % 10;

    /* Skip leading zeros (e.g. for IP ending in .5, show just "5") */
    int start = 0;
    if (digits[0] == 0) start = 1;
    if (digits[0] == 0 && digits[1] == 0) start = 2;

    for (int d = start; d < 3; d++) {
      if (d > start) {
        /* Gap between digits — LED off */
        vTaskDelay(pdMS_TO_TICKS(ML_BLINK_DIGIT_GAP));
      }

      int count = digits[d];
      if (count == 0) {
        /* Zero = one long blink */
        gpio_set_level(gpio, LED_ON);
        vTaskDelay(pdMS_TO_TICKS(ML_BLINK_ZERO_MS));
        gpio_set_level(gpio, LED_OFF);
      } else {
        for (int i = 0; i < count; i++) {
          gpio_set_level(gpio, LED_ON);
          vTaskDelay(pdMS_TO_TICKS(ML_BLINK_ON_MS));
          gpio_set_level(gpio, LED_OFF);
          if (i < count - 1) {
            vTaskDelay(pdMS_TO_TICKS(ML_BLINK_OFF_MS));
          }
        }
      }
    }

    /* LED off, long dark pause between cycles. IP is re-checked at the
         * top of the next iteration so DHCP changes are picked up live. */
    gpio_set_level(gpio, LED_OFF);
    vTaskDelay(pdMS_TO_TICKS(ML_BLINK_CYCLE_GAP));
  }
}

/**
 * @brief Start blinking the chip's IPv4 last octet on the given GPIO
 * @param gpio LED GPIO number (default: 21 for XIAO ESP32-S3)
 *
 * Safe to call at any time during boot. The task self-paces and waits for
 * a usable IP if networking isn't ready yet.
 */
static inline void ml_ip_blink_start(int gpio)
{
  xTaskCreate(ml_ip_blink_task, "ip_blink", 4096, (void *)(intptr_t)gpio, 2, NULL);
}
