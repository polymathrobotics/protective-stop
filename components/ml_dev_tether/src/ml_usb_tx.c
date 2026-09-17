// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
//
// Bounded, in-order TX FIFO for the USB-NCM tether. See ml_usb_tx.h.
//
// Concurrency model: ONE producer (lwIP's tcpip thread via netif_transmit) and
// ONE consumer (the esp_timer task running usb_drain). head is written only by
// the consumer, tail only by the producer; both are atomics, so the ring needs
// no lock. Each slot's frame is fully written before tail is published
// (release), and the consumer reads tail with acquire before touching a slot.
#include "ml_usb_tx.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "tinyusb_net.h"
#include "tusb.h"

#define TX_SLOTS 16u /* power of two: indices wrap safely */
#define TX_FRAME_MAX 1536u /* Ethernet MTU + link-layer headers */
#define TX_TTL_US 100000 /* same 100 ms lifetime the old sync send had */
#define TX_RETRY_US 2000 /* drain cadence while the endpoint is busy */

typedef struct
{
  uint8_t * bytes;
  uint16_t len;
  int64_t queued_us;
} tx_slot_t;

static tx_slot_t s_slots[TX_SLOTS];
static atomic_uint s_head, s_tail; /* consumer / producer indices, free-running */
static atomic_bool s_ready, s_enabled;
static esp_timer_handle_t s_drain_timer;
static atomic_uint s_sent, s_busy_retries, s_expired, s_full_drops;

/* Consumer: offer the head frame; keep it on NCM-busy; drop it on expiry. */
static void usb_drain(void * arg)
{
  (void)arg;
  for (;;) {
    unsigned head = atomic_load_explicit(&s_head, memory_order_relaxed);
    if (head == atomic_load_explicit(&s_tail, memory_order_acquire)) {
      return; /* empty */
    }
    tx_slot_t * s = &s_slots[head % TX_SLOTS];
    if (!atomic_load(&s_enabled)) {
      /* Disabled mid-flight: discard silently (stale netif). */
    } else if (esp_timer_get_time() - s->queued_us >= TX_TTL_US) {
      atomic_fetch_add(&s_expired, 1u);
    } else if (!tud_mounted()) {
      atomic_fetch_add(&s_expired, 1u); /* host gone: same fate, counted the same */
    } else {
      /* Zero-tick offer: esp_tinyusb runs can_xmit+xmit on the TinyUSB task
       * and reports busy as ESP_FAIL (or ESP_ERR_TIMEOUT if the USB event
       * queue itself was full). Either way OUR copy is intact: retry. */
      esp_err_t r = tinyusb_net_send_sync(s->bytes, s->len, NULL, 0);
      if (r != ESP_OK) {
        atomic_fetch_add(&s_busy_retries, 1u);
        return; /* head stays; next timer tick retries in order */
      }
      atomic_fetch_add(&s_sent, 1u);
    }
    atomic_store_explicit(&s_head, head + 1u, memory_order_release);
  }
}

esp_err_t ml_usb_tx_init(void)
{
  if (atomic_load(&s_ready)) {
    return ESP_OK;
  }
  uint8_t * pool = heap_caps_malloc(TX_SLOTS * TX_FRAME_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (pool == NULL) {
    return ESP_ERR_NO_MEM;
  }
  for (unsigned i = 0; i < TX_SLOTS; i++) {
    s_slots[i].bytes = pool + (i * TX_FRAME_MAX);
  }
  const esp_timer_create_args_t args = {
    .callback = usb_drain,
    .dispatch_method = ESP_TIMER_TASK, /* tinyusb_net_send_sync blocks on an event group */
    .name = "usb_tx",
    .skip_unhandled_events = true,
  };
  esp_err_t r = esp_timer_create(&args, &s_drain_timer);
  if (r == ESP_OK) {
    r = esp_timer_start_periodic(s_drain_timer, TX_RETRY_US);
  }
  if (r != ESP_OK) {
    if (s_drain_timer != NULL) {
      (void)esp_timer_delete(s_drain_timer);
      s_drain_timer = NULL;
    }
    heap_caps_free(pool);
    return r;
  }
  atomic_store(&s_ready, true);
  return ESP_OK;
}

void ml_usb_tx_set_enabled(int enabled)
{
  atomic_store(&s_enabled, enabled != 0);
  /* On disable the drain discards whatever is queued (see usb_drain): the
   * frames belong to a netif that is being torn down. */
}

esp_err_t ml_usb_tx_send(const void * buffer, size_t len)
{
  if (!atomic_load(&s_ready) || !atomic_load(&s_enabled) || !tud_mounted()) {
    return ESP_ERR_INVALID_STATE;
  }
  if (buffer == NULL || len == 0 || len > TX_FRAME_MAX) {
    return ESP_ERR_INVALID_ARG;
  }
  unsigned tail = atomic_load_explicit(&s_tail, memory_order_relaxed);
  if (tail - atomic_load_explicit(&s_head, memory_order_acquire) >= TX_SLOTS) {
    atomic_fetch_add(&s_full_drops, 1u);
    return ESP_ERR_NO_MEM; /* lwIP sees a link-layer drop; TCP retransmits */
  }
  tx_slot_t * s = &s_slots[tail % TX_SLOTS];
  memcpy(s->bytes, buffer, len);
  s->len = (uint16_t)len;
  s->queued_us = esp_timer_get_time();
  atomic_store_explicit(&s_tail, tail + 1u, memory_order_release);
  return ESP_OK;
}

void ml_usb_tx_get_diag(ml_usb_tx_diag_t * out)
{
  if (out == NULL) {
    return;
  }
  out->sent = atomic_load(&s_sent);
  out->busy_retries = atomic_load(&s_busy_retries);
  out->expired = atomic_load(&s_expired);
  out->full_drops = atomic_load(&s_full_drops);
  out->pending = atomic_load(&s_tail) - atomic_load(&s_head);
}
