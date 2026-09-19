// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
//
// Bounded, in-order TX FIFO for the USB-NCM tether. See ml_usb_tx.h.
//
// Concurrency model: ONE producer (lwIP's tcpip thread via netif_transmit) and
// ONE consumer (the usb_tx drain task). head is written ONLY by the consumer,
// tail ONLY by the producer; both are atomics, so the ring needs no lock. Each
// slot's frame is fully written before tail is published (release), and the
// consumer reads tail with acquire before touching a slot.
// Disable does not touch head or tail from a third context: it bumps a session
// epoch and wakes the consumer, which discards every frame stamped with an
// older epoch — so nothing queued for a torn-down netif can be sent after a
// quick re-enable, and s_head keeps its single writer.
#include "ml_usb_tx.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "tinyusb_net.h"
#include "tusb.h"

#define TX_SLOTS \
  64u /* power of two: indices wrap safely. 16 lost ~1 safety frame/min on a
       * remote behind a dock hub chain (bench 2026-09-19: 252 ring-full drops
       * in 4 h): the disco probe engine emits bursts of >16 frames toward the
       * tailnet peers, and any pstop frame caught in the burst was dropped
       * while the endpoint drained. 64 x 1536 B = 96 KB of PSRAM. */
#define TX_FRAME_MAX 1536u /* Ethernet MTU + link-layer headers */
#define TX_TTL_US \
  400000 /* a frame older than this is expired, not sent. 100 ms (the old sync
          * send's lifetime) expired frames during ordinary host scheduling
          * hiccups; 400 ms is one heartbeat period, still well inside the
          * machine's 1.6 s stop-on-silence budget and the machine rejects
          * anything that arrives out of order anyway. */
#define TX_RETRY_MS 2 /* first retry delay while the endpoint is busy... */
#define TX_RETRY_MAX_MS \
  32 /* ...doubling to this cap: before the host has configured NCM every
                            * offer is refused, and a flat 2 ms poll deferred ~800 no-op calls
                            * onto the TinyUSB task per 16 frames (bench, 2026-09-17) */
#define TX_TASK_STACK 4096 /* tinyusb_net_send_sync: event group + semaphore + logging */
#define TX_TASK_PRIO 5 /* below the safety tasks; above idle/lwIP housekeeping */

typedef struct
{
  uint8_t * bytes;
  uint16_t len;
  int64_t queued_us;
  unsigned epoch; /* tether session this frame belongs to */
} tx_slot_t;

static tx_slot_t s_slots[TX_SLOTS];
static atomic_uint s_head, s_tail; /* consumer / producer indices, free-running */
static atomic_uint s_epoch; /* bumped on every disable; odd = disabled, even = enabled */
static atomic_bool s_ready, s_init_started, s_producer_busy;
static TaskHandle_t s_drain_task;
static atomic_uint s_sent, s_busy_retries, s_expired, s_full_drops;

/* Consumer: offer the head frame; keep it on NCM-busy; drop it on expiry.
 * Returns true when the ring still holds a frame the endpoint refused. */
static bool usb_drain(void)
{
  for (;;) {
    unsigned head = atomic_load_explicit(&s_head, memory_order_relaxed);
    if (head == atomic_load_explicit(&s_tail, memory_order_acquire)) {
      return false; /* empty */
    }
    tx_slot_t * s = &s_slots[head % TX_SLOTS];
    unsigned epoch = atomic_load_explicit(&s_epoch, memory_order_acquire);
    if ((epoch & 1u) || s->epoch != epoch) {
      atomic_fetch_add(&s_expired, 1u); /* disabled, or queued for a previous session: counted, not sent */
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
        return true; /* head stays; retried in order after TX_RETRY_MS */
      }
      atomic_fetch_add(&s_sent, 1u);
    }
    atomic_store_explicit(&s_head, head + 1u, memory_order_release);
  }
}

/* Dedicated drain task, NOT the shared esp_timer task: tinyusb_net_send_sync()
 * blocks on an event group from a call chain several frames deep, and the
 * esp_timer task's 3.5 KB stack is shared with every other timer callback on
 * the device. Running the drain there crash-looped the Ethernet DUT
 * (w5500_tsk SPI descriptor corruption, 4 panics -> rollback, 2026-09-17).
 * Blocks on a task notification when idle; the producer wakes it. */
static void usb_drain_task(void * arg)
{
  (void)arg;
  uint32_t retry_ms = TX_RETRY_MS;
  for (;;) {
    if (usb_drain()) {
      vTaskDelay(pdMS_TO_TICKS(retry_ms)); /* endpoint busy: poll it, backing off */
      if (retry_ms < TX_RETRY_MAX_MS) retry_ms *= 2u;
    } else {
      retry_ms = TX_RETRY_MS; /* drained (or empty): next burst starts fast again */
      (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* empty: sleep until a frame arrives */
    }
  }
}

esp_err_t ml_usb_tx_init(void)
{
  if (atomic_load(&s_ready)) {
    return ESP_OK;
  }
  /* Single-flight: two concurrent starts (admin toggle racing the net
   * supervisor) must not both allocate a pool and spawn a task. The loser
   * reports busy; its caller's start fails and retries later. */
  if (atomic_exchange(&s_init_started, true)) {
    return atomic_load(&s_ready) ? ESP_OK : ESP_ERR_INVALID_STATE;
  }
  atomic_store(&s_epoch, 1u); /* starts disabled */
  uint8_t * pool = heap_caps_malloc(TX_SLOTS * TX_FRAME_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (pool == NULL) {
    atomic_store(&s_init_started, false);
    return ESP_ERR_NO_MEM;
  }
  for (unsigned i = 0; i < TX_SLOTS; i++) {
    s_slots[i].bytes = pool + (i * TX_FRAME_MAX);
  }
  /* Stack + TCB in PSRAM, like the ring: the tether is brought up by the net
   * supervisor on any Ethernet blip and stays resident, so this must add no
   * internal-DRAM allocation. An internal 4 KB stack here crash-looped the
   * Ethernet DUT (2026-09-17): it landed next to the W5500 emac and exposed a
   * latent overrun into emac->rx_buffer (LoadProhibited in the SPI DMA
   * teardown, w5500_tsk). Same task with a PSRAM stack: 290 s clean. The
   * task only blocks and polls; it never touches DMA. */
  if (
    xTaskCreateWithCaps(
      usb_drain_task,
      "usb_tx",
      TX_TASK_STACK,
      NULL,
      TX_TASK_PRIO,
      &s_drain_task,
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS)
  {
    heap_caps_free(pool);
    atomic_store(&s_init_started, false);
    return ESP_ERR_NO_MEM;
  }
  atomic_store(&s_ready, true);
  return ESP_OK;
}

void ml_usb_tx_set_enabled(int enabled)
{
  /* Epoch parity carries the enabled state: even = enabled, odd = disabled.
   * Every transition bumps it, so a frame stamped under an earlier session can
   * never match again. The consumer is the only s_head writer; wake it so the
   * discard happens now rather than at the next producer wake. */
  unsigned e = atomic_load(&s_epoch);
  bool now_enabled = (e & 1u) == 0u;
  if ((enabled != 0) == now_enabled) {
    return;
  }
  atomic_fetch_add_explicit(&s_epoch, 1u, memory_order_acq_rel);
  if (s_drain_task != NULL) {
    xTaskNotifyGive(s_drain_task);
  }
}

esp_err_t ml_usb_tx_send(const void * buffer, size_t len)
{
  unsigned epoch = atomic_load_explicit(&s_epoch, memory_order_acquire);
  if (!atomic_load(&s_ready) || (epoch & 1u) || !tud_mounted()) {
    return ESP_ERR_INVALID_STATE;
  }
  if (buffer == NULL || len == 0 || len > TX_FRAME_MAX) {
    return ESP_ERR_INVALID_ARG;
  }
  /* Single producer by construction (lwIP's tcpip thread is the only caller
   * of netif_transmit). Guard the invariant so a second caller fails loudly
   * with a drop instead of two writers racing on one slot. */
  if (atomic_exchange(&s_producer_busy, true)) {
    atomic_fetch_add(&s_full_drops, 1u);
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t r = ESP_OK;
  unsigned tail = atomic_load_explicit(&s_tail, memory_order_relaxed);
  if (tail - atomic_load_explicit(&s_head, memory_order_acquire) >= TX_SLOTS) {
    atomic_fetch_add(&s_full_drops, 1u);
    r = ESP_ERR_NO_MEM; /* lwIP sees a link-layer drop; TCP retransmits */
  } else {
    tx_slot_t * s = &s_slots[tail % TX_SLOTS];
    memcpy(s->bytes, buffer, len);
    s->len = (uint16_t)len;
    s->queued_us = esp_timer_get_time();
    s->epoch = epoch; /* a disable racing this publish flips parity; the consumer then discards it */
    atomic_store_explicit(&s_tail, tail + 1u, memory_order_release);
    xTaskNotifyGive(s_drain_task);
  }
  atomic_store(&s_producer_busy, false);
  return r;
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
  /* Two independent atomics: read head FIRST so a producer racing in between
   * can only make the difference larger by real frames, never wrap negative
   * (tail read first + a concurrent retire could yield ~4e9). */
  unsigned head = atomic_load(&s_head);
  unsigned tail = atomic_load(&s_tail);
  out->pending = (tail - head <= TX_SLOTS) ? (tail - head) : TX_SLOTS;
}
