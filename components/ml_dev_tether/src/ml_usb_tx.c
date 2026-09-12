// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

#include "ml_usb_tx.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <string.h>

#include "ml_usb_defer.h"
#include "ml_usb_tx_limits.h"

#ifdef ML_USB_TX_HOST_TEST
  #include "usb_tx_test_platform.h"
#else
  #include "esp_heap_caps.h"
  #include "esp_timer.h"
  #include "tusb.h"
#endif

#define TX_SLOTS 16
#define TX_DEFER_CAP ML_USB_TX_DEFER_CAP
#define TX_FRAME_MAX 1536 /* Ethernet MTU plus link-layer headers. */
#define TX_TTL_US 100000

_Static_assert(TX_SLOTS >= TX_DEFER_CAP, "The owned pool must cover the admitted USB closures");

enum tx_state
{
  TX_FREE,
  TX_FILLING,
  TX_PENDING,
  TX_RUNNING
};

typedef struct
{
  atomic_uint state;
  atomic_uint queued_ms; /* advisory cross-task backlog telemetry */
  uint8_t * bytes;
  uint16_t len;
  uint32_t epoch;
  int64_t queued_us;
  bool copied;
} tx_request_t;

static tx_request_t s_requests[TX_SLOTS];
static atomic_bool s_ready;
static atomic_bool s_enabled;
static atomic_bool s_caller_busy;
static atomic_uint s_epoch;
static atomic_uint s_submitted, s_copied, s_cancelled, s_errors;
static atomic_uint s_rejected_unavailable, s_unmounted_drop;
static atomic_uint s_exhausted, s_defer_full, s_can_xmit_fail, s_integrity_errors;
static atomic_uint s_pending, s_defer_cap, s_last_callback_ms, s_callbacks;
static atomic_uint s_latency[4];

static void request_release(tx_request_t * r)
{
  /* Last request access: a producer may immediately reuse the slot. There is
   * exactly one callback for each successful defer; failed defers enqueue none. */
  atomic_store_explicit(&r->state, TX_FREE, memory_order_release);
}

static bool request_expired(tx_request_t * r, int64_t now)
{
  return !atomic_load(&s_enabled) || r->epoch != atomic_load(&s_epoch) || now < r->queued_us ||
         now - r->queued_us >= TX_TTL_US;
}

/* Only our fixed-pool references take this path. Every legacy vendor ref is
 * forwarded, preserving its callback/free-buffer contract without packet_t ABI
 * assumptions. TinyUSB calls this synchronously while appending to an NCM NTB. */
uint16_t __real_tud_network_xmit_cb(uint8_t * dst, void * ref, uint16_t arg);

uint16_t __wrap_tud_network_xmit_cb(uint8_t * dst, void * ref, uint16_t arg)
{
  uintptr_t address = (uintptr_t)ref;
  uintptr_t first = (uintptr_t)s_requests;
  if (address < first || address - first >= sizeof(s_requests) || (address - first) % sizeof(tx_request_t) != 0) {
    return __real_tud_network_xmit_cb(dst, ref, arg);
  }
  tx_request_t * r = ref;
  if (atomic_load(&r->state) != TX_RUNNING || r->copied || arg != r->len) {
    atomic_fetch_add(&s_integrity_errors, 1);
    return 0;
  }
  memcpy(dst, r->bytes, r->len);
  r->copied = true;
  atomic_fetch_add(&s_copied, 1);
  return r->len;
}

static void usb_send(void * context)
{
  tx_request_t * r = context;
  atomic_fetch_sub(&s_pending, 1); /* this closure has left the shared USB queue */
  atomic_store(&s_last_callback_ms, (uint32_t)(esp_timer_get_time() / 1000));
  atomic_fetch_add(&s_callbacks, 1);
  if (atomic_load_explicit(&r->state, memory_order_acquire) != TX_PENDING) {
    atomic_fetch_add(&s_integrity_errors, 1);
    return;
  }
  int64_t now = esp_timer_get_time();
  int64_t latency = now - r->queued_us;
  unsigned bucket = latency < 10000 ? 0u : latency < 50000 ? 1u : latency < 100000 ? 2u : 3u;
  atomic_fetch_add(&s_latency[bucket], 1);
  if (request_expired(r, now)) {
    atomic_fetch_add(&s_cancelled, 1);
    request_release(r);
    return;
  }
  if (!tud_mounted()) {
    atomic_fetch_add(&s_unmounted_drop, 1);
    request_release(r);
    return;
  }
  if (!tud_network_can_xmit(r->len)) {
    atomic_fetch_add(&s_can_xmit_fail, 1);
    request_release(r);
    return;
  }
  if (request_expired(r, esp_timer_get_time())) {
    atomic_fetch_add(&s_cancelled, 1);
    request_release(r);
    return;
  }
  atomic_store(&r->state, TX_RUNNING);
  tud_network_xmit(r, r->len);
  if (!r->copied) atomic_fetch_add(&s_errors, 1);
  request_release(r);
}

esp_err_t ml_usb_tx_init(void)
{
  if (atomic_load_explicit(&s_ready, memory_order_acquire)) return ESP_OK;
  uint8_t * payloads = heap_caps_malloc(TX_SLOTS * TX_FRAME_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!payloads) return ESP_ERR_NO_MEM;
  for (size_t i = 0; i < TX_SLOTS; i++) s_requests[i].bytes = payloads + i * TX_FRAME_MAX;
  atomic_store_explicit(&s_ready, true, memory_order_release);
  return ESP_OK;
}

void ml_usb_tx_set_enabled(int enabled)
{
  if (!enabled) {
    atomic_store(&s_enabled, false);
    atomic_fetch_add(&s_epoch, 1); /* pending old-netif frames stay invalid after restart */
  } else {
    atomic_store(&s_enabled, true);
  }
}

esp_err_t ml_usb_tx_send(const void * buffer, size_t len)
{
  uint32_t epoch = atomic_load(&s_epoch);
  if (!atomic_load_explicit(&s_ready, memory_order_acquire) || !atomic_load(&s_enabled) || !tud_mounted()) {
    atomic_fetch_add(&s_rejected_unavailable, 1);
    return ESP_ERR_INVALID_STATE;
  }
  if (!buffer || len == 0 || len > TX_FRAME_MAX) {
    atomic_fetch_add(&s_errors, 1);
    return ESP_ERR_INVALID_ARG;
  }
  /* The netif glue checks lwIP-core ownership. Fail fast on concurrent or
   * reentrant use rather than invert publication order. No USB waits here. */
  if (atomic_exchange(&s_caller_busy, true)) {
    atomic_fetch_add(&s_errors, 1);
    return ESP_ERR_INVALID_STATE;
  }
  tx_request_t * r = NULL;
  esp_err_t result = ESP_ERR_NO_MEM;
  if (atomic_load(&s_pending) >= TX_DEFER_CAP) {
    atomic_fetch_add(&s_defer_cap, 1);
    goto out;
  }
  for (size_t i = 0; i < TX_SLOTS; i++) {
    unsigned expected = TX_FREE;
    if (atomic_compare_exchange_strong_explicit(
          &s_requests[i].state, &expected, TX_FILLING, memory_order_acq_rel, memory_order_acquire))
    {
      r = &s_requests[i];
      break;
    }
  }
  if (!r) {
    atomic_fetch_add(&s_exhausted, 1);
    goto out;
  }
  r->len = (uint16_t)len;
  r->epoch = epoch;
  r->queued_us = esp_timer_get_time();
  atomic_store(&r->queued_ms, (uint32_t)(r->queued_us / 1000));
  r->copied = false;
  memcpy(r->bytes, buffer, len);
  atomic_store_explicit(&r->state, TX_PENDING, memory_order_release);
  atomic_fetch_add(&s_pending, 1); /* reserve before the callback can run */
  if (!tud_defer_func_try(usb_send, r)) {
    atomic_fetch_sub(&s_pending, 1);
    atomic_fetch_add(&s_defer_full, 1);
    request_release(r); /* known not enqueued: no callback can retain this slot */
    goto out;
  }
  /* Ownership transferred. The callback may already have freed/reused r;
   * never access it after a successful enqueue, including for accounting. */
  atomic_fetch_add(&s_submitted, 1);
  result = ESP_OK;
out:
  atomic_store(&s_caller_busy, false);
  return result;
}

void ml_usb_tx_get_diag(ml_usb_tx_diag_t * out)
{
  if (!out) return;
  *out = (ml_usb_tx_diag_t){
    .submitted = atomic_load(&s_submitted),
    .copied = atomic_load(&s_copied),
    .cancelled = atomic_load(&s_cancelled),
    .errors = atomic_load(&s_errors),
    .rejected_unavailable = atomic_load(&s_rejected_unavailable),
    .unmounted_drop = atomic_load(&s_unmounted_drop),
    .exhausted = atomic_load(&s_exhausted),
    .defer_full = atomic_load(&s_defer_full),
    .can_xmit_fail = atomic_load(&s_can_xmit_fail),
    .integrity_errors = atomic_load(&s_integrity_errors),
    .defer_cap = atomic_load(&s_defer_cap),
    .pending = atomic_load(&s_pending),
    .callbacks = atomic_load(&s_callbacks),
  };
  uint32_t callback_ms = atomic_load(&s_last_callback_ms);
  out->callback_age_ms = (uint32_t)(esp_timer_get_time() / 1000) - callback_ms;
  for (size_t i = 0; i < 4; i++) out->latency[i] = atomic_load(&s_latency[i]);
  for (size_t i = 0; i < TX_SLOTS; i++) {
    unsigned state = atomic_load_explicit(&s_requests[i].state, memory_order_acquire);
    if (state != TX_FREE) out->used++;
    if (state != TX_PENDING && state != TX_RUNNING) continue;
    uint32_t queued = atomic_load(&s_requests[i].queued_ms);
    uint32_t age = (uint32_t)(esp_timer_get_time() / 1000) - queued;
    if (age > out->oldest_ms) out->oldest_ms = age;
  }
}

void ml_usb_tx_note_ownership_error(void)
{
  atomic_fetch_add(&s_integrity_errors, 1);
}
