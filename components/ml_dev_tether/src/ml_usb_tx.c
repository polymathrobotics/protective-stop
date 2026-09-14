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
#define TX_FRAME_MAX 1536 /* Ethernet MTU plus link-layer headers. */
#define TX_TTL_US 100000
#define TX_RETRY_US 2000

_Static_assert((TX_SLOTS & (TX_SLOTS - 1)) == 0, "FIFO indices must survive unsigned wrap");

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
  bool ncm_busy;
} tx_request_t;

static tx_request_t s_requests[TX_SLOTS];
static atomic_bool s_ready;
static atomic_bool s_enabled;
static atomic_bool s_caller_busy;
static atomic_uint s_epoch;
static atomic_uint s_submitted, s_copied, s_cancelled, s_errors;
static atomic_uint s_rejected_unavailable, s_unmounted_drop;
static atomic_uint s_exhausted, s_defer_full, s_can_xmit_fail, s_integrity_errors;
static atomic_uint s_head, s_tail, s_last_callback_ms, s_callbacks;
static atomic_uint s_ncm_busy_retries, s_busy_since_us;
static atomic_bool s_drain_owned, s_retry_wait;
static esp_timer_handle_t s_kick_timer;
static atomic_uint s_latency[4];

static void request_release(tx_request_t * r)
{
  /* The sole consumer finishes all slot accesses before publishing its head. */
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

static void usb_drain(void * context)
{
  (void)context;
  unsigned head = atomic_load_explicit(&s_head, memory_order_relaxed);
  /* Snapshot bounds each dispatch, even if the producer continues pushing. */
  unsigned limit = atomic_load_explicit(&s_tail, memory_order_acquire);
  atomic_store(&s_retry_wait, false);
  while (head != limit) {
    tx_request_t * r = &s_requests[head % TX_SLOTS];
    int64_t now = esp_timer_get_time();
    if (atomic_load_explicit(&r->state, memory_order_acquire) != TX_PENDING) {
      /* Published non-pending state is corruption, not a skippable packet.
       * Preserve the bounded FIFO and fail visibly rather than release storage
       * that another context might still own. Qualification must stop. */
      atomic_fetch_add(&s_integrity_errors, 1);
      break;
    }
    if (request_expired(r, now)) {
      atomic_fetch_add(&s_cancelled, 1);
      if (r->ncm_busy && now >= r->queued_us && now - r->queued_us >= TX_TTL_US) {
        atomic_fetch_add(&s_can_xmit_fail, 1);
      }
    } else if (!tud_mounted()) {
      atomic_fetch_add(&s_unmounted_drop, 1);
    } else if (!tud_network_can_xmit(r->len)) {
      /* Retain the HEAD, not a re-enqueued packet behind newer frames. A false
       * capacity probe is not a drop: expiry remains a counted hard failure. */
      r->ncm_busy = true;
      atomic_fetch_add(&s_ncm_busy_retries, 1);
      atomic_store(&s_busy_since_us, (uint32_t)esp_timer_get_time());
      atomic_store(&s_retry_wait, true);
      break;
    } else if (request_expired(r, esp_timer_get_time())) {
      atomic_fetch_add(&s_cancelled, 1);
      if (r->ncm_busy) atomic_fetch_add(&s_can_xmit_fail, 1);
    } else {
      atomic_store(&r->state, TX_RUNNING);
      tud_network_xmit(r, r->len);
      if (!r->copied) atomic_fetch_add(&s_errors, 1);
    }
    now = esp_timer_get_time();
    int64_t latency = now - r->queued_us;
    unsigned bucket = latency < 10000 ? 0u : latency < 50000 ? 1u : latency < 100000 ? 2u : 3u;
    atomic_fetch_add(&s_latency[bucket], 1);
    atomic_store(&s_last_callback_ms, (uint32_t)(now / 1000));
    atomic_fetch_add(&s_callbacks, 1); /* per-frame retirement, not capacity probes */
    request_release(r);
    atomic_store_explicit(&s_head, ++head, memory_order_release);
  }
#ifdef ML_USB_TX_HOST_TEST
  usb_tx_test_before_idle();
#endif
  /* A push after the snapshot/final check may see this flag still set. The
   * resident timer supplies the missed kick without racing slot ownership. */
  atomic_store_explicit(&s_drain_owned, false, memory_order_release);
}

static void usb_kick(void * context)
{
  (void)context;
  if (!atomic_load_explicit(&s_ready, memory_order_acquire)) return;
  if (atomic_load_explicit(&s_head, memory_order_acquire) == atomic_load_explicit(&s_tail, memory_order_acquire))
    return;
  if (
    atomic_load(&s_retry_wait) &&
    (uint32_t)((uint32_t)esp_timer_get_time() - atomic_load(&s_busy_since_us)) < TX_RETRY_US)
    return;
  bool expected = false;
  if (!atomic_compare_exchange_strong(&s_drain_owned, &expected, true)) return;
  if (!tud_defer_func_try(usb_drain, NULL)) {
    atomic_fetch_add(&s_defer_full, 1); /* remains a hard-gate fault */
    atomic_store_explicit(&s_drain_owned, false, memory_order_release);
    /* No callback was queued. FIFO ownership stays intact; next tick retries. */
  }
}

esp_err_t ml_usb_tx_init(void)
{
  if (atomic_load_explicit(&s_ready, memory_order_acquire)) return ESP_OK;
  uint8_t * payloads = heap_caps_malloc(TX_SLOTS * TX_FRAME_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!payloads) return ESP_ERR_NO_MEM;
  const esp_timer_create_args_t timer_args = {
    .callback = usb_kick,
    .dispatch_method = ESP_TIMER_TASK, /* tud_defer_func_try uses task-only queue APIs */
    .name = "usb_tx",
    .skip_unhandled_events = true,
  };
  esp_err_t result = esp_timer_create(&timer_args, &s_kick_timer);
  if (result == ESP_OK) result = esp_timer_start_periodic(s_kick_timer, TX_RETRY_US);
  if (result != ESP_OK) {
    if (s_kick_timer) esp_timer_delete(s_kick_timer);
    s_kick_timer = NULL;
    heap_caps_free(payloads);
    return result;
  }
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
  unsigned tail = atomic_load_explicit(&s_tail, memory_order_relaxed);
  esp_err_t result = ESP_ERR_NO_MEM;
  if (tail - atomic_load_explicit(&s_head, memory_order_acquire) >= TX_SLOTS) {
#ifdef ML_USB_TX_HOST_TEST
    usb_tx_test_after_full_check();
#endif
    /* The other core may just have retired a slot. One bounded re-read avoids
     * rejecting on that first full snapshot; this is not a wait/spin loop. */
    if (tail - atomic_load_explicit(&s_head, memory_order_acquire) >= TX_SLOTS) {
      atomic_fetch_add(&s_exhausted, 1);
      goto out;
    }
  }
  tx_request_t * r = &s_requests[tail % TX_SLOTS];
  unsigned expected = TX_FREE;
  if (!atomic_compare_exchange_strong_explicit(
        &r->state, &expected, TX_FILLING, memory_order_acq_rel, memory_order_acquire))
  {
    atomic_fetch_add(&s_integrity_errors, 1);
    result = ESP_ERR_INVALID_STATE;
    goto out;
  }
  r->len = (uint16_t)len;
  r->epoch = epoch;
  r->queued_us = esp_timer_get_time();
  atomic_store(&r->queued_ms, (uint32_t)(r->queued_us / 1000));
  r->copied = false;
  r->ncm_busy = false;
  memcpy(r->bytes, buffer, len);
  atomic_store_explicit(&r->state, TX_PENDING, memory_order_release);
  atomic_fetch_add(&s_submitted, 1);
  atomic_store_explicit(&s_tail, tail + 1, memory_order_release);
  /* Publication transfers ownership. A fast-path drain can already have freed
   * r before this returns; neither the producer nor timer accesses it again. */
  usb_kick(NULL);
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
    .defer_cap = 0, /* legacy per-packet closure cap; now at most ONE drain */
    .ncm_busy_retries = atomic_load(&s_ncm_busy_retries),
    .callbacks = atomic_load(&s_callbacks),
  };
  uint32_t callback_ms = atomic_load(&s_last_callback_ms);
  out->callback_age_ms = (uint32_t)(esp_timer_get_time() / 1000) - callback_ms;
  for (size_t i = 0; i < 4; i++) out->latency[i] = atomic_load(&s_latency[i]);
  for (size_t i = 0; i < TX_SLOTS; i++) {
    unsigned state = atomic_load_explicit(&s_requests[i].state, memory_order_acquire);
    if (state != TX_FREE) out->used++;
    if (state == TX_PENDING || state == TX_RUNNING) out->pending++;
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
