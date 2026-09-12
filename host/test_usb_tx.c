// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Compile the actual adapter and TinyUSB same-TU extension against a
 * deterministic queue/clock. No shadow implementation of their state machine. */
#include <stdio.h>
#include <stdlib.h>

#include "../components/ml_dev_tether/src/ml_usb_tx.c"

static int queue_marker;
static void * _usbd_q = &queue_marker;
#include "../components/ml_dev_tether/src/tud_defer_try.inc"

static unsigned checks, failures;
#define CHECK(condition, message)    \
  do {                               \
    checks++;                        \
    if (!(condition)) {              \
      failures++;                    \
      printf("FAIL: %s\n", message); \
    }                                \
  } while (0)

static int64_t clock_us;
static bool mounted = true, can_xmit = true, fail_alloc, inline_callback, skip_copy, reenter;
static bool restart_at_timestamp;
static int64_t can_xmit_delay;
static dcd_event_t queue_events[32];
static unsigned queue_count, queue_limit = ML_USB_TX_EVENT_QUEUE_SIZE, hooks, legacy_calls, allocations;
static unsigned frame_count;
static uint8_t frames[64][TX_FRAME_MAX];
static uint16_t lengths[64];

void * heap_caps_malloc(size_t size, unsigned caps)
{
  CHECK(caps == (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT), "payload pool uses PSRAM");
  if (fail_alloc) return NULL;
  allocations++;
  return malloc(size);
}

int64_t esp_timer_get_time(void)
{
  if (restart_at_timestamp) {
    restart_at_timestamp = false;
    ml_usb_tx_set_enabled(false);
    ml_usb_tx_set_enabled(true);
  }
  return clock_us;
}

bool tud_mounted(void)
{
  return mounted;
}

bool tud_network_can_xmit(uint16_t len)
{
  CHECK(len <= TX_FRAME_MAX, "capacity queried with bounded frame");
  clock_us += can_xmit_delay;
  return can_xmit;
}

uint16_t __real_tud_network_xmit_cb(uint8_t * dst, void * ref, uint16_t arg)
{
  legacy_calls++;
  memcpy(dst, ref, arg);
  return arg;
}

void tud_network_xmit(void * ref, uint16_t len)
{
  if (skip_copy) return;
  CHECK(frame_count < 64, "capture has room");
  lengths[frame_count] = __wrap_tud_network_xmit_cb(frames[frame_count], ref, len);
  frame_count++;
}

int xQueueSendToBack(void * queue, const void * event, unsigned wait)
{
  CHECK(wait == 0, "enqueue NEVER blocks TCPIP");
  CHECK(queue == _usbd_q && queue != NULL, "enqueue uses existing device queue");
  if (queue_count >= queue_limit) return 0;
  const dcd_event_t * ev = event;
  CHECK(ev->rhport == 0 && ev->event_id == USBD_EVENT_FUNC_CALL, "correct TinyUSB event format");
  if (reenter) {
    reenter = false;
    CHECK(ml_usb_tx_send("nested", 6) == ESP_ERR_INVALID_STATE, "reentrant producer fails fast");
  }
  if (inline_callback)
    ev->func_call.func(ev->func_call.param);
  else
    queue_events[queue_count++] = *ev;
  return pdTRUE;
}

void tud_event_hook_cb(unsigned rhport, unsigned event_id, bool in_isr)
{
  CHECK(rhport == 0 && event_id == USBD_EVENT_FUNC_CALL && !in_isr, "success hook uses task context");
  hooks++;
}

static void drain_one(void)
{
  CHECK(queue_count != 0, "callback exists");
  dcd_event_t ev = queue_events[0];
  for (unsigned i = 1; i < queue_count; i++) queue_events[i - 1] = queue_events[i];
  queue_count--;
  ev.func_call.func(ev.func_call.param);
}

static void drain_all(void)
{
  while (queue_count) drain_one();
}

static ml_usb_tx_diag_t diag(void)
{
  ml_usb_tx_diag_t d;
  ml_usb_tx_get_diag(&d);
  return d;
}

static void reset_fixture(void)
{
  CHECK(queue_count == 0 && diag().used == 0 && diag().pending == 0, "previous scenario released every slot");
  ml_usb_tx_set_enabled(false);
  ml_usb_tx_set_enabled(true);
  clock_us += 1000000;
  mounted = can_xmit = true;
  inline_callback = skip_copy = reenter = false;
  can_xmit_delay = 0;
  queue_limit = ML_USB_TX_EVENT_QUEUE_SIZE;
  frame_count = hooks = legacy_calls = 0;
}

int main(void)
{
  fail_alloc = true;
  CHECK(ml_usb_tx_init() == ESP_ERR_NO_MEM, "init OOM is recoverable");
  fail_alloc = false;
  CHECK(ml_usb_tx_init() == ESP_OK && ml_usb_tx_init() == ESP_OK, "resident init is idempotent");
  CHECK(allocations == 1, "one fixed pool allocation");
  CHECK(TX_DEFER_CAP == 16 && CFG_TUD_TASK_QUEUE_SZ == 64, "reviewed admission/queue limits");
  CHECK(CFG_TUD_TASK_QUEUE_SZ - TX_DEFER_CAP >= 48, "DCD/other events retain shared queue headroom");
  CHECK(ml_usb_tx_send("A", 1) == ESP_ERR_INVALID_STATE, "disabled netif rejects send");
  reset_fixture();

  uint8_t borrowed[] = {1, 2, 3, 4};
  CHECK(ml_usb_tx_send(borrowed, sizeof(borrowed)) == ESP_OK, "accept copied request immediately");
  CHECK(frame_count == 0 && diag().pending == 1, "accepted is not copied/wire-delivery proof");
  memset(borrowed, 0xff, sizeof(borrowed));
  clock_us += 5000;
  drain_all();
  CHECK(
    frame_count == 1 && lengths[0] == 4 && frames[0][0] == 1 && frames[0][3] == 4,
    "caller buffer may change/free before callback");
  CHECK(diag().latency[0] == 1 && diag().copied == 1, "normal callback counters");

  reset_fixture();
  uint32_t cancelled = diag().cancelled;
  CHECK(ml_usb_tx_send("A", 1) == ESP_OK, "enqueue A");
  clock_us += 90000;
  CHECK(ml_usb_tx_send("B", 1) == ESP_OK, "enqueue B while A remains pending");
  clock_us += 10000;
  drain_one();
  clock_us += 1000;
  drain_one();
  CHECK(frame_count == 1 && frames[0][0] == 'B', "A expires, B transmits; no stale A after B");
  CHECK(diag().cancelled == cancelled + 1, "expired packet is counted");

  reset_fixture();
  uint32_t full = diag().defer_full;
  queue_limit = 0;
  CHECK(ml_usb_tx_send("C", 1) == ESP_ERR_NO_MEM, "false defer returns failure");
  CHECK(diag().used == 0 && diag().pending == 0 && hooks == 0, "false defer releases ownership and emits no hook");
  CHECK(diag().defer_full == full + 1, "false defer is counted");
  _usbd_q = NULL;
  CHECK(!tud_defer_func_try(usb_send, NULL), "uninitialized SDK queue fails without dereference");
  _usbd_q = &queue_marker;
  CHECK(!tud_defer_func_try(NULL, NULL), "null callback rejected");

  reset_fixture();
  uint32_t cap = diag().defer_cap;
  for (unsigned i = 0; i < TX_DEFER_CAP; i++) CHECK(ml_usb_tx_send("Q", 1) == ESP_OK, "bounded closures accepted");
  CHECK(ml_usb_tx_send("overflow", 8) == ESP_ERR_NO_MEM, "producer leaves DCD queue headroom");
  CHECK(
    queue_count == TX_DEFER_CAP && diag().pending == TX_DEFER_CAP && diag().defer_cap == cap + 1,
    "closure cap and counter enforced");
  clock_us += 500000;
  CHECK(diag().used == TX_DEFER_CAP && diag().oldest_ms == 500, "stalled USB backlog is observable and bounded");
  drain_all();
  CHECK(diag().used == 0 && frame_count == 0, "drain releases every expired request");
  CHECK(ml_usb_tx_send("recovered", 9) == ESP_OK, "pool recovers without reset");
  drain_all();
  CHECK(frame_count == 1 && frames[0][0] == 'r', "only new request transmits after stall");

  reset_fixture();
  uint32_t capacity = diag().can_xmit_fail;
  can_xmit = false;
  CHECK(ml_usb_tx_send("busy", 4) == ESP_OK, "enqueue before NCM capacity is known");
  drain_all();
  can_xmit = true;
  CHECK(ml_usb_tx_send("ready", 5) == ESP_OK, "enqueue after NCM becomes available");
  drain_all();
  CHECK(
    frame_count == 1 && frames[0][0] == 'r' && diag().can_xmit_fail == capacity + 1,
    "NCM busy is a counted drop, never retried/reordered");

  reset_fixture();
  CHECK(ml_usb_tx_send("old", 3) == ESP_OK, "enqueue before netif stop");
  ml_usb_tx_set_enabled(false);
  ml_usb_tx_set_enabled(true);
  CHECK(ml_usb_tx_send("new", 3) == ESP_OK, "enqueue after netif restart");
  drain_all();
  CHECK(frame_count == 1 && frames[0][0] == 'n', "old epoch cannot transmit after restart");

  reset_fixture();
  cancelled = diag().cancelled;
  restart_at_timestamp = true;
  CHECK(ml_usb_tx_send("preempted", 9) == ESP_OK, "netif restart may interrupt publication");
  drain_all();
  CHECK(
    frame_count == 0 && diag().cancelled == cancelled + 1, "entry epoch prevents old send adopting new netif epoch");

  reset_fixture();
  inline_callback = true;
  CHECK(ml_usb_tx_send("inline", 6) == ESP_OK, "callback may finish before enqueue returns");
  CHECK(diag().used == 0 && diag().pending == 0 && frame_count == 1, "early callback leaves no stale ownership");
  CHECK(ml_usb_tx_send("next", 4) == ESP_OK, "reuse after early completion has no stale signal");
  CHECK(frame_count == 2 && frames[1][0] == 'n', "next request copies exactly once");

  reset_fixture();
  const int64_t delays[] = {9999, 10000, 49999, 50000, 99999, 100000};
  const unsigned buckets[] = {0, 1, 1, 2, 2, 3};
  for (size_t i = 0; i < 6; i++) {
    ml_usb_tx_diag_t before = diag();
    CHECK(ml_usb_tx_send("T", 1) == ESP_OK, "enqueue histogram boundary case");
    clock_us += delays[i];
    drain_all();
    CHECK(diag().latency[buckets[i]] == before.latency[buckets[i]] + 1, "callback latency bucket boundary");
    CHECK(diag().cancelled == before.cancelled + (i == 5), "100ms TTL boundary drops, 99999us accepts");
    clock_us += 1000;
  }

  reset_fixture();
  cancelled = diag().cancelled;
  can_xmit_delay = TX_TTL_US;
  CHECK(ml_usb_tx_send("slow", 4) == ESP_OK, "enqueue capacity-check stall case");
  drain_all();
  CHECK(frame_count == 0 && diag().cancelled == cancelled + 1, "TTL rechecked before claim");

  reset_fixture();
  uint32_t errors = diag().errors;
  reenter = true;
  CHECK(ml_usb_tx_send("outer", 5) == ESP_OK, "outer producer succeeds");
  drain_all();
  CHECK(frame_count == 1 && diag().errors == errors + 1, "reentrant producer cannot reorder publication");
  errors = diag().errors;
  skip_copy = true;
  CHECK(ml_usb_tx_send("notcopied", 9) == ESP_OK, "enqueue missing-copy case");
  drain_all();
  CHECK(diag().errors == errors + 1, "missing TinyUSB copy is counted even after enqueue success");

  reset_fixture();
  uint32_t unavailable = diag().rejected_unavailable;
  uint32_t unmounted = diag().unmounted_drop;
  CHECK(ml_usb_tx_send("unmount", 7) == ESP_OK, "enqueue before USB unmount");
  mounted = false;
  CHECK(ml_usb_tx_send("absent", 6) == ESP_ERR_INVALID_STATE, "unmounted producer is rejected without enqueue");
  drain_all();
  CHECK(diag().unmounted_drop == unmounted + 1 && frame_count == 0, "unmounted callback has distinct drop counter");
  CHECK(diag().rejected_unavailable == unavailable + 1, "unavailable producer has distinct rejection counter");
  mounted = true;
  CHECK(ml_usb_tx_send("remount", 7) == ESP_OK, "producer recovers after remount");
  drain_all();
  CHECK(frame_count == 1 && frames[0][0] == 'r', "remount sends only the new packet");
  uint8_t legacy[] = {9, 8, 7}, dst[3];
  CHECK(__wrap_tud_network_xmit_cb(dst, legacy, 3) == 3 && legacy_calls == 1, "legacy refs forward to vendor callback");
  CHECK(memcmp(dst, legacy, 3) == 0, "legacy payload preserved");
  uint32_t integrity = diag().integrity_errors;
  CHECK(__wrap_tud_network_xmit_cb(dst, &s_requests[0], 3) == 0, "invalid own-pool callback fails closed");
  CHECK(diag().integrity_errors == integrity + 1, "invalid callback is counted");
  integrity = diag().integrity_errors;
  ml_usb_tx_note_ownership_error();
  CHECK(diag().integrity_errors == integrity + 1, "ownership violation is counted without assertion or reboot");

  reset_fixture();
  CHECK(ml_usb_tx_send("old", 3) == ESP_OK, "queue old epoch request");
  void * old_ref = queue_events[0].func_call.param;
  ml_usb_tx_set_enabled(false);
  ml_usb_tx_set_enabled(true);
  CHECK(ml_usb_tx_send("new", 3) == ESP_OK, "queue new epoch request while old closure exists");
  CHECK(queue_events[1].func_call.param != old_ref, "slot cannot be reused before its old closure runs");
  drain_all();
  CHECK(frame_count == 1 && frames[0][0] == 'n', "old epoch closure never copies new request bytes");
  CHECK(ml_usb_tx_send("reuse", 5) == ESP_OK, "slot can be reused after closure completion");
  CHECK(queue_events[0].func_call.param == old_ref, "completed slot is available again");
  drain_all();
  CHECK(frame_count == 2 && frames[1][0] == 'r', "reused slot transmits only the new request");

  reset_fixture();
  uint32_t exhausted = diag().exhausted;
  for (size_t i = 0; i < TX_SLOTS; i++) atomic_store(&s_requests[i].state, TX_FILLING);
  CHECK(ml_usb_tx_send("full", 4) == ESP_ERR_NO_MEM, "pool exhaustion fails fast");
  CHECK(diag().exhausted == exhausted + 1 && queue_count == 0, "pool exhaustion counted without enqueue");
  for (size_t i = 0; i < TX_SLOTS; i++) atomic_store(&s_requests[i].state, TX_FREE);
  CHECK(ml_usb_tx_send(NULL, 1) == ESP_ERR_INVALID_ARG, "null input rejected");
  CHECK(ml_usb_tx_send("x", 0) == ESP_ERR_INVALID_ARG, "empty frame rejected");
  CHECK(ml_usb_tx_send("x", TX_FRAME_MAX + 1) == ESP_ERR_INVALID_ARG, "oversized frame rejected before copy");
  reset_fixture();
  printf("test_usb_tx: %u checks, %u failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
