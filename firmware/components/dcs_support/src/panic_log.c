// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file panic_log.c
 * @brief Reboot-surviving log capture (RTC_NOINIT) for tracing crashes.
 *
 * The chip's USB-Serial-JTAG is gone once TinyUSB takes over, and we can't
 * route ESP_LOG to TinyUSB CDC without blocking early boot. To debug crashes
 * we instead push every ESP_LOG message into a small ringbuffer placed in
 * RTC_NOINIT memory — which survives all reset types EXCEPT a true power
 * cycle (panic / WDT / esp_restart all preserve it).
 *
 * On the next boot we check the magic + length, copy the buffer out, then
 * clear the magic so we don't keep showing the same stale snapshot forever.
 * The captured log is exposed at /api/last_log so we can read it via the
 * admin HTTP server.
 */

#include "panic_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#define PANIC_LOG_MAGIC 0x4A07BCDEu
#define PANIC_LOG_BUF_SIZE 7168u /* fits comfortably in 8 KiB RTC slow memory */

typedef struct
{
  uint32_t magic;
  uint32_t head; /* next write offset, modulo PANIC_LOG_BUF_SIZE */
  uint32_t wrapped; /* nonzero once the ring has wrapped at least once */
  uint32_t reserved;
  char buf[PANIC_LOG_BUF_SIZE];
} panic_log_t;

static RTC_NOINIT_ATTR panic_log_t s_pl;

/* Boot-time copy for /api/last_log. Held in PSRAM, not internal DRAM: it's
 * touched only at boot (memcpy below) and over the admin HTTP server — never
 * from an ISR or with the flash cache disabled — so external RAM is safe, and
 * this keeps ~7 KB of scarce internal SRAM free for the WiFi/WG/TLS stacks.
 * (The crash-critical ring s_pl stays in RTC_NOINIT, which survives reboots.) */
static panic_log_t * s_pl_snapshot = NULL;
static bool s_have_snapshot = false;

static vprintf_like_t s_chain_vprintf = NULL;

static void panic_log_write_chars(const char * src, size_t len)
{
  if (len == 0u) {
    return;
  }

  uint32_t head = s_pl.head;
  for (size_t i = 0; i < len; i++) {
    s_pl.buf[head] = src[i];
    head++;
    if (head >= PANIC_LOG_BUF_SIZE) {
      head = 0;
      s_pl.wrapped = 1;
    }
  }
  s_pl.head = head;
}

static int panic_log_vprintf(const char * fmt, va_list ap)
{
  char buf[160];
  va_list ap_copy;
  va_copy(ap_copy, ap);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n > 0) {
    size_t take = (n < ((int)sizeof(buf) - 1)) ? (size_t)n : (sizeof(buf) - 1u);
    panic_log_write_chars(buf, take);
  }

  if (s_chain_vprintf != NULL) {
    int rv = s_chain_vprintf(fmt, ap_copy);
    va_end(ap_copy);
    return rv;
  }
  va_end(ap_copy);
  return n;
}

void panic_log_init(void)
{
  /* If RTC magic is valid, this boot is following a (non-power-cycle) reset.
     * Snapshot the previous-boot log to user-readable storage, then reset the
     * ring so the new boot starts fresh. */
  if (s_pl.magic == PANIC_LOG_MAGIC) {
    s_pl_snapshot = heap_caps_malloc(sizeof(*s_pl_snapshot), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_pl_snapshot != NULL) {
      (void)memcpy(s_pl_snapshot, &s_pl, sizeof(*s_pl_snapshot));
      s_have_snapshot = true;
    }
  }

  s_pl.magic = PANIC_LOG_MAGIC;
  s_pl.head = 0;
  s_pl.wrapped = 0;
  s_pl.buf[0] = '\0';

  /* Chain into ESP_LOG. esp_log_set_vprintf returns the previous handler so
     * downstream ESP_LOG output still reaches the default console. */
  s_chain_vprintf = esp_log_set_vprintf(panic_log_vprintf);
}

bool panic_log_has_snapshot(void)
{
  return s_have_snapshot;
}

/* ----------------------------------------------------------------------------
 * Panic-time capture
 * ----------------------------------------------------------------------------
 * The IDF panic handler calls panic_print_char() to emit the Guru Meditation
 * banner + backtrace via UART / USB-CDC / USB-Serial-JTAG. None of those
 * outputs reach a host on this board (TinyUSB or no console at all). We use
 * `-Wl,--wrap=panic_print_char` in the main CMakeLists to intercept each
 * char, copy it into the same RTC_NOINIT ringbuffer, and also forward to the
 * real handler so the original chain still runs. After the panic-triggered
 * reboot, the snapshot held over from this boot will end with the panic text.
 */
extern void __real_panic_print_char(const char c);
void __wrap_panic_print_char(const char c);

void __wrap_panic_print_char(const char c)
{
  /* Append to ringbuffer. No locking — we're in panic context, all other
     * tasks are frozen and interrupts are off. */
  s_pl.buf[s_pl.head] = c;
  s_pl.head++;
  if (s_pl.head >= PANIC_LOG_BUF_SIZE) {
    s_pl.head = 0;
    s_pl.wrapped = 1;
  }
  __real_panic_print_char(c);
}

size_t panic_log_copy_snapshot(char * out, size_t out_size)
{
  if ((!s_have_snapshot) || (s_pl_snapshot == NULL) || (out_size == 0u)) {
    if (out_size > 0u) {
      out[0] = '\0';
    }
    return 0;
  }

  /* Reconstruct linear buffer from the ringbuffer. If never wrapped, head
     * points one past the last written byte. If wrapped, oldest data lives
     * at head. */
  size_t total;
  size_t start;
  if (s_pl_snapshot->wrapped != 0u) {
    total = PANIC_LOG_BUF_SIZE;
    start = s_pl_snapshot->head;
  } else {
    total = s_pl_snapshot->head;
    start = 0;
  }

  size_t copied = 0;
  for (size_t i = 0; (i < total) && ((copied + 1u) < out_size); i++) {
    size_t src = (start + i) % PANIC_LOG_BUF_SIZE;
    out[copied] = s_pl_snapshot->buf[src];
    copied++;
  }
  out[copied] = '\0';
  return copied;
}
