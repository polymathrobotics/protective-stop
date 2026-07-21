#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Wires a ringbuffer in RTC_NOINIT memory into esp_log so the previous boot's
 * tail-of-log survives panic/WDT/esp_restart and can be read out on the next
 * boot. Call once at the start of app_main, before anything else logs. */
void   panic_log_init(void);

/* True if RTC_NOINIT had a valid magic at boot — i.e. there's a snapshot of
 * the previous boot's log available. */
bool   panic_log_has_snapshot(void);

/* Copy the snapshot into `out` (NUL-terminated, truncated at out_size).
 * Returns bytes written, excluding the terminating NUL. */
size_t panic_log_copy_snapshot(char *out, size_t out_size);
