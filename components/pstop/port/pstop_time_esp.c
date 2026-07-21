/* SPDX-License-Identifier: Apache-2.0
 *
 * ESP-IDF port of pstop's time_get_now().
 *
 * Upstream's pstop_c/pstop/src/pstop/time.c only implements time_get_now()
 * on __linux__ (via timespec_get); on all other targets it returns 0, which
 * would break heartbeat/timeout logic on the ESP32. This file substitutes
 * an esp_timer-backed implementation. The component's CMakeLists.txt
 * excludes upstream's time.c from the build so only this implementation
 * is linked.
 *
 * Units: pstop expects milliseconds. esp_timer_get_time() returns the
 * time since boot in microseconds, so we divide by 1000. The clock is
 * monotonic, free of NTP/RTC discontinuities — exactly what a timeout/
 * heartbeat clock wants.
 */

#include "pstop/time.h"
#include "esp_timer.h"

uint64_t time_get_now(void) {
    return (uint64_t)(esp_timer_get_time() / 1000);
}
