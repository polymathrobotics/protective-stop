// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Zephyr implementation of pstop_c's time_get_now().
//
// Upstream's pstop_c/pstop/src/pstop/time.c implements this only under
// __linux__ and returns 0 everywhere else. Linking that into a Zephyr image
// gives a clock that never advances, which disables every heartbeat and
// timeout in the protocol while the build succeeds and the code looks
// correct. zephyr/CMakeLists.txt compiles this file instead whenever
// CONFIG_PSTOP_ZEPHYR_TIME is set, and excludes upstream's time.c.
//
// Units: pstop expects milliseconds. k_uptime_get() returns milliseconds
// since boot from the monotonic kernel clock, so it carries no RTC or NTP
// discontinuities -- which is exactly what a timeout clock needs.

#include <zephyr/kernel.h>

#include "pstop/time.h"

uint64_t time_get_now(void)
{
	return (uint64_t)k_uptime_get();
}
