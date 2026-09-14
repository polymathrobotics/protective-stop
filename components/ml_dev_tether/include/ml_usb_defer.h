// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /* FreeRTOS task context only. True transfers callback-context ownership to
 * TinyUSB; false guarantees no callback was queued. Never waits for capacity. */
  bool tud_defer_func_try(void (*func)(void *), void * param);

#ifdef __cplusplus
}
#endif
