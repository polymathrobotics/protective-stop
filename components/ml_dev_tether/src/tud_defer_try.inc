// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

#include "ml_usb_defer.h"
#include "ml_usb_tx_limits.h"

/* Appended to the audited TinyUSB usbd.c translation unit in the build tree.
 * Unlike usbd_defer_func(), return queue acceptance and never block TCPIP.
 * Preserve queue_event()'s event hook after successful enqueue. Task context only. */
#if CFG_TUD_ENABLED && CFG_TUSB_OS == OPT_OS_FREERTOS
  #if CFG_TUD_TASK_QUEUE_SZ != ML_USB_TX_EVENT_QUEUE_SIZE
    #error "Unexpected TinyUSB device queue size"
  #endif
bool tud_defer_func_try(void (*func)(void *), void * param)
{
  if (func == NULL || _usbd_q == NULL) return false;
  dcd_event_t event = {
    .rhport = 0,
    .event_id = USBD_EVENT_FUNC_CALL,
  };
  event.func_call.func = func;
  event.func_call.param = param;
  if (xQueueSendToBack(_usbd_q, &event, 0) != pdTRUE) return false;
  tud_event_hook_cb(event.rhport, event.event_id, false);
  return true;
}
#else
  #error "tud_defer_func_try requires the audited TinyUSB FreeRTOS device configuration"
#endif
