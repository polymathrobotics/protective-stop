// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ml_usb_tx_limits.h"

#define MALLOC_CAP_SPIRAM 1
#define MALLOC_CAP_8BIT 2
#define CFG_TUD_ENABLED 1
#define CFG_TUD_TASK_QUEUE_SZ ML_USB_TX_EVENT_QUEUE_SIZE
#define CFG_TUSB_OS 1
#define OPT_OS_FREERTOS 1
#define USBD_EVENT_FUNC_CALL 7
#define pdTRUE 1

typedef struct
{
  unsigned rhport;
  unsigned event_id;

  struct
  {
    void (*func)(void *);
    void * param;
  } func_call;
} dcd_event_t;

void * heap_caps_malloc(size_t size, unsigned caps);
int64_t esp_timer_get_time(void);
bool tud_mounted(void);
bool tud_network_can_xmit(uint16_t len);
void tud_network_xmit(void * ref, uint16_t len);
int xQueueSendToBack(void * queue, const void * event, unsigned wait);
void tud_event_hook_cb(unsigned rhport, unsigned event_id, bool in_isr);
