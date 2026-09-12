// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
#pragma once

/* Measured nominal USB traffic overflowed four callbacks with NCM capacity
 * still available. Increase the event queue and our cap together, retaining
 * three quarters of the shared queue for DCD/other events. */
#define ML_USB_TX_EVENT_QUEUE_SIZE 64
#if ML_USB_TX_EVENT_QUEUE_SIZE < 4 || ML_USB_TX_EVENT_QUEUE_SIZE % 4 != 0
  #error "USB event queue size must be a positive multiple of four"
#endif
#define ML_USB_TX_DEFER_CAP (ML_USB_TX_EVENT_QUEUE_SIZE / 4)
