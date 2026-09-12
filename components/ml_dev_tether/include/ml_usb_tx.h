// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    uint32_t submitted;
    uint32_t copied;
    uint32_t cancelled;
    uint32_t errors;
    uint32_t rejected_unavailable;
    uint32_t unmounted_drop;
    uint32_t exhausted;
    uint32_t defer_full;
    uint32_t can_xmit_fail;
    uint32_t integrity_errors;
    uint32_t defer_cap;
    uint32_t pending;
    uint32_t callbacks;
    uint32_t callback_age_ms; /* TX-callback progress, not an idle-task heartbeat */
    uint32_t latency[4]; /* enqueue->callback: <10, <50, <100, >=100 ms */
    uint32_t used;
    uint32_t oldest_ms;
  } ml_usb_tx_diag_t;

  /* Called by the tether owner before bringing up the netif. Resources remain
   * resident across netif stop/start so queued USB callbacks never dangle. */
  esp_err_t ml_usb_tx_init(void);
  void ml_usb_tx_set_enabled(int enabled);

  /* Nonblocking owned-copy enqueue. ESP_OK means accepted, not wire delivery.
   * The callback discards requests older than 100ms. One producer and the USB
   * FIFO preserve order; expired/unavailable requests are dropped, never retried.
   * All deferred drops are counted and must be monitored during qualification. */
  esp_err_t ml_usb_tx_send(const void * buffer, size_t len);
  void ml_usb_tx_get_diag(ml_usb_tx_diag_t * out);
  void ml_usb_tx_note_ownership_error(void);

#ifdef __cplusplus
}
#endif
