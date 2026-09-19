// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
//
// Bounded, in-order TX FIFO in front of the USB-NCM tether.
//
// Why: esp_tinyusb's send path has no backpressure. tinyusb_net_send_sync()
// hands the frame to the TinyUSB task, which calls tud_network_can_xmit() ONCE
// and, if the NCM endpoint is still busy with the previous NTB, fails the send
// outright. lwIP then frees the pbuf, so any burst wider than one in-flight
// frame (a 20 KB /state.json response is ~15 frames) lost packets and TCP
// recovered by RTO — the seconds-long stalls seen on the admin UI over USB.
//
// This adapter copies each outgoing frame into a fixed slot ring and drains it
// from a small dedicated task: the head frame is offered with a zero-tick
// timeout; on NCM-busy it is KEPT and retried (2 ms, doubling to 32 ms), in
// order, until its 100 ms lifetime expires. Stock esp_tinyusb / TinyUSB APIs
// only; no patches, no linker wraps.
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
    uint32_t sent; /* frames handed to the NCM endpoint */
    uint32_t busy_retries; /* head offers refused (endpoint busy) and retried */
    uint32_t expired; /* frames dropped after the 100 ms lifetime */
    uint32_t full_drops; /* frames dropped at submit: ring full */
    uint32_t pending; /* frames in the ring right now */
  } ml_usb_tx_diag_t;

  /* Allocate the ring (PSRAM) and start the drain task. Idempotent. */
  esp_err_t ml_usb_tx_init(void);

  /* Enable/disable submission. Disabling starts a new session epoch: every
   * frame still queued (or published concurrently) is discarded by the drain
   * task, counted as expired, and can never reach the next tether session. */
  void ml_usb_tx_set_enabled(int enabled);

  /* Copy one Ethernet frame into the ring. Returns immediately; the caller may
   * free its buffer. ESP_ERR_NO_MEM when the ring is full (frame dropped),
   * ESP_ERR_INVALID_STATE when disabled / USB not mounted. */
  esp_err_t ml_usb_tx_send(const void * buffer, size_t len);

  void ml_usb_tx_get_diag(ml_usb_tx_diag_t * out);

#ifdef __cplusplus
}
#endif
