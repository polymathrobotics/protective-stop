// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ml_perf_bench.c
 * @brief On-device micro-benchmarks to A/B build settings (opt level, flash
 * mode) on the SAME silicon. Exposed at GET /api/perf. Measures the crypto
 * the transport actually uses: X25519 (handshake) and the shipped
 * ChaCha20-Poly1305 AEAD (per packet).
 */

#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "mbedtls/chachapoly.h"
#include "x25519.h"

int microlink_perf_bench(int iters, char * out, size_t cap)
{
  if ((iters < 1) || (iters > 200)) {
    iters = 20;
  }
  uint8_t scalar[32], point[32], o[32];
  for (int i = 0; i < 32; i++) {
    scalar[i] = (uint8_t)(0x42u + (unsigned)i * 7u);
    point[i] = (uint8_t)(0x99u ^ ((unsigned)i * 13u));
  }
  point[31] &= 0x7Fu;

  int64_t t0 = esp_timer_get_time();
  for (int i = 0; i < iters; i++) {
    x25519(o, scalar, point, 1);
  }
  int64_t t1 = esp_timer_get_time();

  /* AEAD: encrypt a WG-sized (1420 B) payload, iters times. */
  static uint8_t buf[1420];
  static uint8_t ct[1420];
  uint8_t tag[16];
  uint8_t key[32] = {0}, nonce[12] = {0};
  memset(buf, 0xA5, sizeof(buf));
  mbedtls_chachapoly_context ctx;
  int64_t t2 = esp_timer_get_time();
  for (int i = 0; i < iters; i++) {
    mbedtls_chachapoly_init(&ctx);
    mbedtls_chachapoly_setkey(&ctx, key);
    mbedtls_chachapoly_encrypt_and_tag(&ctx, sizeof(buf), nonce, NULL, 0, buf, ct, tag);
    mbedtls_chachapoly_free(&ctx);
  }
  int64_t t3 = esp_timer_get_time();

  unsigned x_us = (unsigned)((t1 - t0) / iters);
  unsigned aead_us = (unsigned)((t3 - t2) / iters);
  unsigned aead_mbps = (aead_us > 0u) ? (unsigned)((uint64_t)sizeof(buf) * 8u / aead_us) : 0u;

  return snprintf(
    out, cap,
    "{\"iters\":%d,\"x25519_us\":%u,\"aead_1420B_us\":%u,\"aead_Mbps\":%u}",
    iters, x_us, aead_us, aead_mbps);
}
