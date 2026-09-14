// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Classify one recv() result for a length-prefixed stream read. Pure; host
 * tests in host/test_recv_outcome.c. `err` is errno captured immediately after
 * the call; it is only meaningful when n < 0. */

#pragma once

#include <errno.h>
#include <stddef.h>

typedef enum
{
  ML_RECV_OK, /* n > 0 */
  ML_RECV_EOF, /* n == 0: peer closed, errno is stale */
  ML_RECV_WOULD_BLOCK, /* nothing consumed yet: harmless, caller may poll again */
  ML_RECV_RETRY, /* partial unit consumed, retry budget left */
  ML_RECV_ALIGNMENT_LOST, /* partial unit consumed, retry budget exhausted (count, not wall-clock) */
  ML_RECV_ERROR /* other socket error, errno valid */
} ml_recv_outcome_t;

static inline ml_recv_outcome_t ml_recv_classify(int n, int err, size_t consumed, int retries, int max_retries)
{
  if (n > 0) return ML_RECV_OK;
  if (n == 0) return ML_RECV_EOF;
  if (err != EAGAIN && err != EWOULDBLOCK) return ML_RECV_ERROR;
  if (consumed == 0) return ML_RECV_WOULD_BLOCK;
  return retries < max_retries ? ML_RECV_RETRY : ML_RECV_ALIGNMENT_LOST;
}
