// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/* Host tests for ml_recv_classify() (ml_recv_outcome.h): the boundaries that
 * decide whether a coord stream read is harmless, retryable, or fatal. */

#include <stdio.h>

#include "ml_recv_outcome.h"

static int g_checks = 0;
static int g_fails = 0;
#define CHECK(cond, what)             \
  do {                                \
    g_checks++;                       \
    if (!(cond)) {                    \
      g_fails++;                      \
      printf("  FAIL: %s\n", (what)); \
    }                                 \
  } while (0)

int main(void)
{
  const int MAX = 300;
  CHECK(ml_recv_classify(1, 0, 0, 0, MAX) == ML_RECV_OK, "positive count is progress");
  CHECK(ml_recv_classify(3, EAGAIN, 0, 0, MAX) == ML_RECV_OK, "stale errno ignored when n > 0");
  CHECK(ml_recv_classify(0, EAGAIN, 0, 0, MAX) == ML_RECV_EOF, "n == 0 is EOF even with stale EAGAIN");
  CHECK(ml_recv_classify(0, ECONNRESET, 2, 5, MAX) == ML_RECV_EOF, "n == 0 mid-unit is EOF, not retry");
  CHECK(ml_recv_classify(-1, EAGAIN, 0, 0, MAX) == ML_RECV_WOULD_BLOCK, "EAGAIN with nothing consumed is harmless");
  CHECK(ml_recv_classify(-1, EWOULDBLOCK, 0, 299, MAX) == ML_RECV_WOULD_BLOCK, "EWOULDBLOCK same as EAGAIN");
  CHECK(ml_recv_classify(-1, EAGAIN, 1, 0, MAX) == ML_RECV_RETRY, "partial unit + budget left -> retry");
  CHECK(ml_recv_classify(-1, EAGAIN, 2, 299, MAX) == ML_RECV_RETRY, "last budgeted retry still retries");
  CHECK(ml_recv_classify(-1, EAGAIN, 2, 300, MAX) == ML_RECV_ALIGNMENT_LOST, "budget exhausted mid-unit is fatal");
  CHECK(ml_recv_classify(-1, EAGAIN, 2, 999, MAX) == ML_RECV_ALIGNMENT_LOST, "over budget stays fatal");
  CHECK(ml_recv_classify(-1, ECONNRESET, 0, 0, MAX) == ML_RECV_ERROR, "real socket error with nothing consumed");
  CHECK(ml_recv_classify(-1, EPIPE, 2, 0, MAX) == ML_RECV_ERROR, "real socket error mid-unit");
  CHECK(ml_recv_classify(-1, 0, 0, 0, MAX) == ML_RECV_ERROR, "errno 0 with n < 0 is still an error");
  printf("test_recv_outcome: %d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
