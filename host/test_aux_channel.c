// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Unit test for the wrapper-only aux-channel codec (common/pstop_aux_channel.h).
// Pure, portable C — no ESP-IDF, no pstop_c sources (all codec fns are static
// inline). Build: make test_aux_channel

#include <stdio.h>
#include <string.h>

#include "pstop_aux_channel.h"

static int g_fail;

#define CHECK(cond)                                          \
  do {                                                       \
    if (!(cond)) {                                           \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      g_fail++;                                              \
    }                                                        \
  } while (0)

static void test_role_roundtrip(void)
{
  const pstop_aux_role_t roles[] = {
    PSTOP_AUX_ROLE_UNSPECIFIED,
    PSTOP_AUX_ROLE_STOP_ONLY,
    PSTOP_AUX_ROLE_OPERATOR,
  };
  for (size_t i = 0; i < sizeof(roles) / sizeof(roles[0]); ++i) {
    pstop_msg_t m = {.version = PSTOP_VERSION};
    pstop_aux_encode_role(&m, roles[i]);
    CHECK(pstop_aux_decode_role(&m) == roles[i]);
    // Encoding the role must not disturb the other padding field.
    CHECK(m.padding2 == 0u);
  }
}

static void test_operator_helper(void)
{
  CHECK(pstop_aux_role_is_operator(PSTOP_AUX_ROLE_OPERATOR));
  CHECK(!pstop_aux_role_is_operator(PSTOP_AUX_ROLE_STOP_ONLY));
  CHECK(!pstop_aux_role_is_operator(PSTOP_AUX_ROLE_UNSPECIFIED));
}

static void test_failsafe_unknown_version(void)
{
  // Wrong uplink version => UNSPECIFIED (defers to policy, never operator).
  pstop_msg_t m = {.version = PSTOP_VERSION};
  m.padding1 = 0x00000200u | 0x00000099u;  // version 0x99, role byte = OPERATOR
  CHECK(pstop_aux_decode_role(&m) == PSTOP_AUX_ROLE_UNSPECIFIED);
}

static void test_failsafe_bad_role_byte(void)
{
  // Recognized version but an out-of-range role byte => STOP_ONLY.
  pstop_msg_t m = {.version = PSTOP_VERSION};
  m.padding1 = (uint32_t)PSTOP_AUX_UP_VERSION | ((uint32_t)0x7Fu << PSTOP_AUX_UP_ROLE_SHIFT);
  CHECK(pstop_aux_decode_role(&m) == PSTOP_AUX_ROLE_STOP_ONLY);
}

static void test_zeroed_frame_is_unspecified(void)
{
  // An all-zero padding (unprovisioned/no announcement) is UNSPECIFIED, which
  // the AND-rule denies arming. This is the fail-safe default.
  pstop_msg_t m = {0};
  CHECK(pstop_aux_decode_role(&m) == PSTOP_AUX_ROLE_UNSPECIFIED);
}

static void test_failsafe_unknown_message_version(void)
{
  pstop_msg_t m = {.version = 0x99u};
  pstop_aux_encode_role(&m, PSTOP_AUX_ROLE_OPERATOR);
  CHECK(pstop_aux_decode_role(&m) == PSTOP_AUX_ROLE_UNSPECIFIED);
}

int main(void)
{
  test_role_roundtrip();
  test_operator_helper();
  test_failsafe_unknown_version();
  test_failsafe_bad_role_byte();
  test_zeroed_frame_is_unspecified();
  test_failsafe_unknown_message_version();
  if (g_fail == 0) {
    printf("test_aux_channel: OK\n");
    return 0;
  }
  printf("test_aux_channel: %d failure(s)\n", g_fail);
  return 1;
}
