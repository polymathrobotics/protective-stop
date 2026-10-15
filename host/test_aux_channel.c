// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Unit test for the wrapper-only aux-channel codec (common/pstop_aux_channel.h).
// Pure, portable C, no ESP-IDF. Links the pstop_c sources for the role hooks.
// Build: make test_aux_channel

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
    // Encoding the role must not disturb the other protected fields.
    CHECK(m.inverted_value == 0u);
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

static pstop_remote_data_t g_remote_slots[2];

static pstop_msg_t operator_frame(uint32_t sender)
{
  pstop_msg_t m = {.version = PSTOP_VERSION, .message = PSTOP_MESSAGE_OK};
  m.id.data = sender;
  pstop_aux_encode_role(&m, PSTOP_AUX_ROLE_OPERATOR);
  m.inverted_value = pstop_message_calculate_inverted(&m);
  return m;
}

static void test_role_pre_requires_inverted_value(void)
{
  const uint32_t sender = 0x01020381u;
  pstop_machine_t machine;
  memset(&machine, 0, sizeof(machine));
  machine.remotes.remotes = g_remote_slots;
  machine.remotes.max_remotes = 2U;
  pstop_remotes_init(&machine.remotes);
  pstop_remote_data_t * r = pstop_remote_get_free_remote(&machine.remotes);
  CHECK(r != NULL);
  if (r == NULL) return;
  r->remote_data.remote_id.data = sender;
  r->is_stop_only = true;

  // A stuck bit caught only by the inverted value must not promote the remote.
  pstop_msg_t bad = operator_frame(sender);
  bad.inverted_value ^= 1U;
  pstop_aux_apply_role_pre(&machine, &bad);
  CHECK(r->is_stop_only);

  pstop_msg_t good = operator_frame(sender);
  pstop_aux_apply_role_pre(&machine, &good);
  CHECK(!r->is_stop_only);
}

int main(void)
{
  test_role_roundtrip();
  test_operator_helper();
  test_failsafe_unknown_version();
  test_failsafe_bad_role_byte();
  test_zeroed_frame_is_unspecified();
  test_failsafe_unknown_message_version();
  test_role_pre_requires_inverted_value();
  if (g_fail == 0) {
    printf("test_aux_channel: OK\n");
    return 0;
  }
  printf("test_aux_channel: %d failure(s)\n", g_fail);
  return 1;
}
