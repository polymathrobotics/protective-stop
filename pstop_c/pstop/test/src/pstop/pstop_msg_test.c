
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/pstop_msg.h"

#include <unity/unity.h>

static
uint8_t
PSTOP_MSG_BYTES[] = {
    0x02U, // version/message
    0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U, 0x08U, // timestamp
    0x11U, 0x12U, 0x13U, 0x14U, 0x15U, 0x16U, 0x17U, 0x18U, // received timestamp

    0x20U, 0x21U, 0x22U, 0x23U, 0x24U, 0x25U, 0x26U, 0x27U,
    0x28U, 0x29U, 0x2AU, 0x2BU, 0x2CU, 0x2DU, 0x2EU, 0x2FU, // device UUID

    0x30U, 0x31U, 0x32U, 0x33U, 0x34U, 0x35U, 0x36U, 0x37U,
    0x38U, 0x39U, 0x3AU, 0x3BU, 0x3CU, 0x3DU, 0x3EU, 0x3FU, // receiver UUID

    0x40U, 0x41U, 0x42U, 0x43U, // heartbeat timeout,

    0x50U, 0x51U, 0x52U, 0x53U, // counter
    0x60U, 0x61U, 0x62U, 0x63U, // received counter

    0x70U, 0x71U // checksum
};

static
void
decode_pstop_msg()
{
    pstop_msg_t msg;
    pstop_message_decode(&msg, PSTOP_MSG_BYTES);
    TEST_ASSERT_EQUAL(0x00U, msg.version);
    TEST_ASSERT_EQUAL(0x02U, msg.message);
    TEST_ASSERT_EQUAL(0x0807060504030201U, msg.stamp);
    TEST_ASSERT_EQUAL(0x1817161514131211U, msg.received_stamp);

    uint8_t ID_BYTES[] = {
        0x20U, 0x21U, 0x22U, 0x23U, 0x24U, 0x25U, 0x26U, 0x27U,
        0x28U, 0x29U, 0x2AU, 0x2BU, 0x2CU, 0x2DU, 0x2EU, 0x2FU // device UUID
    };
    device_id_t id;
    device_id_set_bytes(&id, ID_BYTES);

    TEST_ASSERT_EQUAL(0, device_id_cmp(&id, &msg.id));

    uint8_t RECEIVER_ID_BYTES[] = {
        0x30U, 0x31U, 0x32U, 0x33U, 0x34U, 0x35U, 0x36U, 0x37U,
        0x38U, 0x39U, 0x3AU, 0x3BU, 0x3CU, 0x3DU, 0x3EU, 0x3FU // receiver UUID
    };
    device_id_set_bytes(&id, RECEIVER_ID_BYTES);
    TEST_ASSERT_EQUAL(0, device_id_cmp(&id, &msg.receiver_id));
    TEST_ASSERT_EQUAL(0x43424140U, msg.heartbeat_timeout);
    TEST_ASSERT_EQUAL(0x53525150U, msg.counter);
    TEST_ASSERT_EQUAL(0x63626160U, msg.received_counter);
    TEST_ASSERT_EQUAL(0x7170U, msg.checksum);
}

static
void
encode_pstop_msg()
{
    pstop_msg_t msg;
    msg.version = 0x0U;
    msg.message = 0x3U;
    msg.stamp = 0x0807060504030201U;
    msg.received_stamp = 0x1817161514131211U;

    msg.heartbeat_timeout = 0x43424140U;
    msg.counter = 0x53525150U;
    msg.received_counter = 0x63626160U;
    msg.checksum = 0x7170U;

    uint8_t bytes[PSTOP_MESSAGE_SIZE];
    pstop_message_encode(&msg, bytes);
}

void
main_pstop_msg_test(void)
{
    UnitySetTestFile("pstop_msg_test.c");

    RUN_TEST(decode_pstop_msg);
    RUN_TEST(encode_pstop_msg);
}
