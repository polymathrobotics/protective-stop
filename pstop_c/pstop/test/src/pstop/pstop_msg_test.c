
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/pstop_msg.h"

#include <unity/unity.h>

static
uint8_t
PSTOP_MSG_BYTES[] = {
    0x00U, 0x02U, // version/message
    0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U, 0x08U, // timestamp
    0x11U, 0x12U, 0x13U, 0x14U, 0x15U, 0x16U, 0x17U, 0x18U, // received timestamp

    0x20U, 0x21U, 0x22U, 0x23U, // device ID

    0x30U, 0x31U, 0x32U, 0x33U, // receiver ID

    0x40U, 0x41U, 0x42U, 0x43U, // heartbeat timeout,

    0x50U, 0x51U, 0x52U, 0x53U, // counter
    0x60U, 0x61U, 0x62U, 0x63U, // received counter

    0x42U, 0xF9U // checksum
};

static
void
decode_pstop_msg()
{
    pstop_msg_t msg;
    pstop_message_decode(&msg, PSTOP_MSG_BYTES);
    TEST_ASSERT_EQUAL(0x00U, msg.version);
    TEST_ASSERT_EQUAL(0x02U, pstop_message_get_message(&msg));
    TEST_ASSERT_EQUAL(0x0807060504030201U,pstop_message_get_stamp(&msg));
    TEST_ASSERT_EQUAL(0x1817161514131211U, msg.received_stamp);

    device_id_t id;
    id.data = 0x23222120U;

    TEST_ASSERT_EQUAL(0, device_id_cmp(&id, &msg.id));

    id.data = 0x33323130U;
    TEST_ASSERT_EQUAL(0, device_id_cmp(&id, &msg.receiver_id));
    TEST_ASSERT_EQUAL(0x43424140U, msg.heartbeat_timeout);
    TEST_ASSERT_EQUAL(0x53525150U, pstop_message_get_counter(&msg));
    TEST_ASSERT_EQUAL(0x63626160U, msg.received_counter);
    TEST_ASSERT_EQUAL(0xF942U, msg.checksum);
}

static
void
encode_pstop_msg()
{
    pstop_msg_t msg;
    msg.version = 0x0U;
    msg.message = 0x2U;
    msg.stamp = 0x0807060504030201U;
    msg.received_stamp = 0x1817161514131211U;
    msg.id.data = 0x23222120U;
    msg.receiver_id.data = 0x33323130U;
    msg.heartbeat_timeout = 0x43424140U;
    msg.counter = 0x53525150U;
    msg.received_counter = 0x63626160U;
    msg.checksum = 0xF942U;

    uint8_t bytes[PSTOP_MESSAGE_SIZE];
    pstop_message_encode(&msg, bytes);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(PSTOP_MSG_BYTES, bytes, PSTOP_MESSAGE_SIZE);
}

static
void
is_pstop_message_valid()
{
    pstop_msg_t msg;
    msg.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, pstop_is_message_valid(&msg));

    msg.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, pstop_is_message_valid(&msg));

    msg.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, pstop_is_message_valid(&msg));

    msg.message = PSTOP_MESSAGE_UNBOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, pstop_is_message_valid(&msg));

    msg.message = PSTOP_MESSAGE_UNKNOWN;
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_TYPE_INVALID, pstop_is_message_valid(&msg));
}

static
void
create_bond_message(void)
{
    device_id_t this_id;
    this_id.data = 0x123456U;
    device_id_t target_id;
    target_id.data = 0x345678U;

    pstop_msg_t msg;
    pstop_create_bond_message(&msg, 123545, &this_id, &target_id, 50);
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, msg.message);
    TEST_ASSERT_EQUAL(123545, msg.stamp);
    TEST_ASSERT_EQUAL(0U, msg.received_stamp);
    TEST_ASSERT_EQUAL(0U, msg.received_counter);
    TEST_ASSERT_EQUAL(50U, msg.counter);
    TEST_ASSERT_EQUAL(0, device_id_cmp(&this_id, &(msg.id)));
    TEST_ASSERT_EQUAL(0, device_id_cmp(&target_id, &(msg.receiver_id)));
}

static
void
create_ok_message(void)
{
    device_id_t this_id;
    this_id.data = 0x123456U;
    device_id_t target_id;
    target_id.data = 0x345678U;

    pstop_msg_t msg;
    pstop_create_ok_message(&msg, 123545, 123300, &this_id, &target_id, 50, 49);
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, msg.message);
    TEST_ASSERT_EQUAL(123545U, msg.stamp);
    TEST_ASSERT_EQUAL(123300U, msg.received_stamp);
    TEST_ASSERT_EQUAL(49U, msg.received_counter);
    TEST_ASSERT_EQUAL(50U, msg.counter);
    TEST_ASSERT_EQUAL(0, device_id_cmp(&this_id, &(msg.id)));
    TEST_ASSERT_EQUAL(0, device_id_cmp(&target_id, &(msg.receiver_id)));
}

static
void
create_stop_message(void)
{
    device_id_t this_id;
    this_id.data = 0x123456U;
    device_id_t target_id;
    target_id.data = 0x345678U;

    pstop_msg_t msg;
    pstop_create_stop_message(&msg, 123545, 123300, &this_id, &target_id, 50, 49);
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, msg.message);
    TEST_ASSERT_EQUAL(123545U, msg.stamp);
    TEST_ASSERT_EQUAL(123300U, msg.received_stamp);
    TEST_ASSERT_EQUAL(49U, msg.received_counter);
    TEST_ASSERT_EQUAL(50U, msg.counter);
    TEST_ASSERT_EQUAL(0, device_id_cmp(&this_id, &(msg.id)));
    TEST_ASSERT_EQUAL(0, device_id_cmp(&target_id, &(msg.receiver_id)));
}

static
void
create_unbond_message(void)
{
    device_id_t this_id;
    this_id.data = 0x123456U;
    device_id_t target_id;
    target_id.data = 0x345678U;

    pstop_msg_t msg;
    pstop_create_unbond_message(&msg, 123545, 123300, &this_id, &target_id, 50, 49);
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, msg.message);
    TEST_ASSERT_EQUAL(123545U, msg.stamp);
    TEST_ASSERT_EQUAL(123300U, msg.received_stamp);
    TEST_ASSERT_EQUAL(49U, msg.received_counter);
    TEST_ASSERT_EQUAL(50U, msg.counter);
    TEST_ASSERT_EQUAL(0, device_id_cmp(&this_id, &(msg.id)));
    TEST_ASSERT_EQUAL(0, device_id_cmp(&target_id, &(msg.receiver_id)));
}

void
main_pstop_msg_test(void)
{
    UnitySetTestFile("pstop_msg_test.c");

    RUN_TEST(decode_pstop_msg);
    RUN_TEST(encode_pstop_msg);
    RUN_TEST(is_pstop_message_valid);
    RUN_TEST(create_bond_message);
    RUN_TEST(create_ok_message);
    RUN_TEST(create_stop_message);
    RUN_TEST(create_unbond_message);
}
