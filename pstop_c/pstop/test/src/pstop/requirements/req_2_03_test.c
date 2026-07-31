// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/protocol.h"

#include <unity/unity.h>

#include "pstop/checksum.h"
#include "pstop/requirements/test_utils.h"

#define MACHINE_ID 1236
#define PSTOP_ID 1234

static
device_id_t MACHINE = {
    .data = MACHINE_ID
};

static
device_id_t REMOTE = {
    .data = PSTOP_ID
};

static
pstop_application_t pstop_app;

#define MAX_CLIENTS 2U

static pstop_remote_data_t pstop_clients[MAX_CLIENTS];

// 2-03-1: Shall be able to detect lost messages
// Description: After a successful bonding, send a normal heartbeat message,
// then send another message with a counter that is increased by more than
// the max number of lost messages to indicate a lost message and with a
// timestamp still within the heartbeat timeout. A failure will be indicated
// because too many lost messages.

static
void
req_2_03_1_test(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);
    configure_app_defaults(&pstop_app, &MACHINE, 0U, 0U, 1000U);

    set_operator_allowed(true, false, 500U);

    pstop_msg_t req;
    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_BOND;
    req.counter = 10;
    req.stamp = 100;
    req.id.data = PSTOP_ID;
    req.receiver_id.data = MACHINE_ID;
    req.received_counter = 0U;
    req.received_stamp = 0U;

    pstop_msg_t resp;
    pstop_message_init(&resp);

    // succesfull bond
    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    const protocol_data_t *remote = machine_get_protocol_data(&machine, &REMOTE);
    TEST_ASSERT_NOT_NULL(remote);
    uint64_t last_heartbeat = remote->last_timestamp;

    // now send message with a valid stop
    req.message = PSTOP_MESSAGE_STOP;
    req.counter = 11;
    req.stamp = 110;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    remote = machine_get_protocol_data(&machine, &REMOTE);
    TEST_ASSERT_NOT_NULL(remote);
    TEST_ASSERT_NOT_EQUAL(last_heartbeat, remote->last_timestamp);
    last_heartbeat = remote->last_timestamp;

    // now send message with a counter too far in the future
    req.message = PSTOP_MESSAGE_STOP;
    req.counter = 14;
    req.stamp = 110;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    TEST_ASSERT_EQUAL(PSTOP_MSG_LOST, machine_process_message(&machine, &req, &resp));
    remote = machine_get_protocol_data(&machine, &REMOTE);
    TEST_ASSERT_NOT_NULL(remote);
    TEST_ASSERT_EQUAL(last_heartbeat, remote->last_timestamp);
}

// 2-03-2: Shall be able to detect lost messages
// Description:  After a successful bonding, send a normal heartbeat message,
// then send another message with a counter that is increased by less than the
// max number of lost messages to indicate a lost message and with a timestamp
// still within the heartbeat timeout. Because the timestamp is still within the
// expected heartbeat range and the counter is still within the max number of missed
// messages, this will still be a valid message and the heartbeat timestamp shall
// change to the new message timestamp.

static
void
req_2_03_2_test(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);
    configure_app_defaults(&pstop_app, &MACHINE, 1U, 0U, 1000U);

    set_operator_allowed(true, false, 500U);

    pstop_msg_t req;
    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_BOND;
    req.counter = 10;
    req.stamp = 100;
    req.id.data = PSTOP_ID;
    req.receiver_id.data = MACHINE_ID;
    req.received_counter = 0U;
    req.received_stamp = 0U;
    req.calculated_checksum = 0U;
    req.checksum = 0U;

    pstop_msg_t resp;
    pstop_message_init(&resp);

    // succesfull bond
    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    const protocol_data_t *remote = machine_get_protocol_data(&machine, &REMOTE);
    TEST_ASSERT_NOT_NULL(remote);
    uint64_t last_heartbeat = remote->last_timestamp;

    // now send message with a valid stop
    req.message = PSTOP_MESSAGE_STOP;
    req.counter = 11;
    req.stamp = 110;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    remote = machine_get_protocol_data(&machine, &REMOTE);
    TEST_ASSERT_NOT_NULL(remote);
    TEST_ASSERT_NOT_EQUAL(last_heartbeat, remote->last_timestamp);
    last_heartbeat = remote->last_timestamp;

    // now send message with counter that is just within the range of missed messages
    req.message = PSTOP_MESSAGE_STOP;
    req.counter = 13;
    req.stamp = 1000;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    remote = machine_get_protocol_data(&machine, &REMOTE);
    TEST_ASSERT_NOT_NULL(remote);
    TEST_ASSERT_NOT_EQUAL(last_heartbeat, remote->last_timestamp);
}

void
main_req_2_03_test(void)
{
    UnitySetTestFile("req_2_03_test.c");

    RUN_TEST(req_2_03_1_test);
    RUN_TEST(req_2_03_2_test);
}
