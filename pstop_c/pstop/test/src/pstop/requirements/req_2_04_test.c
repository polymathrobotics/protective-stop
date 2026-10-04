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

// 2-04-1: Shall be able to detect delayed messages
// Description: Set number of missed heartbeats to 2. Send BOND message
// with timestamp 100. Send OK message with timestamp 200, current
// clock is 200. Then send another OK message with timestamp 900 and
// current clock is 300. This will indicate we missed too many
// messages and will return PSTOP_MSG_LOST.

static
void
req_2_04_1_test(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);
    configure_app_defaults(&pstop_app, &MACHINE, 0U, 2U, 1000U);

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

    set_time(100);
    // succesfull bond
    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    const protocol_data_t *remote = machine_get_protocol_data(&machine, &REMOTE);
    TEST_ASSERT_NOT_NULL(remote);
    uint64_t last_heartbeat = remote->last_timestamp;

    // now send message with a valid stop
    req.message = PSTOP_MESSAGE_OK;
    req.counter = 11;
    req.stamp = 200;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    set_time(200);
    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    remote = machine_get_protocol_data(&machine, &REMOTE);
    TEST_ASSERT_NOT_NULL(remote);
    TEST_ASSERT_NOT_EQUAL(last_heartbeat, remote->last_timestamp);
    last_heartbeat = remote->last_timestamp;

    // now send message with timestamp too far in the future
    req.message = PSTOP_MESSAGE_OK;
    req.counter = 12;
    req.stamp = 900;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    set_time(300);
    TEST_ASSERT_EQUAL(PSTOP_MSG_LOST, machine_process_message(&machine, &req, &resp));
    remote = machine_get_protocol_data(&machine, &REMOTE);
    TEST_ASSERT_NOT_NULL(remote);
    TEST_ASSERT_EQUAL(last_heartbeat, remote->last_timestamp);
}

void
main_req_2_04_test(void)
{
    UnitySetTestFile("req_2_04_test.c");

    RUN_TEST(req_2_04_1_test);
}
