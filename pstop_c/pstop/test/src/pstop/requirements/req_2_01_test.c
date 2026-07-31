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

// 2-01-1: Shall be able to detect message corruption
// Description: After a successful bonding, a message shall be created where
// the checksum does not match the CRC-16 checksum that is in the message.
// The message shall be ignored. The last heartbeat timestamp shall not
// change for this device ID.
static
void
req_2_01_test(void)
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
    req.checksum = 10U;
    req.calculated_checksum = 10U;

    pstop_msg_t resp;
    pstop_message_init(&resp);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    const protocol_data_t *remote = machine_get_protocol_data(&machine, &REMOTE);
    TEST_ASSERT_NOT_NULL(remote);
    uint64_t last_heartbeat = remote->last_timestamp;

    req.message = PSTOP_MESSAGE_STOP;
    req.counter = 11;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;

    req.calculated_checksum = 14U; // usually calculated by pstop_message_decode
    req.checksum = 10U;

    TEST_ASSERT_EQUAL(PSTOP_MSG_INVALID_CHECKSUM, machine_process_message(&machine, &req, &resp));
    remote = machine_get_protocol_data(&machine, &REMOTE);
    TEST_ASSERT_NOT_NULL(remote);
    TEST_ASSERT_EQUAL(last_heartbeat, remote->last_timestamp);
}

void
main_req_2_01_test(void)
{
    UnitySetTestFile("req_2_01_test.c");

    RUN_TEST(req_2_01_test);
}
