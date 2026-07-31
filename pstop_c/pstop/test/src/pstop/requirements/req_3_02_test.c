// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/protocol.h"

#include <unity/unity.h>

#include "pstop/checksum.h"
#include "pstop/requirements/test_utils.h"

#define MACHINE_ID 1236
#define REMOTE_ID 1234

static
device_id_t MACHINE = {
    .data = MACHINE_ID
};

static
pstop_application_t pstop_app;

#define MAX_CLIENTS 2U

static pstop_remote_data_t pstop_clients[MAX_CLIENTS];

// 3-02-1: Shall provide a way to configure allowable remotes
// Description: The library shall have a way to configure which operators are
// allowed to connect to the machine under control. Configure the library so that
// it will allow a certain device ID and reject all others. Send a bond message
// with the first device ID and validate that a successful bonding has occurred.
// Then send a bond message with the a different device ID and validate that
// the bonding is rejected.

static
void
req_3_02_test(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);
    configure_app_defaults(&pstop_app, &MACHINE, 0U, 0U, 1000U);

    pstop_msg_t req;
    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_BOND;
    req.counter = 10;
    req.stamp = 100;
    req.id.data = REMOTE_ID;
    req.receiver_id.data = MACHINE_ID;
    req.received_counter = 0U;
    req.received_stamp = 0U;
    req.checksum = 10U;
    req.calculated_checksum = 10U;

    pstop_msg_t resp;
    pstop_message_init(&resp);

    set_operator_allowed(true, false, 5000U); // Allow this device ID

    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_BOND;
    req.counter = 10;
    req.stamp = 100;
    req.id.data = REMOTE_ID + 1;
    req.receiver_id.data = MACHINE_ID;
    req.received_counter = 0U;
    req.received_stamp = 0U;
    req.checksum = 10U;
    req.calculated_checksum = 10U;

    pstop_message_init(&resp);

    set_operator_allowed(false, false, 10000U); // Disallow this device ID

    TEST_ASSERT_EQUAL(PSTOP_OPERATOR_NOT_ALLOWED, machine_process_message(&machine, &req, &resp));
}

void
main_req_3_02_test(void)
{
    UnitySetTestFile("req_3_02_test.c");

    RUN_TEST(req_3_02_test);
}
