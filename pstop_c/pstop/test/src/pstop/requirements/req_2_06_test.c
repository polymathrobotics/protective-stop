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
pstop_application_t pstop_app;

#define MAX_CLIENTS 2U

static pstop_remote_data_t pstop_clients[MAX_CLIENTS];

// 2-06-1: Shall optionally only allow PSTOP messages from known device IDs
// Description: Configure the library to accept only a single known device ID.
// Send a message with device ID that is not the configured ID. This
// message will be thrown away.

static
void
req_2_06_test(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);
    configure_app_defaults(&pstop_app, &MACHINE, 0U, 0U, 1000U);

    set_operator_allowed(false, false, 500U);

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

    TEST_ASSERT_EQUAL(PSTOP_OPERATOR_NOT_ALLOWED, machine_process_message(&machine, &req, &resp));
}

void
main_req_2_06_test(void)
{
    UnitySetTestFile("req_2_06_test.c");

    RUN_TEST(req_2_06_test);
}
