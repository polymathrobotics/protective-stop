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

// 3-04-1: Shall stay in a stopped state until a successful bonding and OK transition.
// Description: Send BOND message, machine should be in stopped state. Send at
// least one STOP message, machine should still be in stopped state. Send OK
// message, machine should transition to OK state.

static
void
req_3_04_test(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);
    configure_app_defaults(&pstop_app, &MACHINE, 0U, 0U, 200U);

    set_operator_allowed(true, false, 5000U); // Allow this device ID

    device_id_t remote_id = {
        .data = REMOTE_ID
    };

    pstop_msg_t resp;

    TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp, &remote_id, &MACHINE, 10, 1000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    // send STOP
    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp, &remote_id, &MACHINE, 11, 1500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    // send another stop
    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp, &remote_id, &MACHINE, 12, 2000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote_id, &MACHINE, 13, 2500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status()); // in an OK state
}

void
main_req_3_04_test(void)
{
    UnitySetTestFile("req_3_04_test.c");

    RUN_TEST(req_3_04_test);
}
