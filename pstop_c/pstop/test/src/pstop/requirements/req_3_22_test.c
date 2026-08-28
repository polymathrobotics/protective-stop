// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/protocol.h"

#include <unity/unity.h>

#include "pstop/checksum.h"
#include "pstop/requirements/test_utils.h"

#define MACHINE_ID 1236
#define REMOTE_ID 1234
#define REMOTE_2_ID 1235

static
device_id_t MACHINE = {
    .data = MACHINE_ID
};

static
pstop_application_t pstop_app;

#define MAX_CLIENTS 2U

static pstop_remote_data_t pstop_clients[MAX_CLIENTS];

// 3-22-1: If an operator is in control and unbonds then the machine
//         transitions to a stopped state and another operator must
//         send the STOP/OK sequence to take control.
// Description: After a successful bonding, send STOP/OK sequence to take
// control. Now unbond remote. Robot shall transition to STOP state. Bond
// second remote. Send STOP/OK sequence and robot should transition to
// OK state and second remote has control.

static
void
req_3_22_test(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);
    configure_app_defaults(&pstop_app, &MACHINE, 0U, 0U, 200U);

    reset_robot_status();

    set_time(1000);
    set_operator_allowed(true, false, 1000U);

    device_id_t remote_id = {
        .data = REMOTE_ID
    };

    // bond remote
    pstop_msg_t resp;

    // bond
    TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp, &remote_id, &MACHINE, 10, 1000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    // stop
    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp, &remote_id, &MACHINE, 11, 1500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    // stop then OK
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote_id, &MACHINE, 12, 2000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status()); // in OK state

    // now this operator unbonds and robot transitions to stopped state
    TEST_ASSERT_EQUAL(PSTOP_OK, send_unbond(&machine, &resp, &remote_id, &MACHINE, 13, 2500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status());

    device_id_t remote_id_2 = {
        .data = REMOTE_ID
    };

    // second remote takes control
    TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp, &remote_id_2, &MACHINE, 10, 3000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp, &remote_id_2, &MACHINE, 11, 3500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    // stop then OK
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote_id_2, &MACHINE, 12, 4000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status()); // in OK state
}

void
main_req_3_22_test(void)
{
    UnitySetTestFile("req_3_22_test.c");

    RUN_TEST(req_3_22_test);
}
