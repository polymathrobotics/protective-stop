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

// 3-23-1: If a stop is received by a remote that is not in control
//         then the remote that was in control is required to send
//         the STOP/OK sequence to transition to an OK state.
// Description: Bond two remotes. First sends STOP/OK to take control.
// Second remote sends STOP. Robot transitions to stopped state. Second
// sends OK, shall reply with STOP. First remote sends OK, shall reply with
// STOP. First sends STOP, shall reply with STOP. First then sends OK,
// shall reply with OK and the robot transitions to OK.

static
void
req_3_23_test(void)
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

    device_id_t remote_id_2 = {
        .data = REMOTE_ID + 1
    };

    // bond remote
    pstop_msg_t resp;
    pstop_msg_t resp2;

    // bond
    TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp, &remote_id, &MACHINE, 10, 1000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp2, &remote_id_2, &MACHINE, 10, 1000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp2.message);

    // stop
    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp, &remote_id, &MACHINE, 11, 1500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    // stop then OK
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote_id, &MACHINE, 12, 2000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status()); // in OK state

    // second sends stop. robot stops
    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp2, &remote_id_2, &MACHINE, 11, 1500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp2.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    // second sends OK, still stopped
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp2, &remote_id_2, &MACHINE, 12, 2000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp2.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in OK state

    // first sends OK, still stopped
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote_id, &MACHINE, 13, 2500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in OK state

    // now start stop/ok sequence
    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp, &remote_id, &MACHINE, 14, 3000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    // stop then OK
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote_id, &MACHINE, 15, 3500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status()); // in OK state
}

void
main_req_3_23_test(void)
{
    UnitySetTestFile("req_3_23_test.c");

    RUN_TEST(req_3_23_test);
}
