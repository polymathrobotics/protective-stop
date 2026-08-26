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

// 3-13-1: Shall reply with OK message after OK request unless in a stopped state.
// Description: After a successful bonding, send an OK message. Should
// respond with OK message. Send a STOP message, shall respond with a
// STOP response. Send an OK message, shall respond with a STOP message.

static
void
req_3_13_test(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);
    configure_app_defaults(&pstop_app, &MACHINE, 0U, 0U, 200U);

    reset_robot_status();

    set_time(1000);
    set_operator_allowed(true, false, 1000U);

    device_id_t remote = {
        .data = REMOTE_ID
    };

    pstop_msg_t resp;

    TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp, &remote, &MACHINE, 10, 1000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    // send OK, but we are in a Stoppped state so will reply with STOP
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote, &MACHINE, 11, 1500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    // start the stop/ok process
    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp, &remote, &MACHINE, 12, 2000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state
    const robot_state_t *robot = machine_get_robot_state(&machine);

    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_STOP_RECEIVED, robot->restart_state);

    // now switches to OK, so OK messages reply with OK
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote, &MACHINE, 13, 2500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status()); // in a stopped state
}

void
main_req_3_13_test(void)
{
    UnitySetTestFile("req_3_13_test.c");

    RUN_TEST(req_3_13_test);
}
