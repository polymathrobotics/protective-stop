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

// 3-18-1: A remote is in control and sends a STOP message then that
//         remote is the only remote that can  transition a machine
//         to the OK state by sending OK heartbeat messages.
// Description: Bond a remote and send STOP/OK sequence. Bond a second remote.
// Send STOP on the first remote and verify the robot is in a stopped state.
// Send OK on the second remote and verify robot is still stopped. Then send
// OK on the first remote and verify the robot is now in an OK state.

static
void
req_3_18_test(void)
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

    pstop_msg_t resp2;
    // bond 2nd
    TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp2, &remote_id_2, &MACHINE, 10, 2000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp2.message);

    // first sends stop
    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp, &remote_id, &MACHINE, 13, 2500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    const pstop_remote_data_t *remote = machine_get_remote_data(&machine, &remote_id);
    TEST_ASSERT_NOT_NULL(remote);
    const robot_state_t *robot = machine_get_robot_state(&machine);

    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_STOP_RECEIVED, robot->restart_state);
    TEST_ASSERT_EQUAL(robot->remote_stop_id, remote->local_remote_id);

    // second sends OK, still stopped
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp2, &remote_id_2, &MACHINE, 11, 2500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status());

    // second sends stop, but it's not in control
    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp2, &remote_id_2, &MACHINE, 12, 3000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status());

    robot = machine_get_robot_state(&machine);

    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_NEED_STOP, robot->restart_state);
    TEST_ASSERT_EQUAL(robot->remote_stop_id, remote->local_remote_id);

    // seconds sends ok, but it's not in control so still stopped
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp2, &remote_id_2, &MACHINE, 13, 3500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status());

    // first sends stop to start sequence
    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp, &remote_id, &MACHINE, 14, 3000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    // firest sends OK, should be started
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote_id, &MACHINE, 15, 3500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status()); // in OK state
}

void
main_req_3_18_test(void)
{
    UnitySetTestFile("req_3_18_test.c");

    RUN_TEST(req_3_18_test);
}
