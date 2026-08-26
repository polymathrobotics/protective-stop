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

// 3-06-1: Shall transition to a stopped state if any bonded remote
//         heartbeat messages are not within the specified machine heartbeat.
// Description: After a successful bonding and stop/ok sequence, send OK message
// slightly slower than the heartbeat timeout, machine should transition
// to a stopped state.

static
void
req_3_06_test(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);
    configure_app_defaults(&pstop_app, &MACHINE, 0U, 0U, 200U);

    set_operator_allowed(true, false, 1000U); // Allow this device ID

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

    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote_id, &MACHINE, 12, 2000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status()); // in an OK state

    const pstop_remote_data_t *remote = machine_get_remote_data(&machine, &remote_id);
    TEST_ASSERT_NOT_NULL(remote);
    const robot_state_t *robot = machine_get_robot_state(&machine);

    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_OK, robot->restart_state);
    TEST_ASSERT_EQUAL(robot->remote_stop_id, remote->local_remote_id);

    set_time(get_time() + 2000U); // now a timeout!
    TEST_ASSERT_EQUAL(PSTOP_MISSED_HEARTBEATS, machine_validate_heartbeats(&machine));
    robot = machine_get_robot_state(&machine);

    TEST_ASSERT_EQUAL(robot->remote_stop_id, 0);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_NEED_STOP, robot->restart_state);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status());
}

void
main_req_3_06_test(void)
{
    UnitySetTestFile("req_3_06_test.c");

    RUN_TEST(req_3_06_test);
}
