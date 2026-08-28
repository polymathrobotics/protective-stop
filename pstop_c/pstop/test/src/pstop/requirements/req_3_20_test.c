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

#define MAX_CLIENTS 3U

static pstop_remote_data_t pstop_clients[MAX_CLIENTS];

// 3-20-1: After a loss of signal by any remote, any bonded remote that
//         wishes to transition the machine to an OK state must send
//         STOP messages followed by OK heartbeat messages.
// Description: Bond three remotes. First remote takes control with
// STOP/OK sequence. Second and third remotes sends OK. Robot is now in
// OK state. Trigger timeout on second remote. First remote sends OK,
// shall reply with STOP. Third remote sends STOP/OK sequence. Third
// remote should now be in control.

static
void
req_3_20_test(void)
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
    device_id_t remote_id_3 = {
        .data = REMOTE_ID + 2
    };

    // bond remote
    pstop_msg_t resp;
    pstop_msg_t resp2;
    pstop_msg_t resp3;

    // bond all 3
    TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp, &remote_id, &MACHINE, 10, 1000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp2, &remote_id_2, &MACHINE, 10, 1000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp2.message);
    TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp3, &remote_id_3, &MACHINE, 10, 1000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp3.message);

    // stop
    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp, &remote_id, &MACHINE, 11, 1500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    // stop then OK
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote_id, &MACHINE, 12, 2000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status());

    // second, third send OK
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp2, &remote_id_2, &MACHINE, 11, 2000));
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp3, &remote_id_3, &MACHINE, 11, 2000));
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status());

    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote_id, &MACHINE, 13, 2500));
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp3, &remote_id_3, &MACHINE, 12, 2500));

    set_time(3500); // timeout
    TEST_ASSERT_EQUAL(PSTOP_MISSED_HEARTBEATS, machine_validate_heartbeats(&machine));
    // remote_2 is removed
    TEST_ASSERT_NULL(machine_get_remote_data(&machine, &remote_id_2));

    // remote 1 tries ok but it's no longer in control
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote_id, &MACHINE, 14, 3500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);

    // remote 3 takes over with stop/ok
    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp3, &remote_id_3, &MACHINE, 13, 3500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp3.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    // verify robot is attached to remote 3
    const pstop_remote_data_t *remote = machine_get_remote_data(&machine, &remote_id_3);
    TEST_ASSERT_NOT_NULL(remote);
    const robot_state_t *robot = machine_get_robot_state(&machine);

    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_STOP_RECEIVED, robot->restart_state);
    TEST_ASSERT_EQUAL(robot->remote_stop_id, remote->local_remote_id);

    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp3, &remote_id_3, &MACHINE, 14, 4000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp3.message);
}

void
main_req_3_20_test(void)
{
    UnitySetTestFile("req_3_20_test.c");

    RUN_TEST(req_3_20_test);
}
