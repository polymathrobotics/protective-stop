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

// 3-09-1: A BOND message from a remote that has control (successful stop/ok
//         sequence) will transition the machine to a stopped state and will
//         require that remote to begin a stop/ok sequence to take control.
// After a successful bonding and stop/ok sequence, send BOND message. Machine
// will transition to a stopped state and will reply with STOP. Sending OK messages
// will result in STOP messages. Send STOP message from remote that had control.
// Machine will stay in stopped. Send OK message from remote that had control
// and machine will transition to OK state.

static
void
req_3_09_test(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);
    configure_app_defaults(&pstop_app, &MACHINE, 0U, 0U, 200U);
    set_operator_allowed(true, false, 1000U); // Allow this device ID

    device_id_t remote = {
        .data = REMOTE_ID
    };

    pstop_msg_t resp;

    TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp, &remote, &MACHINE, 10, 1000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status());

    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp, &remote, &MACHINE, 11, 1500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status());

    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote, &MACHINE, 12, 2000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status());

    const pstop_remote_data_t *remote_data = machine_get_remote_data(&machine, &remote);
    TEST_ASSERT_NOT_NULL(remote_data);
    TEST_ASSERT_EQUAL(PSTOP_REMOTE_OK, remote_data->remote_state);

    // this remote has control
    const robot_state_t *robot = machine_get_robot_state(&machine);

    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_OK, robot->restart_state);
    TEST_ASSERT_EQUAL(robot->remote_stop_id, remote_data->local_remote_id);

    // bond from new clients is still OK
    remote.data = REMOTE_ID + 1;
    pstop_msg_t resp2;
    TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp2, &remote, &MACHINE, 10, 2500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp2.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status());

    // this is a problem and should stop machine
    {
        remote.data = REMOTE_ID;

        // this complete req is still required so we can
        // skip typical bond messages and use previous counter/timestamp
        // This forces the check to the machine for bond handling
        // and skips the black channel error handling
        pstop_msg_t req;
        pstop_message_init(&req);
        req.message = PSTOP_MESSAGE_BOND;
        req.counter = 13;
        req.stamp = 2500;
        device_id_copy(&(req.id), &remote);
        device_id_copy(&(req.receiver_id), &MACHINE);
        req.received_counter = resp.counter;
        req.received_stamp = resp.stamp;
        req.checksum = 10U;
        req.calculated_checksum = 10U;
        TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
        TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    }

    robot = machine_get_robot_state(&machine);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_NEED_STOP, robot->restart_state);

    remote_data = machine_get_remote_data(&machine, &remote);
    TEST_ASSERT_NOT_NULL(remote_data);
    TEST_ASSERT_EQUAL(PSTOP_REMOTE_OK, remote_data->remote_state);
    TEST_ASSERT_EQUAL(robot->remote_stop_id, remote_data->local_remote_id);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status());

    // now do stop/ok sequence to get moving again
    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp, &remote, &MACHINE, 14, 3000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);

    robot = machine_get_robot_state(&machine);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_STOP_RECEIVED, robot->restart_state);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status());

    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote, &MACHINE, 15, 3500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status());

    remote_data = machine_get_remote_data(&machine, &remote);
    TEST_ASSERT_EQUAL(PSTOP_REMOTE_OK, remote_data->remote_state);

    // this remote has control
    robot = machine_get_robot_state(&machine);
    TEST_ASSERT_EQUAL(robot->remote_stop_id, remote_data->local_remote_id);
}

void
main_req_3_09_test(void)
{
    UnitySetTestFile("req_3_09_test.c");

    RUN_TEST(req_3_09_test);
}
