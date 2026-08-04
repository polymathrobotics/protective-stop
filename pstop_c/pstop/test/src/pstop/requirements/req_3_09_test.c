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

    pstop_msg_t req;
    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_BOND;
    req.counter = 10;
    req.stamp = 1000;
    req.id.data = REMOTE_ID;
    req.receiver_id.data = MACHINE_ID;
    req.received_counter = 0U;
    req.received_stamp = 0U;
    req.checksum = 10U;
    req.calculated_checksum = 10U;

    pstop_msg_t resp;
    pstop_message_init(&resp);

    set_operator_allowed(true, false, 1000U); // Allow this device ID

    set_time(1000);
    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status());

    req.message = PSTOP_MESSAGE_STOP;
    req.counter++;
    req.stamp++;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status());

    set_time(get_time() + 1001U);
    req.message = PSTOP_MESSAGE_OK;
    req.counter++;
    req.stamp += 1001U;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status());

    const pstop_remote_data_t *remote = machine_get_remote_data(&machine, &(req.id));
    TEST_ASSERT_NOT_NULL(remote);
    TEST_ASSERT_EQUAL(PSTOP_REMOTE_OK, remote->remote_state);

    // this remote has control
    const robot_state_t *robot = machine_get_robot_state(&machine);

    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_OK, robot->restart_state);
    TEST_ASSERT_EQUAL(robot->remote_stop_id, remote->local_remote_id);

    // bond from new clients is still OK
    pstop_msg_t req2;
    pstop_message_init(&req2);
    req2.message = PSTOP_MESSAGE_BOND;
    req2.counter = 10;
    req2.stamp = 1000;
    req2.id.data = REMOTE_ID + 1;
    req2.receiver_id.data = MACHINE_ID;
    req2.received_counter = 0U;
    req2.received_stamp = 0U;
    req2.checksum = 10U;
    req2.calculated_checksum = 10U;

    pstop_msg_t resp2;
    pstop_message_init(&resp2);
    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req2, &resp2));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp2.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status());

    // this is a problem and should stop machine
    req.message = PSTOP_MESSAGE_BOND;
    req.counter++;
    req.stamp++;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);

    robot = machine_get_robot_state(&machine);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_NEED_STOP, robot->restart_state);

    remote = machine_get_remote_data(&machine, &(req.id));
    TEST_ASSERT_NOT_NULL(remote);
    TEST_ASSERT_EQUAL(PSTOP_REMOTE_OK, remote->remote_state);
    TEST_ASSERT_EQUAL(robot->remote_stop_id, remote->local_remote_id);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status());

    // now do stop/ok sequence to get moving again
    req.message = PSTOP_MESSAGE_STOP;
    req.counter++;
    req.stamp++;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);

    robot = machine_get_robot_state(&machine);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_STOP_RECEIVED, robot->restart_state);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status());

    req.message = PSTOP_MESSAGE_OK;
    req.counter++;
    req.stamp += 1001U;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    set_time(get_time() + 1001U);
    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status());

    remote = machine_get_remote_data(&machine, &(req.id));
    TEST_ASSERT_EQUAL(PSTOP_REMOTE_OK, remote->remote_state);

    // this remote has control
    robot = machine_get_robot_state(&machine);
    TEST_ASSERT_EQUAL(robot->remote_stop_id, remote->local_remote_id);
}

void
main_req_3_09_test(void)
{
    UnitySetTestFile("req_3_09_test.c");

    RUN_TEST(req_3_09_test);
}
