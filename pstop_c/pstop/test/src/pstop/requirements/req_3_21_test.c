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

// 3-21-1: Shall stay in a stopped state until all remotes are sending OK messages.
// Description: Bond two remotes. First sends STOP, shall reply with STOP. First
// sends OK, shall reply with OK and transition robot to OK since second has already
// sent BOND heartbeat. Timeout the second remote which will switch the robot
// to a stopped state and unbonds second remote.

static
void
req_3_21_test(void)
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

    // stop then OK. This is fine.
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote_id, &MACHINE, 12, 2000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status());

    set_time(2500); // second remote didn't send OK so we timeout that remote and stop
    // second is now unbonded
    TEST_ASSERT_EQUAL(PSTOP_MISSED_HEARTBEATS, machine_validate_heartbeats(&machine));
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status());

    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp, &remote_id, &MACHINE, 13, 2500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);

    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote_id, &MACHINE, 14, 3000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
}

void
main_req_3_21_test(void)
{
    UnitySetTestFile("req_3_21_test.c");

    RUN_TEST(req_3_21_test);
}
