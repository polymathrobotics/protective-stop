
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/machine.h"

#include <unity/unity.h>

#include "pstop/time.h"
#include "pstop/test_utils.h"

static uint64_t current_time;

static
uint64_t
get_time(void)
{
    return current_time++;
}

static int operator_allowed_flag;

static
int
is_operator_allowed(const device_id_t *id)
{
    return operator_allowed_flag;
}

static pstop_status_message_t lastStatus = PSTOP_STATUS_OK;
static int robot_status_counter = 0;

static
int
robot_status(pstop_status_message_t status)
{
    lastStatus = status;

    robot_status_counter++;

    return 0;
}

static
void
log_error(pstop_error_t error, const char *message)
{

}

static
pstop_application_t pstop_app = {
    .get_time_cb = get_time,
    .machine_device_id.data = "testing",
    .operator_allowed_cb = is_operator_allowed,
    .status_cb = robot_status,
    .log_message_cb = log_error,
    .app_config.default_timeout_ms = 60U,
    .app_config.max_lost_messages = 1U,
    .app_config.max_missed_heartbeats = 1U
};

#define MAX_CLIENTS 2U

static pstop_client_data_t pstop_clients[MAX_CLIENTS];

static
void
init_client(device_id_t *id1, pstop_msg_t *msg, const char *id)
{
    device_id_set(id1, id);
    device_id_copy(&(msg->id), id1);
}

static
void
test_bond_timeout(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    pstop_msg_t msg;
    init_client(&id, &msg, "test1");

    pstop_msg_t resp_obj;
    pstop_message_init(&resp_obj);
    pstop_msg_t *resp = &resp_obj;

    operator_allowed_flag = 1;
    current_time = 100U;

    robot_status_counter = 0;

    msg.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp_obj.message);

    current_time = 150U; // no timeout yet!
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.check_heartbeats_cb(&machine));

    current_time = 170U; // now a timeout!
    TEST_ASSERT_EQUAL(PSTOP_MISSED_HEARTBEATS, machine.check_heartbeats_cb(&machine));
    TEST_ASSERT_EQUAL(PSTOP_CLIENT_UNKNOWN, pstop_clients[0].client_state);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, lastStatus);

    // after the previous timeout that client is marked as UNKNOWN so no more clients connected
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.check_heartbeats_cb(&machine));
}

// client has been bonded and started the stop/ok cycle but timesout before OK
static
void
test_bond_stop_timeout(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    pstop_msg_t msg;
    init_client(&id, &msg, "test1");

    pstop_msg_t resp_obj;
    pstop_message_init(&resp_obj);
    pstop_msg_t *resp = &resp_obj;

    operator_allowed_flag = 1;
    current_time = 100U;

    robot_status_counter = 0;

    msg.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp_obj.message);

    msg.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp_obj.message);

    current_time = 170U; // now a timeout!
    TEST_ASSERT_EQUAL(PSTOP_MISSED_HEARTBEATS, machine.check_heartbeats_cb(&machine));
}

static
void
test_bond_stop_timeout_2_missed_timeouts(void)
{
    pstop_machine_t machine;

    // require 2 missed heartbeats before failing
    pstop_app.app_config.max_missed_heartbeats = 2U;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    pstop_msg_t msg;
    init_client(&id, &msg, "test1");

    pstop_msg_t resp_obj;
    pstop_message_init(&resp_obj);
    pstop_msg_t *resp = &resp_obj;

    operator_allowed_flag = 1;
    current_time = 100U;

    robot_status_counter = 0;

    msg.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp_obj.message);

    msg.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp_obj.message);

    current_time = 170U; // now a missed heartbeat
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.check_heartbeats_cb(&machine));

    current_time = 222U; // now two missed heartbeats
    TEST_ASSERT_EQUAL(PSTOP_MISSED_HEARTBEATS, machine.check_heartbeats_cb(&machine));
}

void
main_machine_timeout_test(void)
{
    UnitySetTestFile("machine_timeout_test.c");

    RUN_TEST(test_bond_timeout);
    RUN_TEST(test_bond_stop_timeout);
    RUN_TEST(test_bond_stop_timeout_2_missed_timeouts);
}
