
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

pstop_application_t pstop_app = {
    .get_time_cb = get_time,
    .machine_device_id.data = "testing",
    .operator_allowed_cb = is_operator_allowed,
    .status_cb = robot_status,
    .log_message_cb = log_error,
    .default_timeout_ms = 60U
};

#define MAX_CLIENTS 2U

pstop_client_data_t pstop_clients[MAX_CLIENTS];

static
void
test_new_client_operator_not_allowed(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    pstop_msg_t msg;
    pstop_message_init(&msg);

    pstop_msg_t resp_obj;
    pstop_message_init(&resp_obj);
    pstop_msg_t *resp = &resp_obj;

    msg.heartbeat_timeout = 50U;
    msg.message = PSTOP_MESSAGE_BOND;

    operator_allowed_flag = 0;
    TEST_ASSERT_EQUAL(PSTOP_OPERATOR_NOT_ALLOWED, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_NOT_NULL(resp);
}

static
void
test_new_client_operator_allowed(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    device_id_set(&id, "test");

    pstop_msg_t msg;
    msg.message = PSTOP_MESSAGE_BOND;
    device_id_copy(&(msg.id), &id);

    pstop_msg_t resp_obj;
    pstop_message_init(&resp_obj);
    pstop_msg_t *resp = &resp_obj;

    operator_allowed_flag = 1;
    current_time = 100U;

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp->message);
}

static
pstop_client_data_t *
client_send_ok(pstop_machine_t *machine, const char *device_id, uint8_t respMsg)
{
    device_id_t id;
    device_id_set(&id, device_id);

    pstop_msg_t msg;
    msg.message = PSTOP_MESSAGE_OK;
    device_id_copy(&(msg.id), &id);

    pstop_msg_t resp_obj;
    pstop_message_init(&resp_obj);
    pstop_msg_t *resp = &resp_obj;

    operator_allowed_flag = 1;
    current_time = 100U;

    TEST_ASSERT_EQUAL(PSTOP_OK, machine->handle_machine_message_cb(machine, &msg, &resp));

    TEST_ASSERT_EQUAL(respMsg, resp_obj.message);

    return pstop_client_get(&(machine->pstops), &id);
}

static
pstop_client_data_t *
bond_client(pstop_machine_t *machine, const char *device_id)
{
    device_id_t id;
    device_id_set(&id, device_id);

    pstop_msg_t msg;
    msg.message = PSTOP_MESSAGE_BOND;
    device_id_copy(&(msg.id), &id);

    pstop_msg_t resp_obj;
    pstop_message_init(&resp_obj);
    pstop_msg_t *resp = &resp_obj;

    operator_allowed_flag = 1;
    current_time = 100U;

    TEST_ASSERT_EQUAL(PSTOP_OK, machine->handle_machine_message_cb(machine, &msg, &resp));

    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp_obj.message);

    return pstop_client_get(&(machine->pstops), &id);
}

static
void
test_bond_req_bond_resp(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    // BOND => BOND
    pstop_client_data_t *client = bond_client(&machine, "test");
    TEST_ASSERT_NOT_NULL(client);
}

static
void
test_ok_req_unbond_resp(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    // OK => UNBOND
    pstop_client_data_t *client = client_send_ok(&machine, "test", PSTOP_MESSAGE_UNBOND);
}

static
void
test_bond_unbond(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    device_id_set(&id, "test");

    pstop_msg_t msg;
    msg.message = PSTOP_MESSAGE_BOND;
    device_id_copy(&(msg.id), &id);

    pstop_msg_t resp_obj;
    pstop_message_init(&resp_obj);
    pstop_msg_t *resp = &resp_obj;

    operator_allowed_flag = 1;
    current_time = 100U;

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp_obj.message);
    TEST_ASSERT_NOT_NULL(pstop_client_get(&(machine.pstops), &id));

    msg.message = PSTOP_MESSAGE_UNBOND;
    pstop_message_init(&resp_obj);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp_obj.message);
    TEST_ASSERT_NULL(pstop_client_get(&(machine.pstops), &id));
    TEST_ASSERT_EQUAL(0U, machine.pstops.num_clients);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, lastStatus);
}

static
void
test_bond_ok_stop(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    device_id_set(&id, "test");

    pstop_msg_t msg;
    msg.message = PSTOP_MESSAGE_BOND;
    device_id_copy(&(msg.id), &id);

    pstop_msg_t resp_obj;
    pstop_message_init(&resp_obj);
    pstop_msg_t *resp = &resp_obj;

    operator_allowed_flag = 1;
    current_time = 100U;

    robot_status_counter = 0;

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp_obj.message);

    TEST_ASSERT_EQUAL(0, robot_status_counter);

    for(int i = 0; i < 10; ++i) {
        msg.message = PSTOP_MESSAGE_OK;
        TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
        TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp_obj.message);
        TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, lastStatus);
        TEST_ASSERT_EQUAL(i + 1, robot_status_counter);
    }

    msg.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp_obj.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, lastStatus);
    TEST_ASSERT_EQUAL(11, robot_status_counter);
}

int
main_machine_test(void)
{
    UnityBegin("machine_test.c");

    RUN_TEST(test_new_client_operator_not_allowed);
    RUN_TEST(test_new_client_operator_allowed);

    RUN_TEST(test_bond_req_bond_resp);
    RUN_TEST(test_ok_req_unbond_resp);
    RUN_TEST(test_bond_unbond);
    RUN_TEST(test_bond_ok_stop);

    return UNITY_END();
}