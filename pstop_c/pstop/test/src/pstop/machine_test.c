
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
    .app_config.default_timeout_ms = 60U,
    .app_config.max_lost_messages = 1U,
    .app_config.max_missed_heartbeats = 1U
};

#define MAX_CLIENTS 2U

pstop_client_data_t pstop_clients[MAX_CLIENTS];

static
void
init_client(device_id_t *id1, pstop_msg_t *msg, const char *id)
{
    device_id_set(id1, id);
    device_id_copy(&(msg->id), id1);
}

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
    pstop_msg_t msg;

    init_client(&id, &msg, device_id);

    pstop_msg_t resp_obj;
    pstop_message_init(&resp_obj);
    pstop_msg_t *resp = &resp_obj;

    operator_allowed_flag = 1;
    current_time = 100U;

    msg.message = PSTOP_MESSAGE_BOND;
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
    pstop_msg_t msg;

    init_client(&id, &msg, "test");

    pstop_msg_t resp_obj;
    pstop_message_init(&resp_obj);
    pstop_msg_t *resp = &resp_obj;

    operator_allowed_flag = 1;
    current_time = 100U;

    msg.message = PSTOP_MESSAGE_BOND;
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

// send bond then ok. Since no stop/ok cycle has been
// completed, replies with STOP
static
void
test_bond_ok(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    pstop_msg_t msg;

    init_client(&id, &msg, "test");

    pstop_msg_t resp_obj;
    pstop_message_init(&resp_obj);
    pstop_msg_t *resp = &resp_obj;

    operator_allowed_flag = 1;
    current_time = 100U;

    msg.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp_obj.message);

    msg.message = PSTOP_MESSAGE_OK;

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp_obj.message);
}

static
void
test_bond_bond(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    pstop_msg_t msg;

    init_client(&id, &msg, "test");

    pstop_msg_t resp_obj;
    pstop_message_init(&resp_obj);
    pstop_msg_t *resp = &resp_obj;

    operator_allowed_flag = 1;
    current_time = 100U;

    msg.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp_obj.message);
    TEST_ASSERT_NOT_NULL(pstop_client_get(&(machine.pstops), &id));

    // already bonded, don't rebond
    msg.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp_obj.message);
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

    msg.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp_obj.message);

    for(int i = 0; i < 10; ++i) {
        msg.message = PSTOP_MESSAGE_OK;
        TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
        TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp_obj.message);
        TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, lastStatus);
        TEST_ASSERT_EQUAL(i + 2, robot_status_counter);
    }

    msg.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp_obj.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, lastStatus);
    TEST_ASSERT_EQUAL(12, robot_status_counter);
}

static
void
test_2_clients(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id1, id2;
    pstop_msg_t msg1, msg2;

    init_client(&id1, &msg1, "test1");
    init_client(&id2, &msg2, "test2");

    pstop_msg_t resp_obj;
    pstop_message_init(&resp_obj);
    pstop_msg_t *resp = &resp_obj;

    operator_allowed_flag = 1;
    current_time = 100U;

    robot_status_counter = 0;

    // bond both nodes
    msg1.message = PSTOP_MESSAGE_BOND;
    msg2.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp_obj.message);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp_obj.message);

    // first node sends stop
    msg1.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp_obj.message);

    // second node sends OK, should reply stop
    msg2.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp_obj.message);
}

static
void
test_2_clients_stop_unbond(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id1, id2;
    pstop_msg_t msg1, msg2;

    init_client(&id1, &msg1, "test1");
    init_client(&id2, &msg2, "test2");

    pstop_msg_t resp_obj;
    pstop_message_init(&resp_obj);
    pstop_msg_t *resp = &resp_obj;

    operator_allowed_flag = 1;
    current_time = 100U;

    robot_status_counter = 0;

    // bond both nodes
    msg1.message = PSTOP_MESSAGE_BOND;
    msg2.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp_obj.message);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp_obj.message);

    // first node sends stop
    msg1.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp_obj.message);

    // then first node sends unbond
    msg1.message = PSTOP_MESSAGE_UNBOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp_obj.message);

    // second node sends STOP, then OK
    msg2.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp_obj.message);

    msg2.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp_obj.message);
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
    RUN_TEST(test_bond_bond);
    RUN_TEST(test_bond_ok_stop);
    RUN_TEST(test_2_clients);
    RUN_TEST(test_2_clients_stop_unbond);
    RUN_TEST(test_bond_ok);

    return UNITY_END();
}