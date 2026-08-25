
#include "pstop/requirements/test_utils.h"

static uint64_t current_time;

uint64_t
get_time(void)
{
    return current_time++;
}

void
set_time(uint64_t newtime)
{
    current_time = newtime;
}

static remote_details_t details = {
    .allowed = true,
    .stop_only = true,
    .heartbeat_ms = 500U
};

remote_details_t
is_operator_allowed(const device_id_t * /* id */)
{
    return details;
}

void
set_operator_allowed(bool is_allowed, bool stop_only, uint64_t heartbeat_ms)
{
    details.allowed = is_allowed;
    details.stop_only = stop_only;
    details.heartbeat_ms = heartbeat_ms;
}

static pstop_status_message_t last_status = PSTOP_STATUS_STOP;
static int robot_status_counter = 0;

void
robot_status(pstop_status_message_t status)
{
    last_status = status;

    robot_status_counter++;
}

pstop_status_message_t
get_last_status()
{
    return last_status;
}

int
get_robot_status_counter()
{
    return robot_status_counter;
}

void
reset_robot_status()
{
    last_status = PSTOP_STATUS_STOP;
    robot_status_counter = 0;
}

void
log_error(uint64_t /* timestamp */, const device_id_t * /* client */, uint8_t /* message */, pstop_error_t /* error */)
{

}

void
configure_app_defaults(pstop_application_t *app,
    const device_id_t *machine_id,
    uint16_t max_lost_messages, uint16_t max_missed_heartbeats,uint32_t stop_ok_delay_ms)
{
    pstop_application_init(app);
    pstop_application_set_time_cb(app, get_time);
    pstop_application_set_machine_id(app, machine_id);
    pstop_application_set_remote_cb(app, is_operator_allowed);
    pstop_application_set_hardware_status_cb(app, robot_status);
    pstop_application_set_log_cb(app, log_error);
    pstop_application_set_protocol_limits(app, max_lost_messages, max_missed_heartbeats, stop_ok_delay_ms);
}

pstop_error_t
send_bond(pstop_machine_t *machine, pstop_msg_t *resp,
    const device_id_t *remote, const device_id_t *local, uint32_t counter, uint64_t timestamp)
{
    pstop_msg_t req;
    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_BOND;
    req.counter = counter;
    req.stamp = timestamp;
    device_id_copy(&(req.id), remote);
    device_id_copy(&(req.receiver_id), local);
    req.received_counter = 0U;
    req.received_stamp = 0U;
    req.checksum = 10U;
    req.calculated_checksum = 10U;

    pstop_message_init(resp);

    return machine_process_message(machine, &req, resp);
}

pstop_error_t
send_stop(pstop_machine_t *machine, pstop_msg_t *resp,
    const device_id_t *remote, const device_id_t *local, uint32_t counter, uint64_t timestamp)
{
    pstop_msg_t req;
    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_STOP;
    req.counter = counter;
    req.stamp = timestamp;
    device_id_copy(&(req.id), remote);
    device_id_copy(&(req.receiver_id), local);
    req.received_counter = resp->counter;
    req.received_stamp = resp->stamp;
    req.checksum = 10U;
    req.calculated_checksum = 10U;

    set_time(timestamp);

    pstop_message_init(resp);

    return machine_process_message(machine, &req, resp);
}

pstop_error_t
send_ok(pstop_machine_t *machine, pstop_msg_t *resp,
    const device_id_t *remote, const device_id_t *local, uint32_t counter, uint64_t timestamp)
{
    pstop_msg_t req;
    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_OK;
    req.counter = counter;
    req.stamp = timestamp;
    device_id_copy(&(req.id), remote);
    device_id_copy(&(req.receiver_id), local);
    req.received_counter = resp->counter;
    req.received_stamp = resp->stamp;
    req.checksum = 10U;
    req.calculated_checksum = 10U;

    set_time(timestamp);

    pstop_message_init(resp);

    return machine_process_message(machine, &req, resp);
}
