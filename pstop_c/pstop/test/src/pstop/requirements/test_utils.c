
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

static pstop_status_message_t last_status = PSTOP_STATUS_OK;
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
    last_status = PSTOP_STATUS_OK;
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
