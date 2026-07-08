
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <stdlib.h>

#include "pstop/pstop_application.h"

static
void
no_log(uint64_t /* timestamp */, const device_id_t * /* client */, uint8_t /* message */, pstop_error_t /* error */)
{

}

void
remote_detail_init(remote_details_t *oper)
{
    remote_detail_set(oper, true, 1000U, true);
}

void
remote_detail_set(remote_details_t *oper, bool is_allowed, uint64_t heartbeat_ms, bool is_stop_only)
{
    oper->allowed = is_allowed;
    oper->heartbeat_ms = heartbeat_ms;
    oper->stop_only = is_stop_only;
}

static
void
pstop_application_config_init(pstop_application_config_t *config, uint16_t max_lost_messages, uint16_t max_missed_heartbeats)
{
    config->max_lost_messages = max_lost_messages;
    config->max_missed_heartbeats = max_missed_heartbeats;
}

void
pstop_application_init(pstop_application_t *app)
{
    pstop_os_env_init(&app->env);
    pstop_application_set_remote_cb(app, NULL);
    pstop_application_set_hardware_status_cb(app, NULL);
    pstop_application_set_log_cb(app, NULL);
    pstop_application_set_protocol_limits(app, 0U, 0U);
}

void
pstop_application_set_time_cb(pstop_application_t *app, get_current_time_t time_cb)
{
    app->env.get_time_cb = time_cb;
}

void
pstop_application_set_machine_id(pstop_application_t *app, const device_id_t *machine_id)
{
    device_id_copy(&(app->machine_device_id), machine_id);
}

void
pstop_application_set_remote_cb(pstop_application_t *app, get_remote_details_t remote_cb)
{
    app->remote_details_cb = remote_cb;
}

void
pstop_application_set_hardware_status_cb(pstop_application_t *app, pstop_status_t status_cb)
{
    app->status_cb = status_cb;
}

void
pstop_application_set_log_cb(pstop_application_t *app, log_message_t log_cb)
{
    if(log_cb == NULL) {
        app->log_message_cb = no_log;
    }
    else {
        app->log_message_cb = log_cb;
    }
}

void
pstop_application_set_protocol_limits(pstop_application_t *app, uint16_t max_lost_messages, uint16_t max_missed_heartbeats)
{
    pstop_application_config_init(&(app->app_config), max_lost_messages, max_missed_heartbeats);
}
