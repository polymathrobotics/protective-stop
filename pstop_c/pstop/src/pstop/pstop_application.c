
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

void
pstop_application_config_init(pstop_application_config_t *config)
{
    config->max_lost_messages = 0U;
    config->max_missed_heartbeats = 0U;
}

void
pstop_application_init(pstop_application_t *app)
{
    pstop_os_env_init(&app->env);
    app->remote_details_cb = NULL;
    app->status_cb = NULL;
    app->log_message_cb = no_log;
    pstop_application_config_init(&app->app_config);
}
