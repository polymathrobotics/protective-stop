
#include <stdlib.h>

#include "pstop/pstop_application.h"
#include "pstop/time.h"

static
void
no_log(pstop_error_t /* error */, const char * /* message */)
{

}

void
pstop_application_config_init(pstop_application_config_t *config)
{
    config->default_timeout_ms = 100U;
    config->max_lost_messages = 1U;
    config->max_missed_heartbeats = 1U;
}

void
pstop_application_init(pstop_application_t *app)
{
    app->get_time_cb = time_get_now;
    app->operator_allowed_cb = NULL;
    app->status_cb = NULL;
    app->log_message_cb = no_log;
    pstop_application_config_init(&app->app_config);
}
