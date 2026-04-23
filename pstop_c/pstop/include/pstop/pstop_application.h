#ifndef PSTOP_PSTOP_APPLICATION_H
#define PSTOP_PSTOP_APPLICATION_H

#include <stdint.h>

#include "pstop/error.h"
#include "pstop/device_id.h"

typedef enum {
    PSTOP_STATUS_OK = 0,
    PSTOP_STATUS_STOP = 1
} pstop_status_message_t;

typedef uint64_t (* get_current_time_t)(void);
typedef int      (* is_operator_allowed_t)(const device_id_t *device_id);
typedef int      (* pstop_status_t)(pstop_status_message_t status);

typedef void     (* log_message_t)(pstop_error_t error, const char *message);

typedef struct pstop_application_t {

    /**
     * Callback to return the current time.
     */
    get_current_time_t get_time_cb;

    /**
     * The device ID for this machine
     */
    device_id_t machine_device_id;

    /**
     * Simple access control to determine if an operator
     * is allowed to connect to this machine.
     *
     * @Return 1 if operator is allowed, 0 otherwise
     */
    is_operator_allowed_t operator_allowed_cb;

    /**
     * Notifies the underlying hardware about the status of this PSTOP.
     */
    pstop_status_t status_cb;

    /**
     * Callback to handle any errors in the system.
     */
    log_message_t log_message_cb;

    /**
     * Default timeout for all new clients.
     */
    uint64_t default_timeout_ms;

    uint16_t max_lost_messages;

    uint16_t max_missed_heartbeats;

} pstop_application_t;

void pstop_application_init(pstop_application_t *app);

#endif /* PSTOP_PSTOP_APPLICATION_H */
