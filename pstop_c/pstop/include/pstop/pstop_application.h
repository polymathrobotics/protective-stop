// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_PSTOP_APPLICATION_H
#define PSTOP_PSTOP_APPLICATION_H

#include <stdint.h>

#include "pstop/error.h"
#include "pstop/device_id.h"
#include "pstop/os.h"

typedef enum {
    PSTOP_STATUS_OK = 0,
    PSTOP_STATUS_STOP = 1
} pstop_status_message_t;

typedef struct {

    int allowed;

    uint64_t heartbeat_ms;

    int stop_only;

} operator_details_t;

void operator_detail_init(operator_details_t *oper);

typedef operator_details_t (* get_operator_details_t)(const device_id_t *device_id);

typedef int      (* pstop_status_t)(pstop_status_message_t status);

typedef void     (* log_message_t)(pstop_error_t error, const char *message);

typedef struct {
    /**
     * Default timeout for all new clients.
     */
    uint64_t default_timeout_ms;

    uint16_t max_lost_messages;

    uint16_t max_missed_heartbeats;

} pstop_application_config_t;

typedef struct pstop_application_t {

    pstop_os_env_t env;

    /**
     * The device ID for this machine
     */
    device_id_t machine_device_id;

    /**
     * Callback to return details for a specified operator.
     * Details determine if the operator is allowed and
     * certain features of the operator.
     */
    get_operator_details_t operator_details_cb;

    /**
     * Notifies the underlying hardware about the status of this PSTOP.
     */
    pstop_status_t status_cb;

    /**
     * Callback to log any errors in the system. Can be NULL.
     */
    log_message_t log_message_cb;

    pstop_application_config_t app_config;

} pstop_application_t;

void pstop_application_init(pstop_application_t *app);

#endif /* PSTOP_PSTOP_APPLICATION_H */
