// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_PSTOP_APPLICATION_H
#define PSTOP_PSTOP_APPLICATION_H

#include <stdint.h>
#include <stdbool.h>

#include "pstop/error.h"
#include "pstop/device_id.h"
#include "pstop/os.h"

typedef enum {
    PSTOP_STATUS_OK = 0,
    PSTOP_STATUS_STOP = 1
} pstop_status_message_t;

typedef struct {

    /**
     * Is this device_id allowed?
     */
    bool allowed;

    /**
     * The heartbeat time for this device_id in milliseconds.
     */
    uint64_t heartbeat_ms;

    /**
     * Is this device_id a stop-only (can't transition to OK) operator?
     */
    bool stop_only;

} operator_details_t;

/**
 * Utility function to default initialize an operator.
 */
void operator_detail_init(operator_details_t *oper);

/**
 * @brief A callback that will return operator details for the given device_id.
 *
 * An application must define this callback in the pstop_application_t struct
 * and fill in the details for the specified device_id.
 *
 * @param device_id A client device_id
 *
 * @return an operator details struct with details for the specified device_id.
 */
typedef operator_details_t (* get_operator_details_t)(const device_id_t *device_id);

/**
 * @brief A callback that is called by the pstop library to indicate the status of the machine.
 *
 * @param status The current status of the machine. Either PSTOP_STATUS_OK or PSTOP_STATUS_STOP.
 */
typedef void (* pstop_status_t)(pstop_status_message_t status);

/**
 * @brief An optional callback to log received pstop messages.
 *
 * @param timestamp The timestamp this message was received.
 * @param client The device_id that sent the message.
 * @param message The pstop message (OK, STOP, BOND, UNBOND).
 * @param error Any errors associated with this message
 */
typedef void (* log_message_t)(uint64_t timestamp, const device_id_t *client, uint8_t message, pstop_error_t error);

/**
 * Some configuration details
 */
typedef struct {

    /**
     * The maximum number of messages that can be lost before the pstop
     * library reports a client as lost and stops the machine.
     */
    uint16_t max_lost_messages;

    /**
     * The maximum number of heartbeat messages that can be lost before
     * the library reports a client as lost and stops the machine.
     */
    uint16_t max_missed_heartbeats;

} pstop_application_config_t;

typedef struct pstop_application_t {

    /**
     * An OS environment struct that contains a callback for getting
     * the current time.
     */
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
