// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_MACHINE_H
#define PSTOP_MACHINE_H

#include "pstop/pstop_msg.h"
#include "pstop/error.h"
#include "pstop/pstop_application.h"
#include "pstop/pstop_client.h"

typedef struct pstop_machine_t pstop_machine_t;

/**
 * A function callback to handle protocol (black channel) messages.
 */
typedef pstop_error_t (* protocol_handle_message_t)(pstop_machine_t *machine, const pstop_msg_t *req, pstop_msg_t **resp);

/**
 * A function callback to handle new messages that have arrived.
 *
 * @Param machine The machine object
 * @Param req     The request message
 * @Param resp    A pointer to a response message. If NULL then no response is sent back.
 */
typedef pstop_error_t (* machine_handle_message_t)(pstop_machine_t *machine, const pstop_msg_t *req, pstop_msg_t **resp);

/**
 * A function callback that is called to check the heartbeats of the connected PSTOPs
 *
 * @Param machine The machine object
 */
typedef pstop_error_t (* pstop_check_heartbeats_t)(pstop_machine_t *machine);

#define ROBOT_STATE_OK      0
#define ROBOT_STATE_STOPPED 1

#define ROBOT_RESTART_STATE_OK 0
#define ROBOT_RESTART_STATE_NEED_STOP 1
#define ROBOT_RESTART_STATE_STOP_RECEIVED 2

typedef struct {

    /* Is the robot stopped or OK? */
    int robot_state;

    /**
     * The client ID that caused the robot to stop if stopped.
     * 0 if robot is not stopped or if applicaton just started
     * Or the client ID that has started the stop/ok cycle to get robot moving.
     */
    uint32_t client_stop_id;

    /**
     * 0 = everything is OK
     * 1 = need stop message
     * 2 = stop received
     */
    int restart_state;
} robot_state_t;

typedef struct pstop_machine_t {

    protocol_handle_message_t handle_protocol_message_cb;

    machine_handle_message_t handle_machine_message_cb;

    pstop_check_heartbeats_t check_heartbeats_cb;

    pstop_clients_t pstops;

    pstop_application_t *application;

    robot_state_t robot_state;

} pstop_machine_t;

void machine_init(pstop_machine_t *machine, pstop_application_t *app, pstop_client_data_t *clients, uint16_t max_clients);

void machine_stop_robot(pstop_machine_t *machine);

#endif /* PSTOP_MACHINE_H */
