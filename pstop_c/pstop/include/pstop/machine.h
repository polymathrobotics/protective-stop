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

typedef struct pstop_machine_t {

    protocol_handle_message_t handle_protocol_message_cb;

    machine_handle_message_t handle_machine_message_cb;

    pstop_check_heartbeats_t check_heartbeats_cb;

    pstop_clients_t pstops;

    pstop_application_t application;

} pstop_machine_t;

void machine_init(pstop_machine_t *machine, const pstop_application_t *app, pstop_client_data_t *clients, uint16_t max_clients);

void machine_stop_robot(pstop_machine_t *machine);

#endif /* PSTOP_MACHINE_H */
