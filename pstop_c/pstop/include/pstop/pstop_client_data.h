// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_PSTOP_CLIENT_DATA_H
#define PSTOP_PSTOP_CLIENT_DATA_H

#include <stdint.h>
#include <stdbool.h>

#include "pstop/device_id.h"
#include "pstop/constants.h"
#include "pstop/protocol_data.h"

typedef enum {

    PSTOP_CLIENT_INITING = 0,
    PSTOP_CLIENT_BONDED = 1,
    PSTOP_CLIENT_UNBONDED = 2,
    PSTOP_CLIENT_STOPPED = 3,
    PSTOP_CLIENT_FAILURE = 4,
    PSTOP_CLIENT_UNKNOWN = 255

} pstop_client_state_t;

typedef struct {

    uint32_t local_client_id;

    protocol_data_t client_data;

    uint8_t last_message;

    // is this client a stop-only operator?
    bool is_stop_only;

    // how many messages have we lost so far?
    uint16_t lost_message_counter;

    // how many heartbeats have we missed?
    uint16_t missed_heartbeats_counter;

    pstop_client_state_t client_state;

} pstop_client_data_t;

typedef struct {

    pstop_client_data_t *clients;
    uint16_t max_clients;

} pstop_clients_t;

/**
 * @brief Initialize the pstop_client_data_t structure to default values.
 *
 * @param client The client data to initialize.
 */
void pstop_client_init(pstop_client_data_t *client);

/**
 * @brief Initializes a pstop_clients object to default values.
 *
 * @param clients The collection of clients to initialize
 */
void pstop_clients_init(pstop_clients_t *clients);

/**
 * @brief Returns the number of active clients.
 *
 * @param clients The collection of clients.
 * @return The number of currently active clients.
 */
uint16_t pstop_client_num_active(const pstop_clients_t *clients);

/**
 * @brief Returns an empty pstop_client if one is available
 *
 * @return the client data if successful, NULL if no space available.
 */
pstop_client_data_t *pstop_client_get_free_client(pstop_clients_t *clients);

/**
 * @brief Deactivates this client.
 *
 * @param clients The collection of clients.
 */
void pstop_client_deactivate(pstop_client_data_t *client);

/**
 * @brief Finds the specified pstop client by device ID.
 *
 * @return the requsted client if found or NULL if not found.
 */
pstop_client_data_t *pstop_client_get(const pstop_clients_t *clients, const device_id_t *client_id);

#endif /* PSTOP_PSTOP_CLIENT_DATA_H */
