// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_PSTOP_REMOTE_DATA_H
#define PSTOP_PSTOP_REMOTE_DATA_H

#include <stdint.h>
#include <stdbool.h>

#include "pstop/device_id.h"
#include "pstop/constants.h"
#include "pstop/protocol_data.h"

typedef enum {

    PSTOP_REMOTE_INITING = 0,
    PSTOP_REMOTE_BONDED = 1,
    PSTOP_REMOTE_UNBONDED = 2,
    PSTOP_REMOTE_STOPPED = 3,
    PSTOP_REMOTE_FAILURE = 4,
    PSTOP_REMOTE_UNKNOWN = 255

} pstop_remote_state_t;

typedef struct {

    uint32_t local_remote_id;

    protocol_data_t remote_data;

    uint8_t last_message;

    // is this remote a stop-only operator?
    bool is_stop_only;

    // how many messages have we lost so far?
    uint16_t lost_message_counter;

    // how many heartbeats have we missed?
    uint16_t missed_heartbeats_counter;

    pstop_remote_state_t remote_state;

} pstop_remote_data_t;

typedef struct {

    pstop_remote_data_t *remotes;
    uint16_t max_remotes;

} pstop_remotes_t;

/**
 * @brief Initialize the pstop_remote_data_t structure to default values.
 *
 * @param remote The remote data to initialize.
 */
void pstop_remote_init(pstop_remote_data_t *remote);

/**
 * @brief Initializes a pstop_remotes object to default values.
 *
 * @param remotes The collection of remotes to initialize
 */
void pstop_remotes_init(pstop_remotes_t *remotes);

/**
 * @brief Returns the number of active remotes.
 *
 * @param remotes The collection of remotes.
 * @return The number of currently active remotes.
 */
uint16_t pstop_remote_num_active(const pstop_remotes_t *remotes);

/**
 * @brief Returns an empty pstop_remote if one is available
 *
 * @return the remote data if successful, NULL if no space available.
 */
pstop_remote_data_t *pstop_remote_get_free_remote(pstop_remotes_t *remotes);

/**
 * @brief Deactivates this remote.
 *
 * @param remotes The collection of remotes.
 */
void pstop_remote_deactivate(pstop_remote_data_t *remote);

/**
 * @brief Finds the specified pstop remote by device ID.
 *
 * @return the requsted remote if found or NULL if not found.
 */
pstop_remote_data_t *pstop_remote_get(const pstop_remotes_t *remotes, const device_id_t *remote_id);

#endif /* PSTOP_PSTOP_REMOTE_DATA_H */
