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
    PSTOP_REMOTE_OK = 2,
    PSTOP_REMOTE_STOPPED = 3,
    PSTOP_REMOTE_UNKNOWN = 255

} pstop_remote_state_t;

/// State tracked for one remote.
typedef struct {

    /// Local identifier of the remote.
    uint32_t local_remote_id;

    /// Protocol state for the remote.
    protocol_data_t remote_data;

    /// Last message type received from the remote.
    uint8_t last_message;

    /// True if the remote can stop but not restart the robot.
    bool is_stop_only;

    /// Messages lost so far.
    uint16_t lost_message_counter;

    /// Heartbeats missed so far.
    uint16_t missed_heartbeats_counter;

    /// Lifecycle state of the remote.
    pstop_remote_state_t remote_state;

} pstop_remote_data_t;

/// A fixed-capacity collection of remotes.
typedef struct {

    /// Caller-owned array of max_remotes entries.
    pstop_remote_data_t *remotes;

    /// Capacity of the remotes array.
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
 * @param remote The remote to deactivate.
 */
void pstop_remote_deactivate(pstop_remote_data_t *remote);

/**
 * @brief Returns the number of remotes in the stopped state.
 *
 * @param remotes The collection of remotes.
 * @return The number of remotes in PSTOP_REMOTE_STOPPED.
 */
uint16_t pstop_remote_num_stopped(const pstop_remotes_t *remotes);

/**
 * @brief Finds the specified pstop remote by device ID.
 *
 * @return the requsted remote if found or NULL if not found.
 */
pstop_remote_data_t *pstop_remote_get(const pstop_remotes_t *remotes, const device_id_t *remote_id);

#endif /* PSTOP_PSTOP_REMOTE_DATA_H */
