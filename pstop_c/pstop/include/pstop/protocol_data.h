// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_PROTOCOL_DATA_H
#define PSTOP_PROTOCOL_DATA_H

#include <stdint.h>

#include "pstop/device_id.h"

/// Per-remote protocol state.
typedef struct {

    /// Device ID of the remote.
    device_id_t remote_id;

    /// Time the last message was sent to the remote.
    uint64_t last_timestamp;

    /// Heartbeat interval for the remote, in milliseconds.
    uint64_t heartbeat_ms;

    /// Counter of the next message sent to the remote.
    uint32_t msg_counter;

    /// Timestamp of the last message received from the remote.
    uint64_t last_received_stamp;

    /// Counter of the last message received from the remote.
    uint32_t last_received_counter;

} protocol_data_t;

void protocol_data_init(protocol_data_t *remote);

#endif /* PSTOP_PROTOCOL_DATA_H */
