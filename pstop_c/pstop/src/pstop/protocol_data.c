
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/protocol_data.h"

void
protocol_data_init(protocol_data_t *remote)
{
    device_id_init(&(remote->remote_id));
    remote->last_timestamp = 0U;
    remote->heartbeat_ms = 0U;
    remote->msg_counter = 0U;
    remote->last_sent_counter = 0U;
}
