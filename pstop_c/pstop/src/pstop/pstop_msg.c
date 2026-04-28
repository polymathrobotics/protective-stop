
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/pstop_msg.h"
#include "pstop/checksum.h"
#include "pstop/constants.h"

void
pstop_message_init(pstop_msg_t *msg)
{
    msg->stamp = 0U;
    msg->received_stamp = 0U;
    device_id_init(&(msg->id));
    device_id_init(&(msg->receiver_id));
    msg->heartbeat_timeout = 0U;
    msg->counter = 0U;
    msg->received_counter = 0U;
    msg->message = PSTOP_MESSAGE_UNKNOWN;
    msg->checksum = 0U;
}

// this check belongs in the black channel protocol code
uint16_t
pstop_calculate_checksum(const pstop_msg_t *msg)
{
    // calculate checksum
    const uint8_t *start = (const uint8_t *)msg;
    const uint8_t *end = start + (((const uint8_t *)&(msg->checksum)) - start);

    return checksum_crc16(start, (end - start));
}

static
int
is_message_type_valid(uint8_t message)
{
    return (message >= PSTOP_MESSAGE_OK) && (message <= PSTOP_MESSAGE_UNBOND);
}

pstop_error_t
pstop_is_message_valid(const pstop_msg_t *msg)
{
    if(!is_message_type_valid(msg->message)) {
        return PSTOP_MESSAGE_TYPE_INVALID;
    }

    return PSTOP_OK;
}
