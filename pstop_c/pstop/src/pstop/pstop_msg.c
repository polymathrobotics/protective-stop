
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <string.h>

#include "pstop/pstop_msg.h"
#include "pstop/checksum.h"
#include "pstop/constants.h"

void
pstop_message_init(pstop_msg_t *msg)
{
    msg->version = 0x0U;
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

static
void
read_device_uuid(device_id_t *id, const uint8_t *data, size_t *pos)
{
    memcpy(&(id->data), data + *pos, DEVICE_ID_LENGTH);
    *pos = *pos + 16U;
}

static
void
write_device_uuid(const device_id_t *id, uint8_t *data, size_t *pos)
{
    memcpy(data + *pos, &(id->data), DEVICE_ID_LENGTH);
    *pos = *pos + 16U;
}

static
uint8_t
read_uint8(const uint8_t *data, size_t *pos)
{
    uint8_t b = data[*pos];
    *pos = *pos + 1U;
    return b;
}

static
void
write_uint8(uint8_t value, uint8_t *data, size_t *pos)
{
    data[*pos] = value;
    *pos = *pos + 1U;
}

static
uint16_t
read_uint16(const uint8_t *data, size_t *pos)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    const uint16_t *bytes = (const uint16_t *)(data + *pos);
    *pos = *pos + 2U;
    return *bytes;
#else
    const uint8_t *bytes = data + *pos;
    *pos = *pos + 2U;
    uint16_t b0 = (uint16_t)bytes[0];
    uint16_t b1 = (uint16_t)bytes[1];
    uint16_t val = (b0 << 8) | b1;
    return val;
#endif
}

static
void
write_uint16(uint16_t value, uint8_t *data, size_t *pos)
{
    uint8_t *bytes = data + *pos;
    *pos = *pos + 2U;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    bytes[0] = (uint8_t)(value & 0xFFU);
    bytes[1] = (uint8_t)((value >> 8U)& 0xFFU);
#else
    bytes[1] = (uint8_t)(value & 0xFFU);
    bytes[0] = (uint8_t)((value >> 8U)& 0xFFU);
#endif
}

static
uint32_t
read_uint32(const uint8_t *data, size_t *pos)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    const uint32_t *bytes = (const uint32_t *)(data + *pos);
    *pos = *pos + 4U;
    return *bytes;
#else
    const uint8_t *bytes = data + *pos;
    *pos = *pos + 4U;
    uint32_t b0 = (uint32_t)bytes[0];
    uint32_t b1 = (uint32_t)bytes[1];
    uint32_t b2 = (uint32_t)bytes[2];
    uint32_t b3 = (uint32_t)bytes[3];
    uint32_t val = (b0 << 24) | (b1 << 16U) | (b2 << 8U) | b3;
    return val;
#endif
}

static
void
write_uint32(uint32_t value, uint8_t *data, size_t *pos)
{
    uint8_t *bytes = data + *pos;
    *pos = *pos + 4U;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    bytes[0] = (uint8_t)(value & 0xFFU);
    bytes[1] = (uint8_t)((value >> 8U) & 0xFFU);
    bytes[2] = (uint8_t)((value >> 16U) & 0xFFU);
    bytes[3] = (uint8_t)((value >> 24U) & 0xFFU);
#else
    bytes[3] = (uint8_t)(value & 0xFFU);
    bytes[2] = (uint8_t)((value >> 8U) & 0xFFU);
    bytes[1] = (uint8_t)((value >> 16U) & 0xFFU);
    bytes[0] = (uint8_t)((value >> 24U) & 0xFFU);
#endif
}

static
uint64_t
read_uint64(const uint8_t *data, size_t *pos)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    const uint64_t *bytes = (const uint64_t *)(data + *pos);
    *pos = *pos + 8U;
    return *bytes;
#else
    const uint8_t *bytes = data + *pos;
    *pos = *pos + 8U;
    uint64_t b0 = (uint64_t)bytes[0];
    uint64_t b1 = (uint64_t)bytes[1];
    uint64_t b2 = (uint64_t)bytes[2];
    uint64_t b3 = (uint64_t)bytes[3];
    uint64_t b4 = (uint64_t)bytes[4];
    uint64_t b5 = (uint64_t)bytes[5];
    uint64_t b6 = (uint64_t)bytes[6];
    uint64_t b7 = (uint64_t)bytes[7];
    uint64_t val = (b0 << 56U) | (b1 << 48U) | (b2 << 40U) | (b3 << 32U) | (b4 << 24U) | (b5 << 16U) | (b6 << 8U) | b7;
    return val;
#endif
}

static
void
write_uint64(uint64_t value, uint8_t *data, size_t *pos)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    uint64_t *bytes = (uint64_t *)(data + *pos);
    *pos = *pos + 8U;
    *bytes = value;
#else
    uint8_t *bytes = data + *pos;
    *pos = *pos + 8U;
    bytes[7] = (uint8_t)(value & 0xFFU);
    bytes[6] = (uint8_t)((value >> 8U) & 0xFFU);
    bytes[5] = (uint8_t)((value >> 16U) & 0xFFU);
    bytes[4] = (uint8_t)((value >> 24U) & 0xFFU);
    bytes[3] = (uint8_t)((value >> 32U) & 0xFFU);
    bytes[2] = (uint8_t)((value >> 40U) & 0xFFU);
    bytes[1] = (uint8_t)((value >> 48U) & 0xFFU);
    bytes[0] = (uint8_t)((value >> 56U) & 0xFFU);
#endif
}

pstop_error_t
pstop_is_message_valid(const pstop_msg_t *msg)
{
    if(!is_message_type_valid(msg->message)) {
        return PSTOP_MESSAGE_TYPE_INVALID;
    }

    return PSTOP_OK;
}

void
pstop_message_decode(pstop_msg_t *msg, const uint8_t *data)
{
    size_t pos = 0U;
    uint8_t b = read_uint8(data, &pos);
    msg->version = (b & 0xF0) >> 4U;
    msg->message = (b & 0x0F);
    msg->stamp = read_uint64(data, &pos);
    msg->received_stamp = read_uint64(data, &pos);
    read_device_uuid(&msg->id, data, &pos);
    read_device_uuid(&msg->receiver_id, data, &pos);
    msg->heartbeat_timeout = read_uint32(data, &pos);
    msg->counter = read_uint32(data, &pos);
    msg->received_counter = read_uint32(data, &pos);
    msg->checksum = read_uint16(data, &pos);
}

void
pstop_message_encode(const pstop_msg_t *msg, uint8_t *data)
{
    size_t pos = 0U;
    uint8_t b = (msg->version << 4) | msg->message;
    write_uint8(b, data, &pos);
    write_uint64(msg->stamp, data, &pos);
    write_uint64(msg->received_stamp, data, &pos);
    write_device_uuid(&msg->id, data, &pos);
    write_device_uuid(&msg->receiver_id, data, &pos);
    write_uint32(msg->heartbeat_timeout, data, &pos);
    write_uint32(msg->counter, data, &pos);
    write_uint32(msg->received_counter, data, &pos);
    write_uint16(msg->checksum, data, &pos);
}
