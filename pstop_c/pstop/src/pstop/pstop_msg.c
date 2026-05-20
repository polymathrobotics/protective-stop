
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <string.h>

#include "pstop/pstop_msg.h"
#include "pstop/checksum.h"
#include "pstop/constants.h"
#include "pstop/endian.h"

void
pstop_message_init(pstop_msg_t *msg)
{
    msg->version = PSTOP_VERSION;
    msg->stamp = 0U;
    msg->received_stamp = 0U;
    device_id_init(&(msg->id));
    device_id_init(&(msg->receiver_id));
    msg->heartbeat_timeout = 0U;
    msg->counter = 0U;
    msg->received_counter = 0U;
    msg->message = PSTOP_MESSAGE_UNKNOWN;
    msg->checksum = 0U;
    msg->calculated_checksum = 0U;
}

static
int
is_message_type_valid(uint8_t message)
{
    return (message >= PSTOP_MESSAGE_OK) && (message <= PSTOP_MESSAGE_UNBOND);
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
    return read_uint16_le(data, pos);
#else
    return read_uint16_be(data, pos);
#endif
}

static
void
write_uint16(uint16_t value, uint8_t *data, size_t *pos)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    write_uint16_le(value, data, pos);
#else
    write_uint16_be(value, data, pos);
#endif
}

static
uint32_t
read_uint32(const uint8_t *data, size_t *pos)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return read_uint32_le(data, pos);
#else
    return read_uint32_be(data, pos);
#endif
}

static
void
write_uint32(uint32_t value, uint8_t *data, size_t *pos)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    write_uint32_le(value, data, pos);
#else
    write_uint32_be(value, data, pos);
#endif
}

static
uint64_t
read_uint64(const uint8_t *data, size_t *pos)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return read_uint64_le(data, pos);
#else
    return read_uint64_be(data, pos);
#endif
}

static
void
write_uint64(uint64_t value, uint8_t *data, size_t *pos)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    write_uint64_le(value, data, pos);
#else
    write_uint64_be(value, data, pos);
#endif
}

static
void
read_device_id(device_id_t *device_id, const uint8_t *data, size_t *pos)
{
#if PSTOP_VERSION == 0x00
    device_id->data = read_uint32(data, pos);
#else
#   error "Unsupported PSTOP_VERSION"
#endif
}

static
void
write_device_id(const device_id_t *device_id, uint8_t *data, size_t *pos)
{
#if PSTOP_VERSION == 0x00
    write_uint32(device_id->data, data, pos);
#else
#   error "Unsupported PSTOP_VERSION"
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
    msg->version = read_uint8(data, &pos);
    msg->message = read_uint8(data, &pos);
    msg->stamp = read_uint64(data, &pos);
    msg->received_stamp = read_uint64(data, &pos);
    read_device_id(&(msg->id), data, &pos);
    read_device_id(&(msg->receiver_id), data, &pos);
    msg->heartbeat_timeout = read_uint32(data, &pos);
    msg->counter = read_uint32(data, &pos);
    msg->received_counter = read_uint32(data, &pos);
    msg->checksum = read_uint16(data, &pos);
    msg->calculated_checksum = checksum_crc16(data, PSTOP_MESSAGE_SIZE - 2U);
}

void
pstop_message_encode(const pstop_msg_t *msg, uint8_t *data)
{
    size_t pos = 0U;
    write_uint8(msg->version, data, &pos);
    write_uint8(msg->message, data, &pos);
    write_uint64(msg->stamp, data, &pos);
    write_uint64(msg->received_stamp, data, &pos);
    write_device_id(&(msg->id), data, &pos);
    write_device_id(&(msg->receiver_id), data, &pos);
    write_uint32(msg->heartbeat_timeout, data, &pos);
    write_uint32(msg->counter, data, &pos);
    write_uint32(msg->received_counter, data, &pos);
    uint16_t checksum = checksum_crc16(data, PSTOP_MESSAGE_SIZE - 2U);
    write_uint16(checksum, data, &pos);
}
