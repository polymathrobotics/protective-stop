// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/endian.h"

uint16_t
read_uint16_le(const uint8_t *data, size_t *pos)
{
    const uint16_t *bytes = (const uint16_t *)(data + *pos);
    *pos = *pos + 2U;
    return *bytes;
}

uint16_t
read_uint16_be(const uint8_t *data, size_t *pos)
{
    const uint8_t *bytes = data + *pos;
    *pos = *pos + 2U;
    uint16_t b0 = (uint16_t)bytes[0];
    uint16_t b1 = (uint16_t)bytes[1];
    uint16_t val = (b0 << 8) | b1;
    return val;
}

uint32_t
read_uint32_le(const uint8_t *data, size_t *pos)
{
    const uint32_t *bytes = (const uint32_t *)(data + *pos);
    *pos = *pos + 4U;
    return *bytes;
}

uint32_t
read_uint32_be(const uint8_t *data, size_t *pos)
{
    const uint8_t *bytes = data + *pos;
    *pos = *pos + 4U;
    uint32_t b0 = (uint32_t)bytes[0];
    uint32_t b1 = (uint32_t)bytes[1];
    uint32_t b2 = (uint32_t)bytes[2];
    uint32_t b3 = (uint32_t)bytes[3];
    uint32_t val = (b0 << 24) | (b1 << 16U) | (b2 << 8U) | b3;
    return val;
}

uint64_t
read_uint64_le(const uint8_t *data, size_t *pos)
{
    const uint64_t *bytes = (const uint64_t *)(data + *pos);
    *pos = *pos + 8U;
    return *bytes;
}

uint64_t
read_uint64_be(const uint8_t *data, size_t *pos)
{
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
}

void
write_uint16_le(uint16_t value, uint8_t *data, size_t *pos)
{
    uint16_t *bytes = (uint16_t *)(data + *pos);
    *pos = *pos + 2U;
    *bytes = value;
}

void
write_uint16_be(uint16_t value, uint8_t *data, size_t *pos)
{
    uint8_t *bytes = (data + *pos);
    *pos = *pos + 2U;
    bytes[1] = (uint8_t)(value & 0xFFU);
    bytes[0] = (uint8_t)((value >> 8U)& 0xFFU);
}

void
write_uint32_le(uint32_t value, uint8_t *data, size_t *pos)
{
    uint32_t *bytes = (uint32_t *)(data + *pos);
    *pos = *pos + 4U;
    *bytes = value;
}

void
write_uint32_be(uint32_t value, uint8_t *data, size_t *pos)
{
    uint8_t *bytes = data + *pos;
    *pos = *pos + 4U;
    bytes[3] = (uint8_t)(value & 0xFFU);
    bytes[2] = (uint8_t)((value >> 8U) & 0xFFU);
    bytes[1] = (uint8_t)((value >> 16U) & 0xFFU);
    bytes[0] = (uint8_t)((value >> 24U) & 0xFFU);
}

void
write_uint64_le(uint64_t value, uint8_t *data, size_t *pos)
{
    uint64_t *bytes = (uint64_t *)(data + *pos);
    *pos = *pos + 8U;
    *bytes = value;
}

void
write_uint64_be(uint64_t value, uint8_t *data, size_t *pos)
{
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
}
