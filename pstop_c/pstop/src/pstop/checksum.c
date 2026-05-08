
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <stddef.h>

#include "pstop/checksum.h"

static const uint16_t POLY = 0x1021U;

uint16_t
checksum_crc16(const uint8_t *data, size_t data_length)
{
    uint16_t crc = 0xFFFF;

    for(uint16_t i = 0U; i < data_length; ++i) {
        crc ^= (uint16_t)data[i] << 8U;
        for(uint16_t j = 0U; j < 8U; ++j) {
            if(crc & 0x8000U) {
                crc = (crc << 1U) ^ POLY;
            } else {
                crc <<= 1U;
            }
        }
    }
    return crc;
}
