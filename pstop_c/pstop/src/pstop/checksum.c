
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <stddef.h>

#include "pstop/checksum.h"

static const uint16_t POLY_CRC16 = 0x8D95U;
static const uint32_t POLY_CRC32 = 0x9D7F97D6U;

uint16_t
checksum_crc16(const uint8_t *data, size_t data_length)
{
    uint16_t crc = 0xFFFF;

    for(uint16_t i = 0U; i < data_length; ++i) {
        crc ^= (uint16_t)data[i] << 8U;
        for(uint16_t j = 0U; j < 8U; ++j) {
            if(crc & 0x8000U) {
                crc = (crc << 1U) ^ POLY_CRC16;
            } else {
                crc <<= 1U;
            }
        }
    }
    return crc;
}

uint32_t
checksum_crc32(const uint8_t *data, size_t data_length)
{
   uint32_t crc = 0xFFFFFFFFU;

   for(size_t i = 0U; i < data_length; ++ i) {
      crc = crc ^ (uint32_t)data[i];

      for(int j = 7; j >= 0; j--) {    // Do eight times.
         uint32_t mask = -(crc & 1);
         crc = (crc >> 1) ^ (POLY_CRC32 & mask);
      }
   }

   return ~crc;
}
