// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_ENDIAN_H
#define PSTOP_ENDIAN_H

#include <stdint.h>
#include <stddef.h>

uint16_t read_uint16_le(const uint8_t *data, size_t *pos);
uint16_t read_uint16_be(const uint8_t *data, size_t *pos);

uint32_t read_uint32_le(const uint8_t *data, size_t *pos);
uint32_t read_uint32_be(const uint8_t *data, size_t *pos);

uint64_t read_uint64_le(const uint8_t *data, size_t *pos);
uint64_t read_uint64_be(const uint8_t *data, size_t *pos);

void write_uint16_le(uint16_t value, uint8_t *data, size_t *pos);
void write_uint16_be(uint16_t value, uint8_t *data, size_t *pos);

void write_uint32_le(uint32_t value, uint8_t *data, size_t *pos);
void write_uint32_be(uint32_t value, uint8_t *data, size_t *pos);

void write_uint64_le(uint64_t value, uint8_t *data, size_t *pos);
void write_uint64_be(uint64_t value, uint8_t *data, size_t *pos);

#endif /* PSTOP_ENDIAN_H */
