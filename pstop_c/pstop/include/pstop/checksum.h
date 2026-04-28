#ifndef PSTOP_CHECKSUM_H
#define PSTOP_CHECKSUM_H

#include <stdint.h>
#include <stddef.h>

uint16_t checksum_crc16(const uint8_t *data, size_t data_length);

#endif /* PSTOP_CHECKSUM_H */
