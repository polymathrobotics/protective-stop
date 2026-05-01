#ifndef PSTOP_PSTOP_MSG_H
#define PSTOP_PSTOP_MSG_H

#include <stdint.h>

#include "pstop/device_id.h"
#include "pstop/error.h"

typedef uint8_t message_type_t;

#define PSTOP_MESSAGE_OK 0U
#define PSTOP_MESSAGE_STOP 1U
#define PSTOP_MESSAGE_BOND 2U
#define PSTOP_MESSAGE_UNBOND 3U
#define PSTOP_MESSAGE_UNKNOWN 0x0FU

#define PSTOP_MESSAGE_SIZE 64U

/**
 * A PSTOP message object. With enough information
 * to provide basic black channel support.
 */
typedef struct {
    /**
     * Version of this message. Currently just 0x0
     */
    uint8_t version;

    /**
     * Type of message.
     */
    message_type_t message;

    /**
     * The timestamp (in milliseconds) of this message.
     */
    uint64_t stamp;

    /**
     * The last timestamp that we received from the client.
     * Or 0 if we are estabilishing a new bonding.
     */
    uint64_t received_stamp;

    /**
     * The UUID of this device.
     */
    device_id_t id;

    /**
     * Unique ID of the device we are sending this message to.
     */
    device_id_t receiver_id;

    /**
     * A timeout in milliseconds that we expect to receive heartbeats from
     * this machine. Only valid when sent from machine to operator.
     */
    uint32_t heartbeat_timeout;

    /**
     * An incrementing counter.
     */
    uint32_t counter;

    /**
     * The last counter we received from the client.
     * Or 0 if we are establishing a new bonding.
     */
    uint32_t received_counter;

    /**
     * a CRC-16 checksum of the above values.
     */
    uint16_t checksum;

} pstop_msg_t;

void pstop_message_init(pstop_msg_t *msg);

uint16_t pstop_calculate_checksum(const pstop_msg_t *msg);

pstop_error_t pstop_is_message_valid(const pstop_msg_t *msg);

/**
 * Network encoding/decoding functions
 */
void pstop_message_decode(pstop_msg_t *msg, const uint8_t *data);
void pstop_message_encode(const pstop_msg_t *msg, uint8_t *data);

#endif /* PSTOP_PSTOP_MSG_H */

