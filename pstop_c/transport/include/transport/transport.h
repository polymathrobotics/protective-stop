#ifndef PSTOP_TRANSPORT_TRANSPORT_H
#define PSTOP_TRANSPORT_TRANSPORT_H

#include <stdint.h>
#include <stddef.h>

typedef int (*transport_open_t)(const char *host, int port);
typedef int (*transport_close_t)();

typedef int (*transport_read_t)(uint8_t *dest, size_t length, struct sockaddr_storage *clientAddr);

typedef int (*transport_write_t)(const uint8_t *data, size_t length, struct sockaddr_in *target);

typedef struct {

    transport_open_t open_cb;
    transport_close_t close_cb;

    transport_read_t read_cb;
    transport_write_t write_cb;

} transport_t;

#endif /* PSTOP_TRANSPORT_TRANSPORT_H */
