// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_TRANSPORT_UDP_UDPTRANSPORT_H
#define PSTOP_TRANSPORT_UDP_UDPTRANSPORT_H

#include <stdint.h>
#include <stddef.h>

#include <poll.h>
#include <netinet/in.h>

typedef struct pollfd udp_transport_poll_t;

typedef struct {

    struct sockaddr_in addr;
    int fd;

    udp_transport_poll_t poll_fds;

} udp_transport_data_t;

void transport_udp_init(udp_transport_data_t *transport);

int transport_udp_listen(udp_transport_data_t *transport, const char *host, int port);

int transport_udp_connect(udp_transport_data_t *transport, const char *host, int port);

int transport_udp_close(udp_transport_data_t *transport);

int transport_udp_read(udp_transport_data_t *transport, uint8_t *dest, size_t length, struct sockaddr_storage *clientAddr);

int transport_udp_write(udp_transport_data_t *transport, const uint8_t *data, size_t length, struct sockaddr_in *target);

#endif /* PSTOP_TRANSPORT_UDP_UDPTRANSPORT_H */
