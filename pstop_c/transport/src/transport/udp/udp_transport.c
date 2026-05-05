
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>

#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "transport/udp/udp_transport.h"

void
transport_udp_init(udp_transport_data_t *transport)
{
    memset(&transport->addr, 0, sizeof(transport->addr));
    transport->fd = -1;
    transport->poll_fds.fd = -1;
    transport->poll_fds.events = 0;
}

int
transport_udp_listen(udp_transport_data_t *transport, const char *host, int port)
{
    memset(&transport->addr, 0, sizeof(transport->addr));
    transport->addr.sin_addr.s_addr = inet_addr(host);
    transport->addr.sin_port = htons(port);
    transport->addr.sin_family = AF_INET;

    // create datagram socket
    transport->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(transport->fd == -1) {
        fprintf(stderr, "socket errno=%d\n", errno);
        return -1;
    }

    int result = bind(transport->fd, (struct sockaddr*)&transport->addr, sizeof(transport->addr));

    if(result < 0) {
        fprintf(stderr, "errno=%d\n", errno);
        return -1;
    }
    transport->poll_fds.fd = transport->fd;
    transport->poll_fds.events = POLLIN;

    return transport->fd;
}

int
transport_udp_connect(udp_transport_data_t *transport, const char *host, int port)
{
    memset(&transport->addr, 0, sizeof(transport->addr));
    transport->addr.sin_addr.s_addr = inet_addr(host);
    transport->addr.sin_port = htons(port);
    transport->addr.sin_family = AF_INET;

    // create datagram socket
    transport->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(transport->fd == -1) {
        fprintf(stderr, "socket errno=%d\n", errno);
        return -1;
    }

    transport->poll_fds.fd = transport->fd;
    transport->poll_fds.events = POLLIN;

    return transport->fd;
}

int
transport_udp_close(udp_transport_data_t *transport)
{
    if(transport->fd != -1) {
        close(transport->fd);
    }

    transport->fd = -1;
    transport->poll_fds.fd = -1;

    return 0;
}

int
transport_udp_read(udp_transport_data_t *transport, uint8_t *dest, size_t length, struct sockaddr_storage *clientAddr)
{
    int numEvents = poll(&(transport->poll_fds), 1, 10);

    if(numEvents == 0) {
        return 0;
    }

    if(transport->poll_fds.revents & POLLIN) {
        socklen_t addrSize = sizeof(struct sockaddr_storage);
        int bytesRead = recvfrom(transport->fd, dest, length, 0, (struct sockaddr *)clientAddr, &addrSize);

        return bytesRead;
    }

    return 0;
}

int
transport_udp_write(udp_transport_data_t *transport, const uint8_t *data, size_t length, struct sockaddr_in *target)
{
    struct sockaddr *addr = (struct sockaddr *)target;
    int addrSize = sizeof(struct sockaddr_in);

    if(target == NULL) {
        addr = (struct sockaddr *)&transport->addr;
    }
    ssize_t numbytes = sendto(transport->fd, data, length, 0, addr, addrSize);

    if(numbytes == -1) {
        fprintf(stderr, "errno=%d\n", errno);
        return -1;
    }

    return numbytes;
}
