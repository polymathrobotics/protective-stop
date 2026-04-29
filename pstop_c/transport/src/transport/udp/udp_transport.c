

#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "transport/udp/udp_transport.h"

int
transport_udp_open(udp_transport_data_t *transport, const char *host, int port)
{
    memset(&transport->addr, 0, sizeof(transport->addr));
    transport->addr.sin_addr.s_addr = inet_addr(host);
    transport->addr.sin_port = htons(port);
    transport->addr.sin_family = AF_INET;

    // create datagram socket
    transport->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(transport->fd == -1) {
        return -1;
    }

    int result = bind(transport->fd, (struct sockaddr*)&transport->addr, sizeof(transport->addr));

    transport->poll_fds.fd = transport->fd;
    transport->poll_fds.events = POLLIN;

    return result;
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

    //fprintf(stderr, "Wrote bytes: %d\n", (int)numbytes);
    if(numbytes == -1) {
        // fprintf(stderr, "errono=%d\n", errno);
        return -1;
    }

    return numbytes;
}

void
transport_init_udp(udp_transport_data_t *transport)
{
    transport->fd = -1;
    memset(&transport->addr, 0, sizeof(transport->addr));
}
