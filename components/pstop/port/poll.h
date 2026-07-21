/* SPDX-License-Identifier: Apache-2.0
 *
 * Shim header so upstream's transport/include/transport/udp/udp_transport.h
 * can `#include <poll.h>` cleanly on ESP-IDF. ESP-IDF / newlib only ships
 * <sys/poll.h>; POSIX exposes both names. This shim is the smallest fix
 * that keeps the upstream submodule untouched.
 *
 * We also pull in <sys/socket.h> here. Upstream's udp_transport.h follows
 * its <poll.h> include with <netinet/in.h> and expects `struct sockaddr_in`
 * to be visible — on glibc that struct is brought in by netinet/in.h, but
 * on lwIP it lives in <sys/socket.h> / <lwip/sockets.h>. Pre-loading
 * <sys/socket.h> here is the smallest fix that keeps upstream untouched. */
#ifndef PSTOP_PORT_POLL_H
#define PSTOP_PORT_POLL_H
#include <sys/poll.h>
#include <sys/socket.h>
#endif
