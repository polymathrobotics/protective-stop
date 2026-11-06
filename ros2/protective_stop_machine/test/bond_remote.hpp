// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Test helper: bonds a remote to a software backend over loopback UDP.
#pragma once

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>

extern "C"
{
#include "pstop/pstop_msg.h"
}

namespace pstop_test
{

/// Sends one BOND frame from remote_id to the machine on 127.0.0.1:port and waits for the reply.
/// @return True if a full reply frame arrived.
inline bool bond_remote(int port, uint32_t machine_id, uint32_t remote_id)
{
  const int sock = socket(AF_INET, SOCK_DGRAM, 0);
  const timeval timeout{1, 0};
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

  const uint64_t now_ms = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
  const device_id_t remote = {remote_id};
  const device_id_t machine = {machine_id};
  pstop_msg_t bond;
  pstop_message_init(&bond);
  pstop_create_bond_message(&bond, now_ms, &remote, &machine, 1);
  uint8_t bytes[PSTOP_MESSAGE_SIZE];
  pstop_message_encode(&bond, bytes);

  sendto(sock, bytes, sizeof(bytes), 0, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
  const ssize_t received = recv(sock, bytes, sizeof(bytes), 0);
  close(sock);
  return static_cast<ssize_t>(PSTOP_MESSAGE_SIZE) == received;
}

}  // namespace pstop_test
