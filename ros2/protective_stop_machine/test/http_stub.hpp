// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// LoopbackHttpStub — a minimal, single-purpose HTTP/1.1 server on 127.0.0.1 for
// exercising the HardwareMachineBackend's libcurl client (http_get / http_post,
// append_response_body, the 2xx success path) WITHOUT a real ESP32 machn. It binds an
// ephemeral loopback port, answers every request with one canned status + body,
// and records the last request body so a test can assert what the backend sent
// (e.g. the configure() JSON). This is test scaffolding, not a general server.
#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

namespace pstop_test
{

class LoopbackHttpStub
{
public:
  explicit LoopbackHttpStub(std::string body, int http_status = 200)
  : body_(std::move(body))
    , status_(http_status)
  {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int reuse_addr = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    // ephemeral — the OS picks a free port
    addr.sin_port = 0;
    ::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    socklen_t addr_len = sizeof(addr);
    ::getsockname(listen_fd_, reinterpret_cast<sockaddr *>(&addr), &addr_len);
    port_ = ntohs(addr.sin_port);
    ::listen(listen_fd_, 4);
    running_ = true;
    thread_ = std::thread([this] {serve();});
  }

  ~LoopbackHttpStub()
  {
    running_ = false;
    // wake the blocking accept()
    ::shutdown(listen_fd_, SHUT_RDWR);
    ::close(listen_fd_);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  LoopbackHttpStub(const LoopbackHttpStub &) = delete;
  LoopbackHttpStub & operator=(const LoopbackHttpStub &) = delete;

  // http://127.0.0.1:<port> — feed straight into HardwareConfig::device_url.
  std::string url() const
  {
    return "http://127.0.0.1:" + std::to_string(port_);
  }

  int port() const
  {
    return port_;
  }

  // Body of the most recent request the backend sent (e.g. the configure POST).
  std::string last_request_body() const
  {
    std::lock_guard<std::mutex> lock(body_mutex_);
    return last_body_;
  }

  int request_count() const
  {
    return request_count_.load();
  }

private:
  void serve()
  {
    while (running_.load()) {
      int client_fd = ::accept(listen_fd_, nullptr, nullptr);
      if (client_fd < 0) {
        if (!running_.load()) {
          break;
        }
        continue;
      }
      handle(client_fd);
      ::close(client_fd);
    }
  }

  void handle(int client_fd)
  {
    std::string request;
    char chunk[2048];
    size_t header_end = std::string::npos;
    while (running_.load()) {
      ssize_t transferred = ::recv(client_fd, chunk, sizeof(chunk), 0);
      if (transferred <= 0) {
        break;
      }
      request.append(chunk, static_cast<size_t>(transferred));
      header_end = request.find("\r\n\r\n");
      if (header_end != std::string::npos) {
        break;
      }
    }
    // If there is a body (POST), drain Content-Length bytes so the captured
    // request is complete regardless of loopback segmentation.
    if (header_end != std::string::npos) {
      size_t expected_body_bytes = 0;
      size_t content_length_pos = request.find("Content-Length:");
      if (content_length_pos == std::string::npos) {
        content_length_pos = request.find("content-length:");
      }
      if (content_length_pos != std::string::npos) {
        expected_body_bytes = static_cast<size_t>(
          std::strtoul(request.c_str() + content_length_pos + 15, nullptr, 10));
      }
      size_t received_body_bytes = request.size() - (header_end + 4);
      while (received_body_bytes < expected_body_bytes && running_.load()) {
        ssize_t transferred = ::recv(client_fd, chunk, sizeof(chunk), 0);
        if (transferred <= 0) {
          break;
        }
        request.append(chunk, static_cast<size_t>(transferred));
        received_body_bytes += static_cast<size_t>(transferred);
      }
    }
    {
      std::lock_guard<std::mutex> lock(body_mutex_);
      last_body_ = header_end == std::string::npos ? "" : request.substr(header_end + 4);
    }
    request_count_.fetch_add(1);

    const std::string response = "HTTP/1.1 " + std::to_string(status_) + " OK\r\n" +
      "Content-Type: application/json\r\n" +
      "Content-Length: " + std::to_string(body_.size()) + "\r\n" + "Connection: close\r\n\r\n" +
      body_;
    size_t sent = 0;
    while (sent < response.size()) {
      ssize_t transferred = ::send(client_fd, response.data() + sent, response.size() - sent, 0);
      if (transferred <= 0) {
        break;
      }
      sent += static_cast<size_t>(transferred);
    }
  }

  int listen_fd_{-1};
  int port_{0};
  std::string body_;
  int status_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<int> request_count_{0};
  mutable std::mutex body_mutex_;
  std::string last_body_;
};

}  // namespace pstop_test

