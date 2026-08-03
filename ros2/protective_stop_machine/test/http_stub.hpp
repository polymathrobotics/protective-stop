// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// LoopbackHttpStub — a minimal, single-purpose HTTP/1.1 server on 127.0.0.1 for
// exercising the HardwareMachineBackend's libcurl client (http_get / http_post,
// write_cb, the 2xx success path) WITHOUT a real ESP32 machn. It binds an
// ephemeral loopback port, answers every request with one canned status + body,
// and records the last request body so a test can assert what the backend sent
// (e.g. the configure() JSON). This is test scaffolding, not a general server.
#ifndef PROTECTIVE_STOP_MACHINE__TEST__HTTP_STUB_HPP_
#define PROTECTIVE_STOP_MACHINE__TEST__HTTP_STUB_HPP_

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
  : body_(std::move(body)), status_(http_status)
  {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;  // ephemeral — the OS picks a free port
    ::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    socklen_t len = sizeof(addr);
    ::getsockname(fd_, reinterpret_cast<sockaddr *>(&addr), &len);
    port_ = ntohs(addr.sin_port);
    ::listen(fd_, 4);
    running_ = true;
    th_ = std::thread([this] { serve(); });
  }

  ~LoopbackHttpStub()
  {
    running_ = false;
    ::shutdown(fd_, SHUT_RDWR);  // wake the blocking accept()
    ::close(fd_);
    if (th_.joinable()) {
      th_.join();
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
    std::lock_guard<std::mutex> lk(mtx_);
    return last_body_;
  }

  int request_count() const
  {
    return count_.load();
  }

private:
  void serve()
  {
    while (running_.load()) {
      int c = ::accept(fd_, nullptr, nullptr);
      if (c < 0) {
        if (!running_.load()) {
          break;
        }
        continue;
      }
      handle(c);
      ::close(c);
    }
  }

  void handle(int c)
  {
    std::string req;
    char buf[2048];
    size_t header_end = std::string::npos;
    while (running_.load()) {
      ssize_t n = ::recv(c, buf, sizeof(buf), 0);
      if (n <= 0) {
        break;
      }
      req.append(buf, static_cast<size_t>(n));
      header_end = req.find("\r\n\r\n");
      if (header_end != std::string::npos) {
        break;
      }
    }
    // If there is a body (POST), drain Content-Length bytes so the captured
    // request is complete regardless of loopback segmentation.
    if (header_end != std::string::npos) {
      size_t want = 0;
      size_t lc = req.find("Content-Length:");
      if (lc == std::string::npos) {
        lc = req.find("content-length:");
      }
      if (lc != std::string::npos) {
        want = static_cast<size_t>(std::strtoul(req.c_str() + lc + 15, nullptr, 10));
      }
      size_t have = req.size() - (header_end + 4);
      while (have < want && running_.load()) {
        ssize_t n = ::recv(c, buf, sizeof(buf), 0);
        if (n <= 0) {
          break;
        }
        req.append(buf, static_cast<size_t>(n));
        have += static_cast<size_t>(n);
      }
    }
    {
      std::lock_guard<std::mutex> lk(mtx_);
      last_body_ = header_end == std::string::npos ? "" : req.substr(header_end + 4);
    }
    count_.fetch_add(1);

    const std::string resp = "HTTP/1.1 " + std::to_string(status_) + " OK\r\n" +
                             "Content-Type: application/json\r\n" +
                             "Content-Length: " + std::to_string(body_.size()) + "\r\n" +
                             "Connection: close\r\n\r\n" + body_;
    size_t sent = 0;
    while (sent < resp.size()) {
      ssize_t n = ::send(c, resp.data() + sent, resp.size() - sent, 0);
      if (n <= 0) {
        break;
      }
      sent += static_cast<size_t>(n);
    }
  }

  int fd_{-1};
  int port_{0};
  std::string body_;
  int status_;
  std::thread th_;
  std::atomic<bool> running_{false};
  std::atomic<int> count_{0};
  mutable std::mutex mtx_;
  std::string last_body_;
};

}  // namespace pstop_test

#endif  // PROTECTIVE_STOP_MACHINE__TEST__HTTP_STUB_HPP_
