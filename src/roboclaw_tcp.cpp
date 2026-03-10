#include "roboclaw_hardware/roboclaw_tcp.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace roboclaw_hardware
{

RoboClawTcp::~RoboClawTcp()
{
  close();
}

bool RoboClawTcp::connect(
  const std::string & host, int port, double timeout_sec)
{
  close();

  host_ = host;
  port_ = port;
  timeout_sec_ = timeout_sec;

  fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd_ < 0) {
    return false;
  }

  int flag = 1;
  ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

  // TCP keepalive: detect dead peer within ~3s (1s idle + 3×1s probes).
  ::setsockopt(fd_, SOL_SOCKET, SO_KEEPALIVE, &flag, sizeof(flag));
  int idle = 1;
  ::setsockopt(fd_, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
  int interval = 1;
  ::setsockopt(fd_, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
  int count = 3;
  ::setsockopt(fd_, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));

  // TCP_USER_TIMEOUT: kernel gives up on unacked data after 3s.
  int user_timeout_ms = 3000;
  ::setsockopt(fd_, IPPROTO_TCP, TCP_USER_TIMEOUT,
    &user_timeout_ms, sizeof(user_timeout_ms));

  struct sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));

  if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    close();
    return false;
  }

  // Non-blocking connect with poll() — max 500ms, doesn't stall the RT loop.
  int orig_flags = ::fcntl(fd_, F_GETFL, 0);
  ::fcntl(fd_, F_SETFL, orig_flags | O_NONBLOCK);

  int rc = ::connect(fd_, reinterpret_cast<struct sockaddr *>(&addr),
                     sizeof(addr));
  if (rc < 0 && errno != EINPROGRESS) {
    close();
    return false;
  }

  if (rc != 0) {
    struct pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLOUT;
    int pr = ::poll(&pfd, 1, 500);
    if (pr <= 0) {
      close();
      return false;
    }
    int err = 0;
    socklen_t elen = sizeof(err);
    ::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &err, &elen);
    if (err != 0) {
      close();
      return false;
    }
  }

  // Restore blocking mode with normal per-byte timeout.
  ::fcntl(fd_, F_SETFL, orig_flags);
  set_socket_timeout(timeout_sec_);

  return true;
}

void RoboClawTcp::close()
{
  if (fd_ >= 0) {
    ::shutdown(fd_, SHUT_RDWR);
    ::close(fd_);
    fd_ = -1;
  }
}

bool RoboClawTcp::reconnect()
{
  close();
  return connect(host_, port_, timeout_sec_);
}

bool RoboClawTcp::is_alive() const noexcept
{
  if (fd_ < 0) {
    return false;
  }
  struct pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLOUT;
  int rc = ::poll(&pfd, 1, 0);
  if (rc <= 0) {
    return false;
  }
  // POLLERR / POLLHUP → peer is gone
  if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    return false;
  }
  // Double-check with getsockopt SO_ERROR
  int err = 0;
  socklen_t elen = sizeof(err);
  ::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &err, &elen);
  return err == 0;
}

bool RoboClawTcp::write(const uint8_t * data, size_t len)
{
  if (fd_ < 0 || len == 0) {
    return false;
  }

  size_t sent = 0;
  while (sent < len) {
    ssize_t n = ::send(fd_, data + sent, len - sent, MSG_NOSIGNAL);
    if (n <= 0) {
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

size_t RoboClawTcp::read(uint8_t * buffer, size_t len)
{
  if (fd_ < 0 || len == 0) {
    return 0;
  }

  size_t total = 0;
  while (total < len) {
    ssize_t n = ::recv(fd_, buffer + total, len - total, 0);
    if (n <= 0) {
      // timeout (EAGAIN/EWOULDBLOCK) or peer closed -- return short read
      break;
    }
    total += static_cast<size_t>(n);
  }
  return total;
}

void RoboClawTcp::flush()
{
  if (fd_ < 0) {
    return;
  }

  // Temporarily switch to non-blocking to drain without stalling.
  int flags = ::fcntl(fd_, F_GETFL, 0);
  ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

  uint8_t discard[256];
  while (true) {
    ssize_t n = ::recv(fd_, discard, sizeof(discard), 0);
    if (n <= 0) {
      break;
    }
  }

  // Restore blocking mode with the original timeout.
  ::fcntl(fd_, F_SETFL, flags);
  set_socket_timeout(timeout_sec_);
}

void RoboClawTcp::set_socket_timeout(double seconds)
{
  if (fd_ < 0) {
    return;
  }

  struct timeval tv{};
  tv.tv_sec = static_cast<time_t>(seconds);
  tv.tv_usec = static_cast<suseconds_t>(
    (seconds - static_cast<double>(tv.tv_sec)) * 1e6);

  ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  ::setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

}  // namespace roboclaw_hardware
