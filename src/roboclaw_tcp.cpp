#include "roboclaw_hardware/roboclaw_tcp.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/tcp.h>
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

  // Disable Nagle -- every write() must produce an immediate TCP segment so
  // the USR-K6 can start UART transmission without waiting for more data.
  int flag = 1;
  ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

  set_socket_timeout(timeout_sec_);

  struct sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));

  if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    close();
    return false;
  }

  // Use a generous 2-second timeout for the initial TCP handshake.
  // The per-byte timeout is applied separately after connection.
  struct timeval connect_tv{};
  connect_tv.tv_sec = 2;
  connect_tv.tv_usec = 0;
  ::setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &connect_tv, sizeof(connect_tv));

  if (::connect(fd_, reinterpret_cast<struct sockaddr *>(&addr),
                sizeof(addr)) < 0)
  {
    close();
    return false;
  }

  // Restore the normal per-byte timeout for send operations.
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
