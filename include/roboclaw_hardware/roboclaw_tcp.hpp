#ifndef ROBOCLAW_HARDWARE__ROBOCLAW_TCP_HPP_
#define ROBOCLAW_HARDWARE__ROBOCLAW_TCP_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace roboclaw_hardware
{

/// POSIX TCP socket wrapper providing the minimal I/O surface required by
/// the RoboClaw Packet Serial protocol.  Mirrors the API of the Python
/// RoboClawTCPPort class so the protocol layer can be a direct port.
///
/// TCP_NODELAY is enabled so each write() produces an immediate segment.
/// The USR-K6 collects arriving bytes and streams them at 115200 baud.
class RoboClawTcp
{
public:
  /// Default socket timeout in seconds.  50 ms covers LAN jitter while
  /// keeping the control loop responsive.
  static constexpr double kDefaultTimeoutSec = 0.05;

  RoboClawTcp() = default;
  ~RoboClawTcp();

  RoboClawTcp(const RoboClawTcp &) = delete;
  RoboClawTcp & operator=(const RoboClawTcp &) = delete;

  /// Connect to the USR-K6 Ethernet-to-Serial converter.
  /// @returns true on success.
  bool connect(const std::string & host, int port,
               double timeout_sec = kDefaultTimeoutSec);

  /// Close the TCP connection.
  void close();

  /// Close and re-open the connection with the last-used parameters.
  bool reconnect();

  /// Quick socket health probe (non-blocking poll for errors/hangup).
  bool is_alive() const noexcept;

  /// Send exactly @p len bytes.  Uses a loop to guarantee full delivery.
  /// @returns true if all bytes were sent.
  bool write(const uint8_t * data, size_t len);

  /// Read up to @p len bytes into @p buffer.
  /// @returns number of bytes actually read (may be < len on timeout,
  ///          matching pyserial short-read semantics).
  size_t read(uint8_t * buffer, size_t len);

  /// Non-blocking drain of the receive buffer.
  void flush();

  bool is_connected() const noexcept { return fd_ >= 0; }

private:
  void set_socket_timeout(double seconds);

  int fd_ = -1;
  std::string host_;
  int port_ = 0;
  double timeout_sec_ = kDefaultTimeoutSec;
};

}  // namespace roboclaw_hardware

#endif  // ROBOCLAW_HARDWARE__ROBOCLAW_TCP_HPP_
