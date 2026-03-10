#include "roboclaw_hardware/roboclaw_protocol.hpp"

#include <algorithm>
#include <cstring>

namespace roboclaw_hardware
{

// ---------------------------------------------------------------------------
// CRC16 lookup table (polynomial 0x1021, matching Python utils.py)
// ---------------------------------------------------------------------------

std::array<uint16_t, 256> RoboClawProtocol::build_crc_table()
{
  std::array<uint16_t, 256> table{};
  for (int i = 0; i < 256; ++i) {
    uint16_t crc = static_cast<uint16_t>(i << 8);
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
    table[static_cast<size_t>(i)] = crc;
  }
  return table;
}

const std::array<uint16_t, 256> RoboClawProtocol::crc_table_ =
  RoboClawProtocol::build_crc_table();

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

RoboClawProtocol::RoboClawProtocol(RoboClawTcp & transport)
: transport_(transport)
{
}

// ---------------------------------------------------------------------------
// CRC helpers
// ---------------------------------------------------------------------------

void RoboClawProtocol::crc_clear()
{
  crc_ = 0;
}

void RoboClawProtocol::crc_update(uint8_t byte)
{
  crc_ = static_cast<uint16_t>(
    (crc_ << 8) ^ crc_table_[static_cast<uint8_t>((crc_ >> 8) ^ byte)]);
}

// ---------------------------------------------------------------------------
// Primitive wire I/O (big-endian, MSB first)
// ---------------------------------------------------------------------------

bool RoboClawProtocol::send_byte(uint8_t val)
{
  crc_update(val);
  return transport_.write(&val, 1);
}

bool RoboClawProtocol::send_word(uint16_t val)
{
  uint8_t buf[2] = {
    static_cast<uint8_t>((val >> 8) & 0xFF),
    static_cast<uint8_t>(val & 0xFF)
  };
  crc_update(buf[0]);
  crc_update(buf[1]);
  return transport_.write(buf, 2);
}

bool RoboClawProtocol::send_sword(int16_t val)
{
  return send_word(static_cast<uint16_t>(val));
}

bool RoboClawProtocol::send_long(uint32_t val)
{
  uint8_t buf[4] = {
    static_cast<uint8_t>((val >> 24) & 0xFF),
    static_cast<uint8_t>((val >> 16) & 0xFF),
    static_cast<uint8_t>((val >> 8)  & 0xFF),
    static_cast<uint8_t>(val & 0xFF)
  };
  crc_update(buf[0]);
  crc_update(buf[1]);
  crc_update(buf[2]);
  crc_update(buf[3]);
  return transport_.write(buf, 4);
}

bool RoboClawProtocol::send_slong(int32_t val)
{
  return send_long(static_cast<uint32_t>(val));
}

bool RoboClawProtocol::recv_byte(uint8_t & out)
{
  uint8_t buf[1];
  if (transport_.read(buf, 1) != 1) {
    return false;
  }
  crc_update(buf[0]);
  out = buf[0];
  return true;
}

bool RoboClawProtocol::recv_word(uint16_t & out)
{
  uint8_t buf[2];
  if (transport_.read(buf, 2) != 2) {
    return false;
  }
  crc_update(buf[0]);
  crc_update(buf[1]);
  out = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
  return true;
}

bool RoboClawProtocol::recv_sword(int16_t & out)
{
  uint16_t raw;
  if (!recv_word(raw)) { return false; }
  out = static_cast<int16_t>(raw);
  return true;
}

bool RoboClawProtocol::recv_long(uint32_t & out)
{
  uint8_t buf[4];
  if (transport_.read(buf, 4) != 4) {
    return false;
  }
  crc_update(buf[0]);
  crc_update(buf[1]);
  crc_update(buf[2]);
  crc_update(buf[3]);
  out = (static_cast<uint32_t>(buf[0]) << 24) |
        (static_cast<uint32_t>(buf[1]) << 16) |
        (static_cast<uint32_t>(buf[2]) << 8)  |
        static_cast<uint32_t>(buf[3]);
  return true;
}

bool RoboClawProtocol::recv_slong(int32_t & out)
{
  uint32_t raw;
  if (!recv_long(raw)) { return false; }
  out = static_cast<int32_t>(raw);
  return true;
}

// ---------------------------------------------------------------------------
// Command framing
// ---------------------------------------------------------------------------

bool RoboClawProtocol::send_command(uint8_t address, uint8_t command)
{
  crc_clear();
  uint8_t header[2] = {address, command};
  crc_update(header[0]);
  crc_update(header[1]);
  return transport_.write(header, 2);
}

bool RoboClawProtocol::write_checksum_and_ack()
{
  uint16_t checksum = crc_get();
  uint8_t crc_buf[2] = {
    static_cast<uint8_t>((checksum >> 8) & 0xFF),
    static_cast<uint8_t>(checksum & 0xFF)
  };
  if (!transport_.write(crc_buf, 2)) {
    return false;
  }
  // Read the single-byte ACK.
  uint8_t ack;
  return (transport_.read(&ack, 1) == 1);
}

bool RoboClawProtocol::verify_read_checksum()
{
  uint16_t expected = crc_get();
  uint8_t buf[2];
  if (transport_.read(buf, 2) != 2) {
    return false;
  }
  uint16_t received = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
  return (received == expected);
}

// ===========================================================================
// Telemetry read commands
// ===========================================================================

VersionResult RoboClawProtocol::ReadVersion(uint8_t address)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::GET_VERSION)) { continue; }

    std::string version;
    bool valid = true;
    while (true) {
      uint8_t ch;
      if (!recv_byte(ch)) { valid = false; break; }
      if (ch == 0) { break; }
      version.push_back(static_cast<char>(ch));
      if (version.size() > 64) { valid = false; break; }
    }

    if (valid && verify_read_checksum()) {
      return {true, version};
    }
  }
  return {false, {}};
}

EncodersResult RoboClawProtocol::GetEncoders(uint8_t address)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::GET_ENCODERS)) { continue; }

    uint32_t e1, e2;
    if (!recv_long(e1)) { continue; }
    if (!recv_long(e2)) { continue; }

    if (verify_read_checksum()) {
      return {true,
              static_cast<int32_t>(e1),
              static_cast<int32_t>(e2)};
    }
  }
  return {false, 0, 0};
}

SpeedsResult RoboClawProtocol::GetSpeeds(uint8_t address)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::GET_SPEEDS)) { continue; }

    int32_t s1, s2;
    if (!recv_slong(s1)) { continue; }
    if (!recv_slong(s2)) { continue; }

    if (verify_read_checksum()) {
      return {true, s1, s2};
    }
  }
  return {false, 0, 0};
}

ErrorResult RoboClawProtocol::ReadError(uint8_t address)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::GET_ERROR)) { continue; }

    uint32_t error;
    if (!recv_long(error)) { continue; }

    if (verify_read_checksum()) {
      return {true, error};
    }
  }
  return {false, 0};
}

CurrentsResult RoboClawProtocol::ReadCurrents(uint8_t address)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::GET_CURRENTS)) { continue; }

    int16_t c1, c2;
    if (!recv_sword(c1)) { continue; }
    if (!recv_sword(c2)) { continue; }

    if (verify_read_checksum()) {
      return {true, c1, c2};
    }
  }
  return {false, 0, 0};
}

TempsResult RoboClawProtocol::GetTemps(uint8_t address)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::GET_TEMPS)) { continue; }

    uint16_t t1_raw, t2_raw;
    if (!recv_word(t1_raw)) { continue; }
    if (!recv_word(t2_raw)) { continue; }

    if (verify_read_checksum()) {
      return {true,
              static_cast<int16_t>(t1_raw),
              static_cast<int16_t>(t2_raw)};
    }
  }
  return {false, 0, 0};
}

VoltsResult RoboClawProtocol::GetVolts(uint8_t address)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::GET_VOLTS)) { continue; }

    uint16_t main_bat, logic_bat;
    if (!recv_word(main_bat)) { continue; }
    if (!recv_word(logic_bat)) { continue; }

    if (verify_read_checksum()) {
      return {true, main_bat, logic_bat};
    }
  }
  return {false, 0, 0};
}

SpeedErrorsResult RoboClawProtocol::GetSpeedErrors(uint8_t address)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::GET_SPEED_ERRORS)) { continue; }

    int16_t e1, e2;
    if (!recv_sword(e1)) { continue; }
    if (!recv_sword(e2)) { continue; }

    if (verify_read_checksum()) {
      return {true, e1, e2};
    }
  }
  return {false, 0, 0};
}

PosErrorsResult RoboClawProtocol::GetPosErrors(uint8_t address)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::GET_POS_ERRORS)) { continue; }

    int32_t e1, e2;
    if (!recv_slong(e1)) { continue; }
    if (!recv_slong(e2)) { continue; }

    if (verify_read_checksum()) {
      return {true, e1, e2};
    }
  }
  return {false, 0, 0};
}

BuffersResult RoboClawProtocol::ReadBuffers(uint8_t address)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::GET_BUFFERS)) { continue; }

    uint8_t b1, b2;
    if (!recv_byte(b1)) { continue; }
    if (!recv_byte(b2)) { continue; }

    if (verify_read_checksum()) {
      return {true, b1, b2};
    }
  }
  return {false, 0, 0};
}

PWMsResult RoboClawProtocol::GetPWMs(uint8_t address)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::GET_PWMS)) { continue; }

    int16_t p1, p2;
    if (!recv_sword(p1)) { continue; }
    if (!recv_sword(p2)) { continue; }

    if (verify_read_checksum()) {
      return {true, p1, p2};
    }
  }
  return {false, 0, 0};
}

// ===========================================================================
// Motor write commands
// ===========================================================================

bool RoboClawProtocol::DutyM1M2(uint8_t address, int16_t m1, int16_t m2)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::MIXED_DUTY))   { continue; }
    if (!send_sword(m1))                            { continue; }
    if (!send_sword(m2))                            { continue; }
    if (write_checksum_and_ack()) { return true; }
  }
  return false;
}

bool RoboClawProtocol::DutyAccelM1M2(
  uint8_t address,
  int16_t duty1, uint32_t accel1,
  int16_t duty2, uint32_t accel2)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::MIXED_DUTY_ACCEL)) { continue; }
    if (!send_sword(duty1))  { continue; }
    if (!send_long(accel1))  { continue; }
    if (!send_sword(duty2))  { continue; }
    if (!send_long(accel2))  { continue; }
    if (write_checksum_and_ack()) { return true; }
  }
  return false;
}

bool RoboClawProtocol::SpeedM1M2(
  uint8_t address, int32_t m1, int32_t m2)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::MIXED_SPEED)) { continue; }
    if (!send_slong(m1))                           { continue; }
    if (!send_slong(m2))                           { continue; }
    if (write_checksum_and_ack()) { return true; }
  }
  return false;
}

bool RoboClawProtocol::SpeedAccelM1M2(
  uint8_t address, uint32_t accel, int32_t speed1, int32_t speed2)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::MIXED_SPEED_ACCEL)) { continue; }
    if (!send_long(accel))    { continue; }
    if (!send_slong(speed1))  { continue; }
    if (!send_slong(speed2))  { continue; }
    if (write_checksum_and_ack()) { return true; }
  }
  return false;
}

bool RoboClawProtocol::SpeedAccelDistM1M2(
  uint8_t address, uint32_t accel,
  int32_t speed1, uint32_t dist1,
  int32_t speed2, uint32_t dist2,
  uint8_t buffer)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::MIXED_SPEED_ACCEL_DIST)) { continue; }
    if (!send_long(accel))    { continue; }
    if (!send_slong(speed1))  { continue; }
    if (!send_long(dist1))    { continue; }
    if (!send_slong(speed2))  { continue; }
    if (!send_long(dist2))    { continue; }
    if (!send_byte(buffer))   { continue; }
    if (write_checksum_and_ack()) { return true; }
  }
  return false;
}

bool RoboClawProtocol::SpeedAccelDeccelPosM1M2(
  uint8_t address,
  uint32_t accel1, uint32_t speed1, uint32_t deccel1, int32_t pos1,
  uint32_t accel2, uint32_t speed2, uint32_t deccel2, int32_t pos2,
  uint8_t buffer)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::MIXED_SPEED_ACCEL_DECCEL_POS)) { continue; }
    if (!send_long(accel1))   { continue; }
    if (!send_long(speed1))   { continue; }
    if (!send_long(deccel1))  { continue; }
    if (!send_slong(pos1))    { continue; }
    if (!send_long(accel2))   { continue; }
    if (!send_long(speed2))   { continue; }
    if (!send_long(deccel2))  { continue; }
    if (!send_slong(pos2))    { continue; }
    if (!send_byte(buffer))   { continue; }
    if (write_checksum_and_ack()) { return true; }
  }
  return false;
}

// ===========================================================================
// Configuration commands
// ===========================================================================

bool RoboClawProtocol::SetTimeout(uint8_t address, uint32_t timeout_ms)
{
  // RoboClaw expects a single byte in 10ms units (0-255 → 0-2550ms).
  uint8_t val = static_cast<uint8_t>(
    std::min(timeout_ms / 10u, static_cast<uint32_t>(255)));

  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::SET_TIMEOUT)) { continue; }
    if (!send_byte(val))                           { continue; }
    if (write_checksum_and_ack()) { return true; }
  }
  return false;
}

bool RoboClawProtocol::ResetEncoders(uint8_t address)
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    transport_.flush();

    if (!send_command(address, cmd::RESET_ENCODERS)) { continue; }
    if (write_checksum_and_ack()) { return true; }
  }
  return false;
}

}  // namespace roboclaw_hardware
