#ifndef ROBOCLAW_HARDWARE__ROBOCLAW_PROTOCOL_HPP_
#define ROBOCLAW_HARDWARE__ROBOCLAW_PROTOCOL_HPP_

#include "roboclaw_hardware/roboclaw_tcp.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

namespace roboclaw_hardware
{

// ---------------------------------------------------------------------------
// Result types (mirror basicmicro_python/types.py)
// ---------------------------------------------------------------------------

struct VersionResult    { bool ok; std::string version; };
struct EncodersResult   { bool ok; int32_t enc1; int32_t enc2; };
struct SpeedsResult     { bool ok; int32_t speed1; int32_t speed2; };
struct ErrorResult      { bool ok; uint32_t error; };
struct CurrentsResult   { bool ok; int16_t current1; int16_t current2; };
struct TempsResult      { bool ok; int16_t temp1; int16_t temp2; };
struct VoltsResult      { bool ok; uint16_t main_bat; uint16_t logic_bat; };
struct SpeedErrorsResult{ bool ok; int16_t error1; int16_t error2; };
struct PosErrorsResult  { bool ok; int32_t error1; int32_t error2; };
struct BuffersResult    { bool ok; uint8_t buffer1; uint8_t buffer2; };
struct PWMsResult       { bool ok; int16_t pwm1; int16_t pwm2; };

/// Full status as returned by GetStatus (cmd 73).
/// The upstream library returns 20+ fields; we store the raw 32-bit words
/// and decode lazily.
struct StatusResult {
  bool ok;
  uint32_t error_status;
  uint32_t warn_status;
  // Raw fields kept available for future expansion.
  std::vector<uint32_t> raw;
};

// ---------------------------------------------------------------------------
// Command bytes (subset from basicmicro_python/commands.py)
// ---------------------------------------------------------------------------

namespace cmd
{
  constexpr uint8_t SET_TIMEOUT              = 14;
  constexpr uint8_t GET_TIMEOUT              = 15;
  constexpr uint8_t RESET_ENCODERS           = 20;
  constexpr uint8_t GET_VERSION              = 21;
  constexpr uint8_t MIXED_DUTY               = 34;
  constexpr uint8_t MIXED_SPEED              = 37;
  constexpr uint8_t MIXED_SPEED_ACCEL        = 40;
  constexpr uint8_t MIXED_SPEED_ACCEL_DIST   = 46;
  constexpr uint8_t GET_BUFFERS              = 47;
  constexpr uint8_t GET_PWMS                 = 48;
  constexpr uint8_t GET_CURRENTS             = 49;
  constexpr uint8_t MIXED_DUTY_ACCEL         = 54;
  constexpr uint8_t MIXED_SPEED_ACCEL_DECCEL_POS = 67;
  constexpr uint8_t GET_STATUS               = 73;
  constexpr uint8_t GET_ENCODERS             = 78;
  constexpr uint8_t GET_ERROR                = 90;
  constexpr uint8_t GET_VOLTS                = 100;
  constexpr uint8_t GET_TEMPS                = 101;
  constexpr uint8_t GET_SPEEDS               = 108;
  constexpr uint8_t GET_SPEED_ERRORS         = 111;
  constexpr uint8_t GET_POS_ERRORS           = 114;
}  // namespace cmd

// ---------------------------------------------------------------------------
// Error / warning bit flags (from controller.py constants)
// ---------------------------------------------------------------------------

namespace err
{
  constexpr uint32_t NONE          = 0x0000;
  constexpr uint32_t ESTOP         = 0x0001;
  constexpr uint32_t TEMP          = 0x0002;
  constexpr uint32_t TEMP2         = 0x0004;
  constexpr uint32_t MBAT_HIGH     = 0x0008;
  constexpr uint32_t LBAT_HIGH     = 0x0010;
  constexpr uint32_t LBAT_LOW      = 0x0020;
  constexpr uint32_t SPEED1        = 0x0100;
  constexpr uint32_t SPEED2        = 0x0200;
  constexpr uint32_t POS1          = 0x0400;
  constexpr uint32_t POS2          = 0x0800;
  constexpr uint32_t CURRENT_M1    = 0x1000;
  constexpr uint32_t CURRENT_M2    = 0x2000;
  constexpr uint32_t MBAT_LOW      = 0x4000;
}  // namespace err

// ---------------------------------------------------------------------------
// Protocol class
// ---------------------------------------------------------------------------

/// Implements the Basicmicro Packet Serial protocol over a RoboClawTcp
/// transport.  CRC16 is computed with a pre-built lookup table (polynomial
/// 0x1021, matching the upstream Python library).
///
/// Every public command method retries up to kMaxRetries times on timeout
/// or CRC mismatch.
class RoboClawProtocol
{
public:
  static constexpr int kMaxRetries = 3;

  explicit RoboClawProtocol(RoboClawTcp & transport);

  // -- Telemetry reads -------------------------------------------------------
  VersionResult    ReadVersion(uint8_t address);
  EncodersResult   GetEncoders(uint8_t address);
  SpeedsResult     GetSpeeds(uint8_t address);
  ErrorResult      ReadError(uint8_t address);
  CurrentsResult   ReadCurrents(uint8_t address);
  TempsResult      GetTemps(uint8_t address);
  VoltsResult      GetVolts(uint8_t address);
  SpeedErrorsResult GetSpeedErrors(uint8_t address);
  PosErrorsResult  GetPosErrors(uint8_t address);
  BuffersResult    ReadBuffers(uint8_t address);
  PWMsResult       GetPWMs(uint8_t address);

  // -- Motor commands ---------------------------------------------------------
  bool DutyM1M2(uint8_t address, int16_t m1, int16_t m2);
  bool DutyAccelM1M2(uint8_t address,
                     int16_t duty1, uint32_t accel1,
                     int16_t duty2, uint32_t accel2);
  bool SpeedM1M2(uint8_t address, int32_t m1, int32_t m2);
  bool SpeedAccelM1M2(uint8_t address, uint32_t accel,
                      int32_t speed1, int32_t speed2);
  bool SpeedAccelDistM1M2(uint8_t address, uint32_t accel,
                          int32_t speed1, uint32_t dist1,
                          int32_t speed2, uint32_t dist2,
                          uint8_t buffer);
  bool SpeedAccelDeccelPosM1M2(
    uint8_t address,
    uint32_t accel1, uint32_t speed1, uint32_t deccel1, int32_t pos1,
    uint32_t accel2, uint32_t speed2, uint32_t deccel2, int32_t pos2,
    uint8_t buffer);

  // -- Configuration ----------------------------------------------------------
  bool SetTimeout(uint8_t address, uint32_t timeout_ms);
  bool ResetEncoders(uint8_t address);

private:
  // CRC16 (CCITT polynomial 0x1021)
  void     crc_clear();
  void     crc_update(uint8_t byte);
  uint16_t crc_get() const { return crc_; }

  // Primitive wire I/O -- each updates the running CRC.
  bool send_byte(uint8_t val);
  bool send_word(uint16_t val);
  bool send_sword(int16_t val);
  bool send_long(uint32_t val);
  bool send_slong(int32_t val);

  bool recv_byte(uint8_t & out);
  bool recv_word(uint16_t & out);
  bool recv_sword(int16_t & out);
  bool recv_long(uint32_t & out);
  bool recv_slong(int32_t & out);

  // Send address + command, initializing CRC.
  bool send_command(uint8_t address, uint8_t command);

  // Write CRC and read single-byte ACK.
  bool write_checksum_and_ack();

  // Read 16-bit CRC from the wire and verify against the running CRC.
  bool verify_read_checksum();

  RoboClawTcp & transport_;
  uint16_t crc_ = 0;

  // Pre-computed CRC lookup table (256 entries).
  static const std::array<uint16_t, 256> crc_table_;
  static std::array<uint16_t, 256> build_crc_table();
};

}  // namespace roboclaw_hardware

#endif  // ROBOCLAW_HARDWARE__ROBOCLAW_PROTOCOL_HPP_
