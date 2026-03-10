#ifndef ROBOCLAW_HARDWARE__UNIT_CONVERTER_HPP_
#define ROBOCLAW_HARDWARE__UNIT_CONVERTER_HPP_

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace roboclaw_hardware
{

/// Converts between ROS2 standard units (radians, m/s) and RoboClaw
/// encoder counts.  All conversion factors are pre-computed at construction
/// time so the per-cycle cost is a single multiply.
class UnitConverter
{
public:
  /// @throws std::invalid_argument if any parameter is non-positive.
  UnitConverter(double wheel_radius, int encoder_counts_per_rev, double gear_ratio);

  // -- position --
  double   counts_to_radians(int32_t counts) const noexcept;
  int32_t  radians_to_counts(double radians) const;

  // -- velocity --
  double   counts_per_sec_to_rad_per_sec(int32_t cps) const noexcept;
  int32_t  rad_per_sec_to_counts_per_sec(double rad_s) const;

  // -- duty (open-loop) --
  int16_t  rad_per_sec_to_duty(double rad_s, double max_rad_s = 10.0) const;

  // -- linear distance --
  int32_t  meters_to_counts(double meters) const;
  double   counts_to_meters(int32_t counts) const noexcept;

  // -- convenience --
  double wheel_radius()      const noexcept { return wheel_radius_; }
  double wheel_circumference() const noexcept { return wheel_circumference_; }
  double gear_ratio()         const noexcept { return gear_ratio_; }
  int    encoder_cpr()        const noexcept { return encoder_counts_per_rev_; }

  std::string to_string() const;

private:
  static constexpr double kTwoPi = 2.0 * M_PI;
  static constexpr int32_t kInt32Max = INT32_MAX;
  static constexpr int16_t kMaxDuty = 32767;

  int32_t safe_double_to_int32(double value) const;

  double wheel_radius_;
  int    encoder_counts_per_rev_;
  double gear_ratio_;

  double wheel_circumference_;
  double radians_per_count_;
  double counts_per_radian_;
  double counts_per_meter_;
};

}  // namespace roboclaw_hardware

#endif  // ROBOCLAW_HARDWARE__UNIT_CONVERTER_HPP_
