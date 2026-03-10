#include "roboclaw_hardware/unit_converter.hpp"

#include <cmath>
#include <sstream>

namespace roboclaw_hardware
{

UnitConverter::UnitConverter(
  double wheel_radius, int encoder_counts_per_rev, double gear_ratio)
: wheel_radius_(wheel_radius),
  encoder_counts_per_rev_(encoder_counts_per_rev),
  gear_ratio_(gear_ratio)
{
  if (wheel_radius <= 0.0) {
    throw std::invalid_argument("wheel_radius must be positive, got " +
      std::to_string(wheel_radius));
  }
  if (encoder_counts_per_rev <= 0) {
    throw std::invalid_argument("encoder_counts_per_rev must be positive, got " +
      std::to_string(encoder_counts_per_rev));
  }
  if (gear_ratio <= 0.0) {
    throw std::invalid_argument("gear_ratio must be positive, got " +
      std::to_string(gear_ratio));
  }

  wheel_circumference_ = kTwoPi * wheel_radius_;
  radians_per_count_ = kTwoPi / (encoder_counts_per_rev_ * gear_ratio_);
  counts_per_radian_ = (encoder_counts_per_rev_ * gear_ratio_) / kTwoPi;
  counts_per_meter_ = (encoder_counts_per_rev_ * gear_ratio_) / wheel_circumference_;
}

// ---------------------------------------------------------------------------
// Position
// ---------------------------------------------------------------------------

double UnitConverter::counts_to_radians(int32_t counts) const noexcept
{
  return static_cast<double>(counts) * radians_per_count_;
}

int32_t UnitConverter::radians_to_counts(double radians) const
{
  return safe_double_to_int32(radians * counts_per_radian_);
}

// ---------------------------------------------------------------------------
// Velocity
// ---------------------------------------------------------------------------

double UnitConverter::counts_per_sec_to_rad_per_sec(int32_t cps) const noexcept
{
  return static_cast<double>(cps) * radians_per_count_;
}

int32_t UnitConverter::rad_per_sec_to_counts_per_sec(double rad_s) const
{
  return safe_double_to_int32(rad_s * counts_per_radian_);
}

// ---------------------------------------------------------------------------
// Duty (open-loop)
// ---------------------------------------------------------------------------

int16_t UnitConverter::rad_per_sec_to_duty(double rad_s, double max_rad_s) const
{
  if (max_rad_s <= 0.0) {
    throw std::invalid_argument("max_rad_s must be positive");
  }
  double ratio = rad_s / max_rad_s;
  ratio = std::clamp(ratio, -1.0, 1.0);
  return static_cast<int16_t>(ratio * kMaxDuty);
}

// ---------------------------------------------------------------------------
// Linear distance
// ---------------------------------------------------------------------------

int32_t UnitConverter::meters_to_counts(double meters) const
{
  return safe_double_to_int32(meters * counts_per_meter_);
}

double UnitConverter::counts_to_meters(int32_t counts) const noexcept
{
  return static_cast<double>(counts) / counts_per_meter_;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

int32_t UnitConverter::safe_double_to_int32(double value) const
{
  if (std::abs(value) > static_cast<double>(kInt32Max)) {
    throw std::overflow_error(
      "Value " + std::to_string(value) + " exceeds int32 range");
  }
  return static_cast<int32_t>(std::lround(value));
}

std::string UnitConverter::to_string() const
{
  std::ostringstream os;
  os << "UnitConverter(wheel_radius=" << wheel_radius_
     << "m, cpr=" << encoder_counts_per_rev_
     << ", gear_ratio=" << gear_ratio_
     << ", counts_per_radian=" << counts_per_radian_
     << ", counts_per_meter=" << counts_per_meter_ << ")";
  return os.str();
}

}  // namespace roboclaw_hardware
