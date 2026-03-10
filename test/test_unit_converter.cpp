#include "roboclaw_hardware/unit_converter.hpp"

#include <gtest/gtest.h>
#include <cmath>

using roboclaw_hardware::UnitConverter;

class UnitConverterTest : public ::testing::Test
{
protected:
  // Typical robot: 0.1 m radius, 1000 CPR, no gear reduction.
  UnitConverter simple{0.1, 1000, 1.0};

  // Our robot: 0.2 m radius, 1000 CPR, 16:1 gear ratio.
  UnitConverter robot{0.2, 1000, 16.0};
};

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

TEST_F(UnitConverterTest, ThrowsOnInvalidParams)
{
  EXPECT_THROW(UnitConverter(0.0, 1000, 1.0), std::invalid_argument);
  EXPECT_THROW(UnitConverter(-0.1, 1000, 1.0), std::invalid_argument);
  EXPECT_THROW(UnitConverter(0.1, 0, 1.0), std::invalid_argument);
  EXPECT_THROW(UnitConverter(0.1, 1000, 0.0), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Position: counts <-> radians
// ---------------------------------------------------------------------------

TEST_F(UnitConverterTest, OneRevolutionSimple)
{
  double rad = simple.counts_to_radians(1000);
  EXPECT_NEAR(rad, 2.0 * M_PI, 1e-9);
}

TEST_F(UnitConverterTest, RadiansToCountsRoundTrip)
{
  int32_t counts = simple.radians_to_counts(2.0 * M_PI);
  EXPECT_EQ(counts, 1000);

  double back = simple.counts_to_radians(counts);
  EXPECT_NEAR(back, 2.0 * M_PI, 1e-9);
}

TEST_F(UnitConverterTest, GearRatioApplied)
{
  // With 16:1 gear ratio, 1000 encoder counts = 1 motor rev,
  // but only 1/16 of a wheel rev.
  double rad = robot.counts_to_radians(1000);
  EXPECT_NEAR(rad, 2.0 * M_PI / 16.0, 1e-9);

  // 1 full wheel revolution = 16000 encoder counts.
  int32_t counts = robot.radians_to_counts(2.0 * M_PI);
  EXPECT_EQ(counts, 16000);
}

TEST_F(UnitConverterTest, NegativeCounts)
{
  double rad = simple.counts_to_radians(-500);
  EXPECT_NEAR(rad, -M_PI, 1e-9);
}

// ---------------------------------------------------------------------------
// Velocity: counts/sec <-> rad/sec
// ---------------------------------------------------------------------------

TEST_F(UnitConverterTest, VelocityConversion)
{
  double rad_s = simple.counts_per_sec_to_rad_per_sec(1000);
  EXPECT_NEAR(rad_s, 2.0 * M_PI, 1e-9);

  int32_t cps = simple.rad_per_sec_to_counts_per_sec(2.0 * M_PI);
  EXPECT_EQ(cps, 1000);
}

TEST_F(UnitConverterTest, VelocityWithGearRatio)
{
  // 16 km/h target:
  // linear speed = 16000/3600 = 4.444 m/s
  // angular speed = v / r = 4.444 / 0.2 = 22.22 rad/s
  // counts/sec = 22.22 * (1000 * 16) / (2*pi) = 22.22 * 2546.48 = ~56588

  double v_linear = 16000.0 / 3600.0;
  double omega = v_linear / 0.2;
  int32_t cps = robot.rad_per_sec_to_counts_per_sec(omega);

  EXPECT_GT(cps, 56000);
  EXPECT_LT(cps, 57000);
}

// ---------------------------------------------------------------------------
// Duty
// ---------------------------------------------------------------------------

TEST_F(UnitConverterTest, DutyClamp)
{
  // Above max -> clamp to +32767
  int16_t d = simple.rad_per_sec_to_duty(20.0, 10.0);
  EXPECT_EQ(d, 32767);

  // Below -max -> clamp to -32767
  d = simple.rad_per_sec_to_duty(-20.0, 10.0);
  EXPECT_EQ(d, -32767);

  // Half speed
  d = simple.rad_per_sec_to_duty(5.0, 10.0);
  EXPECT_NEAR(d, 16383, 1);
}

TEST_F(UnitConverterTest, DutyThrowsOnZeroMax)
{
  EXPECT_THROW(simple.rad_per_sec_to_duty(1.0, 0.0), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Distance: meters <-> counts
// ---------------------------------------------------------------------------

TEST_F(UnitConverterTest, MetersToCounts)
{
  // One wheel circumference = 2*pi*0.1 = 0.6283 m -> 1000 counts
  double circumference = 2.0 * M_PI * 0.1;
  int32_t c = simple.meters_to_counts(circumference);
  EXPECT_EQ(c, 1000);
}

TEST_F(UnitConverterTest, CountsToMeters)
{
  double m = simple.counts_to_meters(1000);
  EXPECT_NEAR(m, 2.0 * M_PI * 0.1, 1e-9);
}

// ---------------------------------------------------------------------------
// Overflow
// ---------------------------------------------------------------------------

TEST_F(UnitConverterTest, OverflowProtection)
{
  EXPECT_THROW(simple.radians_to_counts(1e15), std::overflow_error);
  EXPECT_THROW(simple.meters_to_counts(1e12), std::overflow_error);
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

TEST_F(UnitConverterTest, Accessors)
{
  EXPECT_DOUBLE_EQ(robot.wheel_radius(), 0.2);
  EXPECT_DOUBLE_EQ(robot.gear_ratio(), 16.0);
  EXPECT_EQ(robot.encoder_cpr(), 1000);
  EXPECT_NEAR(robot.wheel_circumference(), 2.0 * M_PI * 0.2, 1e-9);
}

TEST_F(UnitConverterTest, ToString)
{
  auto s = robot.to_string();
  EXPECT_TRUE(s.find("0.2") != std::string::npos);
  EXPECT_TRUE(s.find("16") != std::string::npos);
}
