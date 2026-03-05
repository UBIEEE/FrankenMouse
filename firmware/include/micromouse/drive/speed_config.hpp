#pragma once

#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

namespace drive {

struct SpeedConstraints {
  units::millimeters_per_second_t linear_velocity;
  units::millimeters_per_second_squared_t linear_acceleration;

  units::degrees_per_second_t angular_velocity;
  units::degrees_per_second_squared_t angular_acceleration;

  units::millimeters_per_second_t turn_linear_velocity;
};

struct SpeedConfig {
  const SpeedConstraints slow_speeds = {
      .linear_velocity = 100_mmps,
      .linear_acceleration = 300_mmps_sq,
      .angular_velocity = 180_deg_per_s,
      .angular_acceleration = 360_deg_per_s_sq,
      .turn_linear_velocity = 100_mmps,
  };

  const SpeedConstraints normal_speeds = {
      .linear_velocity = 200_mmps,
      .linear_acceleration = 500_mmps_sq,
      .angular_velocity = 360_deg_per_s,
      .angular_acceleration = 720_deg_per_s_sq,
      .turn_linear_velocity = 200_mmps,
  };

  const SpeedConstraints fast_speeds = {
      .linear_velocity = 500_mmps,
      .linear_acceleration = 1000_mmps_sq,
      .angular_velocity = 360_deg_per_s,
      .angular_acceleration = 720_deg_per_s_sq,
      .turn_linear_velocity = 250_mmps,
  };
};

}  // namespace drive
