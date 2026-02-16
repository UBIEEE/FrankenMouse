#pragma once

#include <units/angular_velocity.h>
#include <units/velocity.h>

namespace drive {

struct ChassisSpeeds {
  units::millimeters_per_second_t linear_velocity = 0_mmps;    // positive is forward
  units::degrees_per_second_t angular_velocity = 0_deg_per_s;  // positive is CCW
};

struct WheelSpeeds {
  units::millimeters_per_second_t left = 0_mmps;
  units::millimeters_per_second_t right = 0_mmps;
};

ChassisSpeeds to_chassis_speeds(const WheelSpeeds& wheel_speeds, units::millimeter_t track_width);
WheelSpeeds to_wheel_speeds(const ChassisSpeeds& chassis_speeds, units::millimeter_t track_width);

}  // namespace drive
