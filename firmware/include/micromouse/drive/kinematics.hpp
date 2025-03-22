#pragma once

namespace drive {

struct ChassisSpeeds {
  float linear_velocity_mmps;  // positive is forward
  float angular_velocity_dps;  // positive is CCW
};

struct WheelSpeeds {
  float left_mmps;
  float right_mmps;
};

ChassisSpeeds to_chassis_speeds(const WheelSpeeds& wheel_speeds,
                                float track_width);

WheelSpeeds to_wheel_speeds(const ChassisSpeeds& chassis_speeds,
                            float track_width);

}  // namespace drive
