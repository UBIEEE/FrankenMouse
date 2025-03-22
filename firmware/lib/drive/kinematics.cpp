#include <micromouse/drive/kinematics.hpp>

#include <numbers>

using namespace drive;

ChassisSpeeds drive::to_chassis_speeds(const WheelSpeeds& wheel_speeds,
                                       float b) {
  const float& vl = wheel_speeds.left_mmps;
  const float& vr = wheel_speeds.right_mmps;

  const float v = (vl + vr) / 2.f;
  const float w = ((vr - vl) / b) * (180.f / std::numbers::pi_v<float>);

  return ChassisSpeeds{v, w};
}

WheelSpeeds drive::to_wheel_speeds(const ChassisSpeeds& chassis_speeds,
                                   float b) {
  const float& v = chassis_speeds.linear_velocity_mmps;
  const float& w =
      chassis_speeds.angular_velocity_dps * (std::numbers::pi_v<float> / 180.f);

  const float vl = v - b * w / 2.f;
  const float vr = v + b * w / 2.f;

  return WheelSpeeds{vl, vr};
}
