#include <micromouse/drive/kinematics.hpp>

#include <numbers>

using namespace drive;

ChassisSpeeds drive::to_chassis_speeds(const WheelSpeeds& wheel_speeds, units::millimeter_t b) {
  const units::millimeters_per_second_t& vl = wheel_speeds.left;
  const units::millimeters_per_second_t& vr = wheel_speeds.right;

  const units::millimeters_per_second_t v = (vl + vr) / 2.f;
  const units::radians_per_second_t w{((vr - vl) / b).value()};

  return ChassisSpeeds{v, w};
}

WheelSpeeds drive::to_wheel_speeds(const ChassisSpeeds& chassis_speeds, units::millimeter_t b) {
  const units::millimeters_per_second_t& v = chassis_speeds.linear_velocity;
  const units::radians_per_second_t w = chassis_speeds.angular_velocity;

  const units::millimeters_per_second_t vl = v - units::millimeters_per_second_t{(b * w / 2.f).value()};
  const units::millimeters_per_second_t vr = v + units::millimeters_per_second_t{(b * w / 2.f).value()};

  return WheelSpeeds{vl, vr};
}
