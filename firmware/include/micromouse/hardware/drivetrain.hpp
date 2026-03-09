#pragma once

#include <micromouse/drive/kinematics.hpp>
#include <micromouse/drive/pose.hpp>
#include <micromouse/hardware/component.hpp>

namespace hardware {

class Drivetrain : public Component {
 protected:
  Drivetrain() = default;

 public:
  virtual void reset() {}
  virtual void stop() = 0;
  virtual void set_wheel_speeds(const drive::WheelSpeeds& wheel_speeds) = 0;
  virtual void set_chassis_speeds(const drive::ChassisSpeeds& chassis_speeds) = 0;

  struct EncoderData {
    units::millimeter_t position = 0_mm;
    units::millimeters_per_second_t velocity = 0_mmps;
    int32_t ticks = 0;
  };

  struct MotorData {
    EncoderData left_encoder;
    EncoderData right_encoder;
  };
  static_assert(sizeof(MotorData) == sizeof(float[6]));
};

}  // namespace hardware

/**
 * @brief Returns an instance of the platform-specific drivetrain.
 *
 * This function is to be implemented by the user in platform-specific code.
 *
 * @return hardware::Drivetrain&
 */
hardware::Drivetrain& get_platform_drivetrain();
