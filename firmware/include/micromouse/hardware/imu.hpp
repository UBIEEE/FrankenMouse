#pragma once

#include <micromouse/hardware/component.hpp>
#include <units/angular_velocity.h>
#include <units/acceleration.h>

namespace hardware {

class IMU : public Component {
 protected:
  IMU() = default;

 public:
  enum Axis {
    X = 0,
    Y,
    Z,
  };

 public:
  virtual void set_standby(bool on_standby) { (void)on_standby; };
  virtual bool is_on_standby() { return false; }

  /**
   * @brief Get an angular velocity reading from the IMU.
   *
   * @return Angular velocity.
   */
  virtual units::degrees_per_second_t get_angular_velocity(Axis axis) = 0;

  /**
   * @brief Get a linear acceleration reading from the IMU.
   *
   * @return Linear acceleration.
   */
  virtual units::standard_gravity_t get_linear_accel(Axis axis) = 0;
};

}  // namespace hardware

/**
 * @brief Returns an instance of the platform-specific IMU.
 *
 * This function is to be implemented by the user in platform-specific code.
 *
 * @return hardware::IMU&
 */
hardware::IMU& get_platform_imu();
