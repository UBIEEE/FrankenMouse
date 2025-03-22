#pragma once

#include <micromouse/hardware/component.hpp>

namespace hardware {

class IRSensors : public Component {
 protected:
  IRSensors() = default;

 public:
  enum Sensor {
    FAR_LEFT = 0,
    MID_LEFT = 1,
    MID_RIGHT = 2,
    FAR_RIGHT = 3,
  };

  virtual void set_enabled(bool enabled) { (void)enabled; }
  virtual bool is_enabled() const { return true; }

  /**
   * @brief Get the raw readings of the IR sensors (1 => max intensity, 0 => see
   * nothing).
   *
   * @return float* Pointer to array of 4 readings, from left to right.
   */
  virtual const float* get_raw_readings() const = 0;

  /**
   * @brief Get the distance readings of the IR sensors, in millimeters.
   *
   * @return float* Pointer to array of 4 distances, from left to right.
   */
  virtual const float* get_distances_mm() const = 0;
};

}  // namespace hardware

/**
 * @brief Returns an instance of the platform-specific IR sensors.
 *
 * This function is to be implemented by the user in platform-specific code.
 *
 * @return hardware::IRSensors&
 */
hardware::IRSensors& get_platform_ir_sensors();
