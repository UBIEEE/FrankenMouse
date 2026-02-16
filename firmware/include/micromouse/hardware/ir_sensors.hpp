#pragma once

#include <micromouse/hardware/component.hpp>
#include <units/length.h>
#include <array>

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
   * Get the raw readings of the IR sensors (1 => max intensity, 0 => see nothing).
   *
   * @return Array of 4 readings, from left to right.
   */
  virtual const std::array<float, 4>& get_raw_readings() const = 0;

  /**
   * Get the distance readings of the IR sensors.
   *
   * @return Array of 4 distances, from left to right.
   */
  virtual const std::array<units::millimeter_t, 4>& get_distances() const = 0;

  const float& get_raw_reading(Sensor sensor) const {
    return get_raw_readings()[static_cast<int>(sensor)];
  }

  units::millimeter_t get_distance(Sensor sensor) const {
    return get_distances()[static_cast<int>(sensor)];
  }
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
