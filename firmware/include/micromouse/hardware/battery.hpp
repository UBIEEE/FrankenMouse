#pragma once

#include <micromouse/hardware/component.hpp>
#include <units/voltage.h>

namespace hardware {

class Battery : public Component {
 protected:
  Battery() = default;

 public:
  virtual float get_charge_percentage() const = 0;
  virtual units::volt_t get_voltage() const = 0;

  virtual bool is_battery() const = 0;
  bool is_usb() const { return !is_battery(); }
};

}  // namespace hardware

/**
 * Returns an instance of the platform Battery.
 *
 * This function is to be implemented by the user in platform-specific code.
 *
 * @return The battery component.
 */
hardware::Battery& get_platform_battery();
