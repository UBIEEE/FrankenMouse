#pragma once

#include <micromouse/hardware/component.hpp>

namespace hardware {

class BatteryStatus : public Component {
 protected:
  BatteryStatus() = default;

 public:
  virtual float get_charge_percentage() = 0;

  virtual bool is_battery() const = 0;
  bool is_usb() const { return !is_battery(); }
};

}  // namespace hardware

/**
 * @brief Returns an instance of the platform-specific battery status.
 *
 * This function is to be implemented by the user in platform-specific code.
 *
 * @return hardware::BatteryStatus&
 */
hardware::BatteryStatus& get_platform_battery_status();
