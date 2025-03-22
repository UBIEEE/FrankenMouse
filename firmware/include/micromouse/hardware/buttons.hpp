#pragma once

#include <functional>
#include <micromouse/hardware/component.hpp>

namespace hardware {

class Buttons : public Component {
 protected:
  Buttons() = default;

 public:
  virtual void register_btn1_callback(std::function<void()> cb) = 0;
  virtual void register_btn2_callback(std::function<void()> cb) = 0;
};

}  // namespace hardware

/**
 * @brief Returns an instance of the platform-specific Buttons.
 *
 * This function is to be implemented by the user in platform-specific code.
 *
 * @return hardware::Buttons&
 */
hardware::Buttons& get_platform_buttons();
