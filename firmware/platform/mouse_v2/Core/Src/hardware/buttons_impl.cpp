#include "hardware/buttons_impl.hpp"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
    case BUTTON_1_Pin: break;
    case BUTTON_2_Pin: break;
    case IMU_INT1_Pin:
      // get_mouse_v2_imu().int1_handler();
      break;
  }
}

ButtonsImpl& get_mouse_v2_buttons() {
  static ButtonsImpl s_buttons;
  return s_buttons;
}

hardware::Buttons& get_platform_buttons() {
  return get_mouse_v2_buttons();
}
