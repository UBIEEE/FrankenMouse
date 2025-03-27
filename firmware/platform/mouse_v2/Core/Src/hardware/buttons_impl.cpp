#include "hardware/buttons_impl.hpp"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  ButtonsImpl& buttons = get_mouse_v2_buttons();

  switch (GPIO_Pin) {
    case BUTTON_1_Pin:
      buttons.button_1_callback();
      break;
    case BUTTON_2_Pin:
      buttons.button_2_callback();
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
