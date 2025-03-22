#pragma once

#include <micromouse/hardware/buttons.hpp>

#include "main.h"
#include "stm32wbxx_hal.h"

class ButtonsImpl : public hardware::Buttons {
  std::function<void()> m_btn1_cb;
  std::function<void()> m_btn2_cb;

 public:
  void register_btn1_callback(std::function<void()> cb) { m_btn1_cb = cb; }
  void register_btn2_callback(std::function<void()> cb) { m_btn2_cb = cb; }

 private:
  friend void ::HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

  void button_1_callback() {
    if (m_btn1_cb)
      m_btn1_cb();
  }

  void button_2_callback() {
    if (m_btn2_cb)
      m_btn2_cb();
  }
};

ButtonsImpl& get_mouse_v2_buttons();
