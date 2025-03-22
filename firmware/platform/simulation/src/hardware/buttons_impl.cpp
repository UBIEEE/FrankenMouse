#include <simulation/hardware/buttons_impl.hpp>

using namespace std::placeholders;

ButtonsImpl::ButtonsImpl() : Node("micromouse_buttons") {
  m_button1_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/simulation/buttons/button1", 10,
      std::bind(&ButtonsImpl::button_1_callback, this, _1));
  m_button2_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/simulation/buttons/button2", 10,
      std::bind(&ButtonsImpl::button_2_callback, this, _1));
}

void ButtonsImpl::button_1_callback(const std_msgs::msg::Bool& msg) {
  if (msg.data && m_btn1_cb) {
    m_btn1_cb();
    printf("Button 1 pressed\n");
  }
}

void ButtonsImpl::button_2_callback(const std_msgs::msg::Bool& msg) {
  if (msg.data && m_btn2_cb) {
    m_btn2_cb();
    printf("Button 2 pressed\n");
  }
}

std::shared_ptr<ButtonsImpl> get_simulation_buttons() {
  static auto buttons = std::make_shared<ButtonsImpl>();
  return buttons;
}

hardware::Buttons& get_platform_buttons() {
  return *get_simulation_buttons();
}
