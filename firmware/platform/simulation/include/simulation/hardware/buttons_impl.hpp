#pragma once

#include <micromouse/hardware/buttons.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class ButtonsImpl : public hardware::Buttons, public rclcpp::Node {
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_button1_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_button2_sub;

  std::function<void()> m_btn1_cb;
  std::function<void()> m_btn2_cb;

 public:
  ButtonsImpl();

  void register_btn1_callback(std::function<void()> cb) { m_btn1_cb = cb; }
  void register_btn2_callback(std::function<void()> cb) { m_btn2_cb = cb; }

 private:
  void button_1_callback(const std_msgs::msg::Bool& msg);
  void button_2_callback(const std_msgs::msg::Bool& msg);
};

std::shared_ptr<ButtonsImpl> get_simulation_buttons();
