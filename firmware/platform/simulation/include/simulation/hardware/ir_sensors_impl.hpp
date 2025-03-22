#pragma once

#include <micromouse/hardware/ir_sensors.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class IRSensorsImpl : public hardware::IRSensors, public rclcpp::Node {
  float m_raw_readings[4] = {0};
  float m_distances_mm[4];

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      m_ir_readings_sub;

 public:
  IRSensorsImpl();

  const float* get_raw_readings() const override { return m_raw_readings; }
  const float* get_distances_mm() const override { return m_distances_mm; }

 private:
  void ir_readings_callback(const std_msgs::msg::Float32MultiArray& msg);
};

std::shared_ptr<IRSensorsImpl> get_simulation_ir_sensors();
