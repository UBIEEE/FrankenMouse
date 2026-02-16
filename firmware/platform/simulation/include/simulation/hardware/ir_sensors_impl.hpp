#pragma once

#include <micromouse/hardware/ir_sensors.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class IRSensorsImpl : public hardware::IRSensors, public rclcpp::Node {
  std::array<float, 4> m_raw_readings = {0};
  std::array<units::millimeter_t, 4> m_distances;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_ir_readings_sub;

 public:
  IRSensorsImpl();

  const std::array<float, 4>& get_raw_readings() const override { return m_raw_readings; }
  const std::array<units::millimeter_t, 4>& get_distances() const override { return m_distances; }

 private:
  void ir_readings_callback(const std_msgs::msg::Float32MultiArray& msg);
};

std::shared_ptr<IRSensorsImpl> get_simulation_ir_sensors();
