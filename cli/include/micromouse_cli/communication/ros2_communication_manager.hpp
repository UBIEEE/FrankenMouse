#pragma once

#ifndef WITH_ROS2
#error "ROS2 not enabled, so don't include this file!"
#endif

#include <micromouse_cli/communication/communication_manager.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

class ROS2CommunicationManager final : public CommunicationManager, public rclcpp::Node {
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr m_main_task_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_main_command_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_main_song_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_drive_pid_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_drive_chassis_speeds_pub;

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr m_main_task_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr m_main_error_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr m_main_song_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr m_main_status_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_main_battery_voltage_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_vision_raw_readings_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_vision_distances_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_drive_motor_data_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_drive_imu_data_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_drive_pid_data_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_drive_chassis_speeds_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr m_maze_cell_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr m_maze_coordinates_sub;

 public:
  ROS2CommunicationManager();
  ~ROS2CommunicationManager();

  static const char* name() { return "ros2"; }

  bool is_initialized() const override { return true; }
  bool is_connected() const override { return true; }

  int peripheral_rssi() const override { return 0; }

 private:
  void configure_publishers();
  void configure_subscribers();

 private:
  void write_topic(FeedbackTopicWrite topic_id, const uint8_t* data, size_t size) override;
};
