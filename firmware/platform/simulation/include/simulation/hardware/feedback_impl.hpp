#pragma once

#include <micromouse/hardware/feedback.hpp>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

class FeedbackImpl : public hardware::Feedback, public rclcpp::Node {
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr m_main_task_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr m_main_command_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr m_main_song_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      m_drive_pid_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      m_drive_chassis_speeds_sub;

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_main_task_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr m_main_error_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_main_song_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr m_main_status_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      m_vision_raw_readings_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      m_vision_distances_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      m_drive_motor_data_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      m_drive_imu_data_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      m_drive_pid_data_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      m_drive_chassis_speeds_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr m_maze_cell_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_maze_coordinates_pub;

 public:
  FeedbackImpl();

  void publish_topic(FeedbackTopicSend topic, uint8_t* data) override;
};

std::shared_ptr<FeedbackImpl> get_simulation_feedback();
