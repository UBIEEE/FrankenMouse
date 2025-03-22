#pragma once

#include <micromouse/hardware/drivetrain.hpp>
#include <micromouse/hardware/measurements.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class DrivetrainImpl : public hardware::Drivetrain, public rclcpp::Node {
  const hardware::RobotMeasurements& m_measurements = get_robot_measurements();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_twist_pub;

 public:
  DrivetrainImpl();

  void stop() override;

  void set_chassis_speeds(const drive::ChassisSpeeds& speeds) override;
  void set_wheel_speeds(const drive::WheelSpeeds& speeds) override {
    set_chassis_speeds(
        drive::to_chassis_speeds(speeds, m_measurements.track_width_mm));
  }
};

std::shared_ptr<DrivetrainImpl> get_simulation_drivetrain();
