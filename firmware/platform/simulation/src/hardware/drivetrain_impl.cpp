#include <simulation/hardware/drivetrain_impl.hpp>

#include <micromouse/drive/kinematics.hpp>
#include <numbers>

using namespace std::placeholders;

DrivetrainImpl::DrivetrainImpl() : Node("micromouse_drivetrain") {
  m_twist_pub = this->create_publisher<geometry_msgs::msg::Twist>(
      "/simulation/drive/cmd_vel", 10);
}

void DrivetrainImpl::stop() {
  set_chassis_speeds({0.f, 0.f});
}

void DrivetrainImpl::set_chassis_speeds(const drive::ChassisSpeeds& speeds) {
  geometry_msgs::msg::Twist twist;
  twist.angular.z = speeds.angular_velocity_dps;
  twist.linear.x = speeds.linear_velocity_mmps / 1000.0;

  m_twist_pub->publish(twist);
}

std::shared_ptr<DrivetrainImpl> get_simulation_drivetrain() {
  static auto drivetrain = std::make_shared<DrivetrainImpl>();
  return drivetrain;
}

hardware::Drivetrain& get_platform_drivetrain() {
  return *get_simulation_drivetrain();
}
