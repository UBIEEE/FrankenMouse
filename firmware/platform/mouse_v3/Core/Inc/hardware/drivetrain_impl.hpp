#pragma once

#include <micromouse/drive/kinematics.hpp>
#include <micromouse/drive/pid_controller.hpp>
#include <micromouse/drive/pose.hpp>
#include <micromouse/hardware/drivetrain.hpp>
#include <micromouse/hardware/feedback.hpp>
#include <micromouse/hardware/measurements.hpp>
#include <micromouse/hardware/battery.hpp>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/angular_velocity.h>
#include "hardware/drivetrain_impl.h"
#include "hardware/encoder.hpp"
#include "hardware/imu_impl.hpp"
#include "stm32wbxx_hal.h"

class DrivetrainImpl : public hardware::Drivetrain {
  const hardware::RobotMeasurements& m_measurements = get_robot_measurements();
  const hardware::Battery& m_battery = get_platform_battery();
  hardware::Feedback& m_feedback = get_platform_feedback();

  IMUImpl& m_imu = *reinterpret_cast<IMUImpl*>(&get_platform_imu());

  Encoder m_left_encoder;
  Encoder m_right_encoder;

  MotorData m_drive_data;

  // Control stuff.

  drive::PIDController m_translational_left_pid;
  drive::PIDController m_translational_right_pid;
  drive::PIDController m_angular_pid;

  enum class ControlMode {
    IDLE,
    MANUAL,
    VELOCITY,
  } m_control_mode = ControlMode::IDLE;

  struct {
    units::volt_t left = 0_V;
    units::volt_t right = 0_V;
  } m_raw_speed_data;

  struct {
    drive::ChassisSpeeds target_speeds{};

    units::degrees_per_second_t final_angular = 0_deg_per_s;
    units::volt_t final_right = 0_V;
    units::volt_t final_left = 0_V;
  } m_velocity_control_data;

 public:
  DrivetrainImpl();

  void reset() override;
  void reset_encoders();
  void reset_pid_controllers();

  void periodic() override;
  void publish_periodic_feedback() override;
  void publish_status_feedback() override;
  void stop() override;

  void set_chassis_speeds(const drive::ChassisSpeeds& speeds) override;
  void set_wheel_speeds(const drive::WheelSpeeds& speeds) override {
    set_chassis_speeds(drive::to_chassis_speeds(speeds, m_measurements.track_width));
  }
  void set_motors_manual(float left_percent, float right_percent) override;

 private:
  void update_encoders();
  void update_pid_controllers();

  void set_motors(units::volt_t left, units::volt_t right);
  void set_motors_raw(uint16_t left, GPIO_PinState left_dir, uint16_t right, GPIO_PinState right_dir);

 private:
  friend void ::DrivetrainImpl_UpdatePIDValues(const float*);

  void update_pid_values(const float* translational_pid, const float* angular_pid);

  enum PIDComponent {
    TRANSLATIONAL,
    ANGULAR,
  };

  void update_pid_value(PIDComponent component, drive::PIDController::Term term, float value);
};

DrivetrainImpl& get_mouse_v3_drivetrain();
