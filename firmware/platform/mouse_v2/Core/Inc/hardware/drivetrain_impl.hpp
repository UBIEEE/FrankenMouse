#pragma once

#include <micromouse/drive/kinematics.hpp>
#include <micromouse/drive/pid_controller.hpp>
#include <micromouse/drive/pose.hpp>
#include <micromouse/hardware/drivetrain.hpp>
#include <micromouse/hardware/feedback.hpp>
#include <micromouse/hardware/measurements.hpp>
#include "hardware/drivetrain_impl.h"
#include "hardware/encoder.hpp"
#include "hardware/imu_impl.hpp"
#include "stm32wbxx_hal.h"

class DrivetrainImpl : public hardware::Drivetrain {
  const hardware::RobotMeasurements& m_measurements = get_robot_measurements();
  hardware::Feedback& m_feedback = get_platform_feedback();

  static inline uint8_t m_pwm_counter = 0;
  static inline uint8_t m_pwm_pulse_right = 0;
  static inline uint8_t m_pwm_pulse_left = 0;

  IMUImpl& m_imu = *reinterpret_cast<IMUImpl*>(&get_platform_imu());

  Encoder m_left_encoder;
  Encoder m_right_encoder;

  struct {
    struct {
      Encoder::Data left;
      Encoder::Data right;
    } encoder;

    drive::Pose pose;
  } m_drive_data;

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
    float left = 0.f;
    float right = 0.f;
  } m_raw_speed_data;

  struct {
    drive::ChassisSpeeds target_speeds = {0.f, 0.f};

    float final_angular_dps = 0.f;
    float final_right_speed = 0.f;
    float final_left_speed = 0.f;
  } m_velocity_control_data;

 public:
  DrivetrainImpl();

  void reset() override;
  void reset_encoders();
  void reset_pid_controllers();

  void periodic() override;
  void publish_periodic_feedback() override;
  void publish_extra_feedback() override;
  void stop() override;

  void set_chassis_speeds(const drive::ChassisSpeeds& speeds) override;
  void set_wheel_speeds(const drive::WheelSpeeds& speeds) override {
    set_chassis_speeds(
        drive::to_chassis_speeds(speeds, m_measurements.track_width_mm));
  }

 private:
  void update_encoders();
  void update_pid_controllers();

  void set_motors(float left_percent, float right_percent);
  void set_motors_raw(uint8_t left,
                      GPIO_PinState left_dir,
                      uint8_t right,
                      GPIO_PinState right_dir);

 private:
  friend void ::DrivetrainImpl_UpdatePIDValues(const float*);

  void update_pid_values(const float* translational_pid,
                         const float* angular_pid);

 private:
  friend void ::HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

  void update_pwm();
};

DrivetrainImpl& get_mouse_v2_drivetrain();
