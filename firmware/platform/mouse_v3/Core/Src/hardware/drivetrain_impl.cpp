#include "hardware/drivetrain_impl.hpp"

#include "stm32wbxx_hal_lptim.h"

#include <micromouse/robot/robot.h>
#include <cmath>
#include <units/math.h>
#include "main.h"

using namespace drive;
using namespace hardware;

// Left encoder
extern LPTIM_HandleTypeDef hlptim1;  // main.c
// Right encoder
extern TIM_HandleTypeDef htim2;  // main.c

static constexpr IMU::Axis YAW_AXIS = IMU::Axis::Z;
static constexpr IMU::Axis ROLL_AXIS = IMU::Axis::Y;
static constexpr IMU::Axis PITCH_AXIS = IMU::Axis::X;

static constexpr float TRANSLATIONAL_KP = 0.0004616805171f;
static constexpr float TRANSLATIONAL_KI = 0.0f;
static constexpr float TRANSLATIONAL_KD = 0.0f;
static constexpr float ANGULAR_KP = 0.06f;
static constexpr float ANGULAR_KI = 0.0f;
static constexpr float ANGULAR_KD = 0.0f;

DrivetrainImpl::DrivetrainImpl()
    : m_translational_left_pid(TRANSLATIONAL_KP, TRANSLATIONAL_KI, TRANSLATIONAL_KD, ROBOT_UPDATE_PERIOD_S),
      m_translational_right_pid(TRANSLATIONAL_KP, TRANSLATIONAL_KI, TRANSLATIONAL_KD, ROBOT_UPDATE_PERIOD_S),
      m_angular_pid(ANGULAR_KP, ANGULAR_KI, ANGULAR_KD, ROBOT_UPDATE_PERIOD_S) {}

void DrivetrainImpl::reset() {
  reset_encoders();
  reset_pid_controllers();

  m_control_mode = ControlMode::IDLE;
}

void DrivetrainImpl::reset_encoders() {
  m_left_encoder.reset();
  m_right_encoder.reset();

  m_drive_data = {};
}

void DrivetrainImpl::reset_pid_controllers() {
  m_translational_left_pid.reset();
  m_translational_right_pid.reset();
  m_angular_pid.reset();

  m_velocity_control_data = {};
}

void DrivetrainImpl::periodic() {
  update_encoders();

  m_imu.begin_read();

  switch (m_control_mode) {
    using enum ControlMode;
    case IDLE:
      m_raw_speed_data = {};
      break;
    case MANUAL:
      // Raw speeds already set, so nothing to do here.
      break;
    case VELOCITY:
      // Update PID controllers with velocity control data to produce raw speed
      // data.
      update_pid_controllers();
      break;
  }

  // Set the motors.
  const auto& [left, right] = m_raw_speed_data;
  set_motors(left, right);
}

void DrivetrainImpl::stop() {
  m_raw_speed_data = {};

  m_control_mode = ControlMode::IDLE;
}

void DrivetrainImpl::set_chassis_speeds(const drive::ChassisSpeeds& speeds) {
  m_velocity_control_data.target_speeds = speeds;

  m_control_mode = ControlMode::VELOCITY;
}

void DrivetrainImpl::update_encoders() {
  const uint16_t left_ticks = hlptim1.Instance->CNT;
  const uint16_t right_ticks = htim2.Instance->CNT / 2;

  const EncoderData left_data = m_left_encoder.update(left_ticks);
  const EncoderData right_data = m_right_encoder.update(right_ticks);

  const units::millimeter_t& delta_left = left_data.position - m_drive_data.left_encoder.position;
  const units::millimeter_t& delta_right = right_data.position - m_drive_data.right_encoder.position;

  m_drive_data.left_encoder = left_data;
  m_drive_data.right_encoder = right_data;
}

void DrivetrainImpl::update_pid_controllers() {
  auto& control_data = m_velocity_control_data;

  // Angular PID controller.
  {
    const units::degrees_per_second_t gyro_z = m_imu.get_angular_velocity(YAW_AXIS);

    if (units::math::abs(gyro_z) > 1.5_deg_per_s) {
      control_data.final_angular += units::degrees_per_second_t{
          m_angular_pid.calculate(gyro_z.value(), control_data.target_speeds.angular_velocity.value())};
    }
  }

  ChassisSpeeds chassis_speeds = {
      .linear_velocity = control_data.target_speeds.linear_velocity,
      .angular_velocity = control_data.final_angular,
  };

  // Translational PID controllers.
  {
    auto [v_l, v_r] = to_wheel_speeds(chassis_speeds, m_measurements.track_width);

    control_data.final_right_speed +=
        m_translational_right_pid.calculate(m_drive_data.right_encoder.velocity.value(), v_r.value());

    control_data.final_left_speed +=
        m_translational_left_pid.calculate(-m_drive_data.left_encoder.velocity.value(), v_l.value());
  }

  float final_left = control_data.final_left_speed, final_right = control_data.final_right_speed;

  // Clamp the values.
  if (std::abs(final_left) < 0.1f) {
    final_left = 0.f;
  }
  if (std::abs(final_right) < 0.1f) {
    final_right = 0.f;
  }

  // Set raw speed data.
  m_raw_speed_data = {
      .left = final_left,
      .right = final_right,
  };
}

void DrivetrainImpl::set_motors(float left_percent, float right_percent) {
  left_percent = std::clamp(left_percent, -1.f, 1.f);
  right_percent = std::clamp(right_percent, -1.f, 1.f);

  const uint16_t left_out(std::abs(left_percent) * 7199);
  const uint16_t right_out(std::abs(right_percent) * 7199);

  const bool left_dir = !std::signbit(left_percent);
  const bool right_dir = std::signbit(right_percent);

  const GPIO_PinState left_dir_pin = static_cast<GPIO_PinState>(left_dir);
  const GPIO_PinState right_dir_pin = static_cast<GPIO_PinState>(right_dir);

  set_motors_raw(left_out, left_dir_pin, right_out, right_dir_pin);
}

void DrivetrainImpl::set_motors_raw(uint16_t left,
                                    GPIO_PinState left_dir,
                                    uint16_t right,
                                    GPIO_PinState right_dir) {
  HAL_GPIO_WritePin(MOTOR_LEFT_DIR_GPIO_Port, MOTOR_LEFT_DIR_Pin, left_dir);
  HAL_GPIO_WritePin(MOTOR_RIGHT_DIR_GPIO_Port, MOTOR_RIGHT_DIR_Pin, right_dir);

  // Duty-cycle = CCR/ARR
  TIM1->CCR1 = left;
  TIM1->CCR2 = right;
}

void DrivetrainImpl::update_pid_values(const float* translational, const float* angular) {
  using enum drive::PIDController::Term;
  for (auto term : {PROPORTIONAL, INTEGRAL, DERIVATIVE}) {
    update_pid_value(PIDComponent::TRANSLATIONAL, term, translational[term]);
    update_pid_value(PIDComponent::ANGULAR, term, angular[term]);
  }

  m_translational_left_pid.reset();
  m_translational_right_pid.reset();
  m_angular_pid.reset();
}

void DrivetrainImpl::update_pid_value(PIDComponent component, drive::PIDController::Term term, float value) {
  if (value < 0.f)
    return;

  switch (component) {
    case PIDComponent::TRANSLATIONAL:
      m_translational_left_pid.set_term(term, value);
      m_translational_right_pid.set_term(term, value);
      break;
    case PIDComponent::ANGULAR:
      m_angular_pid.set_term(term, value);
      break;
  }
}

void DrivetrainImpl::publish_periodic_feedback() {
  using namespace feedback;

  m_feedback.publish<TopicSend::DRIVE_MOTOR_DATA>(m_drive_data);
}

void DrivetrainImpl::publish_status_feedback() {}

void DrivetrainImpl_UpdatePIDValues(const float* pid) {
  const float* translational = pid;
  const float* angular = pid + 3;

  DrivetrainImpl* d = reinterpret_cast<DrivetrainImpl*>(&get_platform_drivetrain());

  d->update_pid_values(translational, angular);
}

DrivetrainImpl& get_mouse_v3_drivetrain() {
  static DrivetrainImpl s_drivetrain;
  return s_drivetrain;
}

hardware::Drivetrain& get_platform_drivetrain() {
  return get_mouse_v3_drivetrain();
}
