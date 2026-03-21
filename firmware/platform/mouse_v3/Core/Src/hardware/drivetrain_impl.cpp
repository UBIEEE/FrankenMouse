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

static constexpr float TRANSLATIONAL_KP = 0.000500f;
static constexpr float TRANSLATIONAL_KI = 0.000000f;
static constexpr float TRANSLATIONAL_KD = 0.000050f;
static constexpr float ANGULAR_KP = 0.050000f;
static constexpr float ANGULAR_KI = 0.000000f;
static constexpr float ANGULAR_KD = 0.000000f;

// Minimum output to overcome static friction.
static constexpr units::volt_t MOTOR_FF = 0.8_V;

static constexpr units::volt_t MOTOR_MAX_OUTPUT = 6_V;

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

void DrivetrainImpl::set_motors_manual(float left_percent, float right_percent) {
  m_raw_speed_data.left = left_percent * MOTOR_MAX_OUTPUT;
  m_raw_speed_data.right = right_percent * MOTOR_MAX_OUTPUT;

  m_control_mode = ControlMode::MANUAL;
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

  const units::degrees_per_second_t gyro_z = m_imu.get_angular_velocity(YAW_AXIS);
  control_data.final_angular += units::degrees_per_second_t{
      m_angular_pid.calculate(gyro_z.value(), control_data.target_speeds.angular_velocity.value())};

  ChassisSpeeds chassis_speeds = {
      .linear_velocity = control_data.target_speeds.linear_velocity,
      .angular_velocity = control_data.final_angular,
  };

  // Translational PID controllers.
  auto [v_l, v_r] = to_wheel_speeds(chassis_speeds, m_measurements.track_width);

  control_data.final_right += units::volt_t{
      m_translational_right_pid.calculate(m_drive_data.right_encoder.velocity.value(), v_r.value())};
  control_data.final_left += units::volt_t{
      m_translational_left_pid.calculate(-m_drive_data.left_encoder.velocity.value(), v_l.value())};

  units::volt_t final_left = control_data.final_left, final_right = control_data.final_right;

#if 0
  // Feedforward
  if (units::math::abs(final_left) > 0.025_V) {
    final_left += units::math::copysign(MOTOR_FF, final_left);
  }
  if (units::math::abs(final_right) > 0.025_V) {
    final_right += units::math::copysign(MOTOR_FF, final_right);
  }
#endif

  // Set output speeds.
  m_raw_speed_data = {
      .left = final_left,
      .right = final_right,
  };
}

void DrivetrainImpl::set_motors(units::volt_t left_voltage, units::volt_t right_voltage) {
  /*
   * The motors are supplied power directly from the battery through the motor driver, so we need to monitor
   * the battery voltage and adjust the duty cycle accordingly to send the correct voltage to the motors.
   */

  const units::volt_t battery_voltage = m_battery.get_voltage();

  units::volt_t max_motor_voltage = std::min(MOTOR_MAX_OUTPUT, battery_voltage);

  if (m_battery.is_usb()) {
    // Limit current draw over USB by not driving too fast
    max_motor_voltage = std::min(3_V, battery_voltage);
  }

  left_voltage = std::clamp(left_voltage, -max_motor_voltage, +max_motor_voltage);
  right_voltage = std::clamp(right_voltage, -max_motor_voltage, +max_motor_voltage);

  float left_percent = left_voltage / battery_voltage;
  float right_percent = right_voltage / battery_voltage;

  // Set duty cycle and direction

  const uint16_t left_out = static_cast<uint16_t>(std::abs(left_percent) * 7199.f);
  const uint16_t right_out = static_cast<uint16_t>(std::abs(right_percent) * 7199.f);

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
    float translational_value = translational[term];
    float angular_value = angular[term];
    update_pid_value(PIDComponent::TRANSLATIONAL, term, translational_value);
    update_pid_value(PIDComponent::ANGULAR, term, angular_value);
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

void DrivetrainImpl::publish_status_feedback() {
  using namespace feedback;

  float pid_values[6] = {
      m_translational_left_pid.kp(),
      m_translational_left_pid.ki(),
      m_translational_left_pid.kd(),
      m_angular_pid.kp(),
      m_angular_pid.ki(),
      m_angular_pid.kd(),
  };
  m_feedback.publish<TopicSend::DRIVE_PID>(pid_values);
}

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
