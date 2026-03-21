#include "hardware/ir_sensors_and_battery_impl.hpp"

#include "main.h"
#include "micromouse/feedback/feedback_topic.hpp"
#include "micromouse/hardware/battery.hpp"
#include <units/math.h>
#include <cmath>
#include <algorithm>

extern ADC_HandleTypeDef hadc1;  // main.c

static GPIO_TypeDef* EMIT_PORTS[] = {
    IR_FAR_RIGHT_EMIT_GPIO_Port,
    IR_MID_RIGHT_EMIT_GPIO_Port,
    IR_MID_LEFT_EMIT_GPIO_Port,
    IR_FAR_LEFT_EMIT_GPIO_Port,
};

static const uint16_t EMIT_PINS[] = {
    IR_FAR_RIGHT_EMIT_Pin,
    IR_MID_RIGHT_EMIT_Pin,
    IR_MID_LEFT_EMIT_Pin,
    IR_FAR_LEFT_EMIT_Pin,
};

IRSensorsImpl::IRSensorsImpl() {
  std::fill(m_distances.begin(), m_distances.end(),
            units::millimeter_t{std::numeric_limits<float>::infinity()});
  m_battery_read_timer->reset();
  m_battery_read_timer->start();
}

void IRSensorsImpl::periodic() {
  if (!m_enabled) {
    if (m_state != State::IDLE) {
      // Turn off the emitter.
      set_emitter(m_sensor, GPIO_PIN_RESET);
      m_state = State::IDLE;
    }

    // Keep reading the battery voltage
    if (m_battery_read_timer->get() >= 20_s) {
      m_battery_read_timer->reset();
      HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(&m_raw_readings), 5);
    }

    if (m_adc_ready) {
      m_adc_ready = false;
      handle_raw_battery_reading();
      m_battery_read_timer->reset();
      m_battery_read_timer->start();
    }
    return;
  }

  switch (m_state) {
    case State::READING:
      if (!m_adc_ready)
        return;
      m_adc_ready = false;
      m_state = State::IDLE;

      // Turn off the emitter.
      set_emitter(m_sensor, GPIO_PIN_RESET);

      handle_raw_sensor_reading();

      if (m_battery_read_timer->get() >= 20_s) {
        handle_raw_battery_reading();
        m_battery_read_timer->reset();
        m_battery_read_timer->start();
      }

      // Next sensor.
      m_sensor = static_cast<Sensor>((m_sensor + 1) % 4);
      break;

    case State::IDLE:
      // Turn on the emitter.
      set_emitter(m_sensor, GPIO_PIN_SET);

      m_state = State::WAITING;  // Wait one tick for the emitter to turn on.
      break;

    case State::WAITING:
      // Start DMA read.
      HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(&m_raw_readings), 5);
      m_state = State::READING;
      break;
  }
}

void IRSensorsImpl::publish_status_feedback() {
  m_feedback.publish<feedback::TopicSend::MAIN_BATTERY_VOLTAGE>(m_battery_voltage);
}

void IRSensorsImpl::set_emitter(Sensor sensor, GPIO_PinState state) {
  HAL_GPIO_WritePin(EMIT_PORTS[sensor], EMIT_PINS[sensor], state);
}

void IRSensorsImpl::handle_raw_sensor_reading() {
  m_readings[m_sensor] = m_raw_readings[m_sensor] / 1024.f;  // 10-Bit reading.

  // TODO: Correct for any light bleed through the sensor cover.

  // Normalize the reading to be linear (undo the inverse square law).
  m_distances[m_sensor] = calculate_distance(m_readings[m_sensor]);
}

units::millimeter_t IRSensorsImpl::calculate_distance(const float& R) {
  if (R < 0.001f)
    return units::millimeter_t{std::numeric_limits<float>::infinity()};

  // Measurements:
  // 120mm: 0.047852
  // 160mm: 0.027344
  // const units::millimeter_t known_distance = 120_mm;
  // const float known_intensity = 0.047852f;

  const units::millimeter_t known_distance = 84_mm;
  const float known_intensity = 0.068359f;

  // The light intensity emitted gets weaker as the distance increases (by the
  // inverse square law). The distance is actually double, since it needs to
  // travel away from the robot and back after being reflected.
  //
  // R = 1 / (2d)^2
  //
  // To solve for distance, use a known intensity and distance to solve for K.
  //
  // K = R * (2d)^2
  //
  // Solve for d.
  //
  // d = sqrt(K / (4 * R))

  const auto K = known_intensity * 4 * (known_distance * known_distance);

  const units::millimeter_t d = units::math::sqrt(K / (4.f * R));

  return d;
}

void IRSensorsImpl::handle_raw_battery_reading() {
  // 10-Bit reading, scaled to ADC reference voltage.
  const units::volt_t raw_voltage = m_raw_readings[4] / 1024.f * 3.3_V;

  /**
   * Undo the voltage divider to get the actual voltage powering the robot.
   *
   * R1 = 10k, R2 = 5.1k, so the battery voltage should be raw_voltage / (R2 / (R1 + R2))
   * For some reason this isn't quite right, so just linear interpolate based on two measurements for now.
   */

  units::volt_t x1 = 1.40507805_V, y1 = 5.212_V;
  units::volt_t x2 = 2.11083984_V, y2 = 7.710_V;

  m_battery_voltage = ((y2 - y1) / (x2 - x1)) * (raw_voltage - x1) + y1;

  // Feedback
  m_feedback.publish<feedback::TopicSend::MAIN_BATTERY_VOLTAGE>(m_battery_voltage);
}

bool IRSensorsImpl::is_battery() const {
  return m_battery_voltage > 6_V;
}

float IRSensorsImpl::get_charge_percentage() const {
  float percentage = (m_battery_voltage - LOW_VOLTAGE) / (FULLY_CHARGED_VOLTAGE - LOW_VOLTAGE);
  return std::clamp(percentage, 0.f, 1.f);
}

units::volt_t IRSensorsImpl::get_voltage() const {
  return m_battery_voltage;
}

void IRSensorsImpl::read_complete_handler() {
  m_adc_ready = true;
}

bool BatteryImpl::is_battery() const {
  return get_mouse_v3_ir_sensors().is_battery();
}

float BatteryImpl::get_charge_percentage() const {
  return get_mouse_v3_ir_sensors().get_charge_percentage();
}

units::volt_t BatteryImpl::get_voltage() const {
  return get_mouse_v3_ir_sensors().get_voltage();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc) {
  assert_param(hadc->Instance == ADC1);
  UNUSED(hadc);

  get_mouse_v3_ir_sensors().read_complete_handler();
}

IRSensorsImpl& get_mouse_v3_ir_sensors() {
  static IRSensorsImpl ir_sensors;
  return ir_sensors;
}

BatteryImpl& get_mouse_v3_battery() {
  static BatteryImpl battery;
  return battery;
}

hardware::IRSensors& get_platform_ir_sensors() {
  return get_mouse_v3_ir_sensors();
}

hardware::Battery& get_platform_battery() {
  return get_mouse_v3_battery();
}
