#include "hardware/ir_sensors_impl.hpp"

#include "main.h"

#include <cmath>

extern ADC_HandleTypeDef hadc1;  // main.c

static constexpr GPIO_TypeDef* EMIT_PORTS[] = {
    IR_FAR_RIGHT_EMIT_GPIO_Port,
    IR_MID_RIGHT_EMIT_GPIO_Port,
    IR_MID_LEFT_EMIT_GPIO_Port,
    IR_FAR_LEFT_EMIT_GPIO_Port,
};

static constexpr uint16_t EMIT_PINS[] = {
    IR_FAR_RIGHT_EMIT_Pin,
    IR_MID_RIGHT_EMIT_Pin,
    IR_MID_LEFT_EMIT_Pin,
    IR_FAR_LEFT_EMIT_Pin,
};

void IRSensorsImpl::periodic() {
  if (!m_enabled) {
    if (m_state != State::IDLE) {
      // Turn off the emitter.
      set_emitter(m_sensor, GPIO_PIN_RESET);
      m_state = State::IDLE;
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
      HAL_ADC_Start_DMA(&hadc1,
                        reinterpret_cast<uint32_t*>(&m_raw_readings), 4);
      m_state = State::READING;
      break;
  }
}

void IRSensorsImpl::set_emitter(Sensor sensor, GPIO_PinState state) {
  HAL_GPIO_WritePin(EMIT_PORTS[sensor], EMIT_PINS[sensor], state);
}

void IRSensorsImpl::handle_raw_sensor_reading() {
  m_readings[m_sensor] = m_raw_readings[m_sensor] / 1024.f; // 10-Bit reading.

  // TODO: Correct for any light bleed through the sensor cover.

  // Normalize the reading to be linear (undo the inverse square law).
  m_distances_mm[m_sensor] = calculate_distance_mm(m_readings[m_sensor]);

}

float IRSensorsImpl::calculate_distance_mm(const float& R) {
  if (R < 0.001f) return std::numeric_limits<float>::infinity();

  // Measurements:
  // 120mm: 0.047852
  // 160mm: 0.027344
  const float known_distance_mm = 120.f;
  const float known_intensity   = 0.047852f;

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

  const float K = known_intensity * 4 * (known_distance_mm * known_distance_mm);

  const float d = std::sqrt(K / (4.f * R));

  return d;
}

void IRSensorsImpl::read_complete_handler() {
  m_adc_ready = true;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  assert_param(hadc->Instance == ADC1);
  UNUSED(hadc);

  get_mouse_v2_ir_sensors().read_complete_handler();
}

IRSensorsImpl& get_mouse_v2_ir_sensors() {
  static IRSensorsImpl ir_sensors;
  return ir_sensors;
}

hardware::IRSensors& get_platform_ir_sensors() {
  return get_mouse_v2_ir_sensors();
}