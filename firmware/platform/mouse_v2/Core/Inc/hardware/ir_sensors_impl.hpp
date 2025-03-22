#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <micromouse/hardware/ir_sensors.hpp>
#include "stm32wbxx_hal.h"

class IRSensorsImpl : public hardware::IRSensors {
  bool m_enabled = false;

  uint16_t m_raw_readings[4] = {0};

  float m_readings[4] = {0};
  float m_distances_mm[4];

  enum Sensor : uint8_t {
    FAR_RIGHT,  // Channel 7
    MID_RIGHT,  // Channel 8
    MID_LEFT,   // Channel 9
    FAR_LEFT,   // Channel 10
  } m_sensor = Sensor::FAR_RIGHT;

  enum class State {
    IDLE,
    WAITING,
    READING,
  } m_state = State::IDLE;

  volatile bool m_adc_ready = false;

 public:
  IRSensorsImpl() {
    std::fill(m_distances_mm, m_distances_mm + 4,
              std::numeric_limits<float>::infinity());
  }

  void periodic() override;

  void set_enabled(bool enabled) override { m_enabled = enabled; }
  bool is_enabled() const override { return m_enabled; }

  const float* get_raw_readings() const override { return m_readings; }
  const float* get_distances_mm() const override { return m_distances_mm; }

 private:
  void set_emitter(Sensor sensor, GPIO_PinState state);

  void handle_raw_sensor_reading();

  static float calculate_distance_mm(const float& intensity_reading);

 private:
  friend void ::HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

  void read_complete_handler();
};

IRSensorsImpl& get_mouse_v2_ir_sensors();
