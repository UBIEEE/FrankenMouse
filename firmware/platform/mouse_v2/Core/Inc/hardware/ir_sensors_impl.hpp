#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <micromouse/hardware/ir_sensors.hpp>
#include "stm32wbxx_hal.h"

class IRSensorsImpl : public hardware::IRSensors {
  bool m_enabled = false;

  uint16_t m_raw_readings[4] = {0};

  std::array<float, 4> m_readings = {0};
  std::array<units::millimeter_t, 4> m_distances;

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
    std::fill(m_distances.begin(), m_distances.end(), units::millimeter_t{std::numeric_limits<float>::infinity()});
  }

  void periodic() override;

  void set_enabled(bool enabled) override { m_enabled = enabled; }
  bool is_enabled() const override { return m_enabled; }

  const std::array<float, 4>& get_raw_readings() const override { return m_readings; }
  const std::array<units::millimeter_t, 4>& get_distances() const override { return m_distances; }

 private:
  void set_emitter(Sensor sensor, GPIO_PinState state);

  void handle_raw_sensor_reading();

  static units::millimeter_t calculate_distance(const float& intensity_reading);

 private:
  friend void ::HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

  void read_complete_handler();
};

IRSensorsImpl& get_mouse_v2_ir_sensors();
