#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <micromouse/hardware/ir_sensors.hpp>
#include <micromouse/hardware/battery.hpp>
#include <micromouse/hardware/timer.hpp>
#include <micromouse/hardware/feedback.hpp>
#include "stm32wbxx_hal.h"
#include <units/voltage.h>

class IRSensorsImpl : public hardware::IRSensors {
  hardware::Feedback& m_feedback = get_platform_feedback();

  bool m_enabled = false;

  uint16_t m_raw_readings[5] = {0};

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

  units::volt_t m_battery_voltage = FULLY_CHARGED_VOLTAGE;

  static constexpr units::volt_t FULLY_CHARGED_VOLTAGE = 8.4_V;
  static constexpr units::volt_t MEDIUM_VOLTAGE = 7.8_V;
  static constexpr units::volt_t LOW_VOLTAGE = 7.0_V;

  std::unique_ptr<hardware::Timer> m_battery_read_timer = make_platform_timer();

 public:
  IRSensorsImpl();

  void periodic() override;

  void set_enabled(bool enabled) override { m_enabled = enabled; }
  bool is_enabled() const override { return m_enabled; }

  const std::array<float, 4>& get_raw_readings() const override { return m_readings; }
  const std::array<units::millimeter_t, 4>& get_distances() const override { return m_distances; }

  bool is_battery() const;
  float get_charge_percentage() const;
  units::volt_t get_voltage() const;

 private:
  void set_emitter(Sensor sensor, GPIO_PinState state);

  void handle_raw_sensor_reading();
  void handle_raw_battery_reading();

  static units::millimeter_t calculate_distance(const float& intensity_reading);

 private:
  friend void ::HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

  void read_complete_handler();
};

class BatteryImpl : public hardware::Battery {
 public:
  bool is_battery() const override;
  float get_charge_percentage() const override;
  units::volt_t get_voltage() const override;
};

// Two separate classes so that single class doesn't have process() called twice.

IRSensorsImpl& get_mouse_v3_ir_sensors();
BatteryImpl& get_mouse_v3_battery();
