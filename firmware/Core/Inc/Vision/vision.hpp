#pragma once

#include "Basic/subsystem.hpp"

#include <cstdint>
#include <cstring>

class Vision : public Subsystem {
  static constexpr struct {
    float distance_mm;
    float intensity_reading;
  } KNOWN_READING = {
      .distance_mm       = 120.f,
      .intensity_reading = 0.047852f,
  };

  static constexpr float CALIBRATION_MID_DISTANCE = 0.f;
  static constexpr float CALIBRATION_FAR_DISTANCE = 0.f;

private:
  bool m_enabled = false;

public:
  enum Sensor : uint8_t {
    FAR_RIGHT = 0, // Channel 7
    MID_RIGHT,     // Channel 8
    MID_LEFT,      // Channel 9
    FAR_LEFT,      // Channel 10
  };

private:
  float m_calibration[4] = {0};

  uint16_t m_raw_readings[4]; // From ADC DMA.
  float m_readings[4];
  float m_corrected_readings[4]; // After calibration.
  float m_distances[4];
  float m_wall_distances[4]; // After angle adjustment.

  bool m_is_calibrated = false;

  Sensor m_sensor = Sensor::FAR_RIGHT;

  enum class State {
    IDLE,
    WAITING,
    READING,
  } m_state = State::IDLE;

  volatile bool m_adc_ready = false;

public:
  void process() override;
  void send_feedback() override;

  void set_enabled(bool enabled) { m_enabled = enabled; }
  bool is_enabled() const { return m_enabled; }

  // Calibrate the sensors by measuring readings when no object is present.
  void calibrate();

  void reset_calibration() {
    std::memset(m_calibration, 0, sizeof(m_calibration));
    m_is_calibrated = false;
  }

  bool is_calibrated() const { return m_is_calibrated; }

  uint16_t get_raw_reading(Sensor sensor) const { return m_readings[sensor]; }

  // Distance to object directly in front of the respective sensor.
  float get_distance_mm(Sensor sensor) const {
    return m_distances[sensor];
  }

  // Distance from the sensor to the wall it's angled towards.
  float get_wall_distance_mm(Sensor sensor) const {
    return m_wall_distances[sensor];
  }

private:
  void set_emitter(Sensor sensor, GPIO_PinState state);

  void handle_raw_sensor_reading();

  // Compensate for the inverse square law and calculate the distance to an
  // object based on the intensity of light received back from the
  // phototransistor.
  //
  // Intensity should range between 0 and 1 (1 being the maximum intensity
  // capable of being read).
  static float calculate_distance_mm(const float& intensity_reading);

  // Adjusts a distance reading to account for the angle of the sensor.
  static float calculate_distance_to_wall_mm(const float& distance_mm,
                                             Sensor sensor);

private:
  friend void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*);

  void read_complete_handler();
};
