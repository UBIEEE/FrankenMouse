#pragma once

#include <micromouse/hardware/feedback.hpp>
#include <micromouse/hardware/ir_sensors.hpp>
#include <micromouse/subsystem.hpp>

namespace vision {

class Vision : public Subsystem {
  hardware::Feedback& m_feedback = get_platform_feedback();
  hardware::IRSensors& m_ir_sensors = get_platform_ir_sensors();

  bool m_was_left_wall = false;
  bool m_was_right_wall = false;

 public:
  Vision();

  void periodic() override;
  void publish_periodic_feedback() override;

  void reset_calibration();
  void calibrate();

  bool left_wall();
  bool right_wall();
  bool front_wall();

  bool did_left_wall_just_disappear();
  bool did_right_wall_just_disappear();
};

}  // namespace vision
