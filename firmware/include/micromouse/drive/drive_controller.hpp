#pragma once

#include <micromouse/drive/pid_controller.hpp>
#include <micromouse/drive/speed_config.hpp>
#include <micromouse/drive/trapezoid_profile.hpp>
#include <micromouse/hardware/drivetrain.hpp>
#include <micromouse/hardware/ir_sensors.hpp>
#include <micromouse/hardware/measurements.hpp>
#include <micromouse/hardware/timer.hpp>
#include <micromouse/subsystem.hpp>
#include <micromouse/vision/vision.hpp>
#include <micromouse/robot.h>

#include <cmath>
#include <functional>
#include <optional>
#include <queue>

namespace drive {

class DriveController : public Subsystem {
  hardware::RobotMeasurements& m_measurements = get_robot_measurements();
  hardware::Drivetrain& m_drivetrain = get_platform_drivetrain();
  vision::Vision& m_vision;
  hardware::IRSensors& m_ir_sensors = get_platform_ir_sensors();

  TrapezoidProfile m_linear_profile;
  TrapezoidProfile m_angular_profile;

  std::unique_ptr<hardware::Timer> m_linear_timer = make_platform_timer();
  std::unique_ptr<hardware::Timer> m_angular_timer = make_platform_timer();

  SpeedConstraints m_speeds;

  drive::PIDController m_vision_align_pid {
    0.05f, 0.f, 0.f, ROBOT_UPDATE_PERIOD_S,
  };

  enum class MotionState {
    NONE,
    IDLE,
    MOTION,
  } m_motion_state = MotionState::NONE;

 public:
  enum class TurnAngle : int16_t {
    CW_180 = -180,
    CW_90 = -90,
    CW_45 = -45,
    CCW_45 = 45,
    CCW_90 = 90,
    CCW_180 = 180,
  };

 private:
  struct Motion {
    enum class Type {
      FORWARD,
      TURN,
    } type;

    union {
      // Forward
      struct {
        float distance;
        bool end_high = true;
      } forward;

      // Turn
      struct {
        TurnAngle angle;
        float arc_distance_mm;
      } turn;
    };

    std::function<void()> completion_func = nullptr;
  };

  std::queue<Motion> m_motions;
  std::optional<Motion> m_current_motion;

 public:
  using CompletionCallback = std::function<void()>;

 public:
  DriveController(vision::Vision& vision, const SpeedConstraints& speeds)
      : m_vision(vision), m_speeds(speeds) {}

  void reset();
  void set_speeds(const SpeedConstraints& speeds) { m_speeds = speeds; }
  const SpeedConstraints& get_speeds() const { return m_speeds; }

  void periodic() override;
  void publish_periodic_feedback() override;
  void publish_extra_feedback() override;

  bool is_done() const {
    return m_motions.empty() && ((m_motion_state == MotionState::NONE) ||
                                 (m_motion_state == MotionState::IDLE));
  }

  // Adds a forward motion to the motion queue.
  void enqueue_forward(float distance_mm,
                       bool end_high = true,
                       CompletionCallback completion_func = nullptr);

  // Adds a turning motion to the motion queueue.
  void enqueue_turn(float leadup_distance_mm,
                    TurnAngle angle,
                    float turn_radius_mm,
                    float followup_distance_mm,
                    CompletionCallback completion_func = nullptr);

  void enqueue_turn(TurnAngle angle,
                    float turn_radius_mm,
                    CompletionCallback completion_func = nullptr) {
    enqueue_turn(0.f, angle, turn_radius_mm, 0.f, completion_func);
  }

  void enqueue_turn(TurnAngle angle,
                    CompletionCallback completion_func = nullptr) {
    enqueue_turn(0.f, angle, 0.f, 0.f, completion_func);
  }

  // Stops and clears queued motions.
  void stop() {
    m_drivetrain.stop();
    m_motion_state = MotionState::NONE;
    m_motions = {};
    reset();
  }

  void make_idle() {
    m_motion_state = MotionState::IDLE;
    m_motions = {};
    m_current_motion = std::nullopt;
  }

 private:
  void start_next_motion();
  void start_arc(Motion& motion);

  void process_motion(bool linear_done, bool angular_done);

  void process_forward(bool linear_done, bool angular_done);
  void process_turn(bool linear_done, bool angular_done);

 private:
  void config_linear(float distance_mm, bool end_high = true);
  void config_linear(float distance_mm, float final_velocity_mmps);
  void config_linear(float distance_mm,
                     float final_velocity_mmps,
                     float max_velocity_mmps);

  void config_angular(float angle_deg);
  void config_angular(float angle_deg,
                      float final_velocity_dps,
                      float max_velocity_dps,
                      float acceleration_dps2);
};

}  // namespace drive
