#pragma once

#include "Basic/subsystem.hpp"
#include "Drive/drive.hpp"
#include "Drive/speed_config.hpp"
#include "DriveTools/pid_controller.hpp"
#include "DriveTools/trapezoid_profile.hpp"
#include "Utilities/timer.hpp"
#include "Vision/vision.hpp"
#include <optional>

namespace drive {

class DriveController : public Subsystem {
  Drive& m_drive;
  Vision& m_vision;

  PIDController m_vision_pid;

  TrapezoidProfile m_linear_profile;
  TrapezoidProfile m_angular_profile;

  Timer m_linear_timer;
  Timer m_angular_timer;

  const SpeedConstraints* m_speeds;

  enum class MotionState {
    NONE,
    IDLE,
    MOTION,
  } m_motion_state = MotionState::NONE;

public:
  enum class TurnAngle : int16_t {
    CW_180  = 180,
    CW_90   = 90,
    CW_45   = 45,
    CCW_45  = -45,
    CCW_90  = -90,
    CCW_180 = -180,
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
        bool end_high;
      } forward;

      // Turn
      struct {
        TurnAngle angle;
        float arc_distance_mm;
        // float turn_arc_max_velocity;
      } turn;
    };
  };

  std::queue<Motion> m_motions;
  std::optional<Motion> m_current_motion;

public:
  DriveController(Drive& drive, Vision& vision);

  void reset();
  void set_speeds(const SpeedConstraints& speeds) { m_speeds = &speeds; }

  void process() override;

  bool is_done() const {
    return m_motions.empty() && ((m_motion_state == MotionState::NONE) ||
                                 (m_motion_state == MotionState::IDLE));
  }

  // Adds a forward motion to the motion queue.
  void enqueue_forward(float distance_mm, bool end_high = true);

  // Adds a turning motion to the motion queueue.
  void enqueue_turn(float leadup_distance_mm, TurnAngle angle,
                    float followup_distance_mm);

  void enqueue_turn(TurnAngle angle, float total_turn_radius_mm) {
    total_turn_radius_mm = std::max(total_turn_radius_mm, m_speeds->turn_radius_mm);
    float leadup_followup_mm = total_turn_radius_mm - m_speeds->turn_radius_mm;
    enqueue_turn(leadup_followup_mm, angle, leadup_followup_mm);
  }

  void enqueue_turn(TurnAngle angle) { enqueue_turn(0.f, angle, 0.f); }

  // Stops and clears queued motions.
  void stop() {
    m_motion_state = MotionState::NONE;
    m_motions      = {};
    reset();
  }

  void make_idle() {
    m_motion_state   = MotionState::IDLE;
    m_motions        = {};
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
  void config_linear(float distance_mm, float final_velocity_mmps,
                     float max_velocity_mmps);

  void config_angular(float angle_deg);
  void config_angular(float angle_deg, float final_velocity_dps,
                      float max_velocity_dps, float acceleration_dps2);
};

} // namespace drive

using drive::DriveController;
