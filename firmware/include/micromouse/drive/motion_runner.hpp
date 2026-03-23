#pragma once

#include <micromouse/drive/speed_config.hpp>
#include <micromouse/drive/pid_controller.hpp>
#include <micromouse/drive/trapezoid_profile.hpp>
#include <micromouse/maze/coordinate.hpp>
#include <micromouse/maze/direction.hpp>
#include <micromouse/maze/maze.hpp>
#include <micromouse/hardware/timer.hpp>
#include <micromouse/hardware/drivetrain.hpp>
#include <micromouse/hardware/ir_sensors.hpp>
#include <micromouse/hardware/measurements.hpp>
#include <micromouse/hardware/timer.hpp>
#include <micromouse/vision/vision.hpp>
#include <micromouse/subsystem.hpp>
#include <micromouse/robot/robot.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <functional>
#include <cstdint>
#include <queue>

namespace drive {

class MotionRunner : public Subsystem {
  static constexpr float VISION_ALIGN_P = 0.50;
  static constexpr float VISION_ALIGN_I = 0.00;
  static constexpr float VISION_ALIGN_D = 0.00;

 public:
  using CompletionCallback = std::function<void()>;

  enum class TurnAngle : int16_t {
    CW_180 = -177,
    CW_90 = -90,
    CCW_90 = 90,
    CCW_180 = 177,
  };

  MotionRunner(maze::Maze& maze, vision::Vision& vision, const SpeedConstraints& speeds)
      : m_maze(maze), m_vision(vision), m_speeds(speeds) {}

  void reset();
  void set_speeds(const SpeedConstraints& speeds) {
    m_speeds = speeds;
    m_vision_align_pid.reset();
  }
  const SpeedConstraints& get_speeds() const { return m_speeds; }

  void periodic() override;
  void publish_periodic_feedback() override;
  void publish_status_feedback() override;

  bool is_done() const {
    return m_motions.empty() &&
           ((m_motion_state == MotionState::NONE) || (m_motion_state == MotionState::IDLE));
  }

  struct ForwardMotionEndState {
    bool end_high = false;
    bool end_for_turn = false;
    units::millimeter_t distance_until_turn = 0_mm;
  };

  /**
   * Add a linear motion to the motion queue. The robot will start with the final velocity of the previous
   * motion (or 0 if none).
   *
   * @param start_cell          The starting cell of the robot.
   * @param start_cell_position The position of the robot in the start cell.
   * @param direction           The maze-centric direction the robot is moving in.
   * @param distance            The distance the robot should travel in the specified direction. Should be
   *                            positive.
   * @param end_state           The end state of the forward motion.
   * @param monitor_vision      Whether to monitor IR sensors during the motion and attempt to keep centered
   *                            between walls and/or adjust distance traveled to avoid
   *                            undershooting/overshooting.
   * @param completion_func     Callback function that will be called when the motion is complete.
   */
  void enqueue_forward(maze::Coordinate start_cell,
                       units::millimeter_t start_cell_position,
                       maze::Direction direction,
                       units::millimeter_t distance,
                       ForwardMotionEndState end_state,
                       bool monitor_vision = true,
                       CompletionCallback completion_func = nullptr);

  // Forward with no info about position or direction in maze.
  void enqueue_forward(units::millimeter_t start_cell_position,
                       units::millimeter_t distance,
                       ForwardMotionEndState end_state,
                       CompletionCallback completion_func = nullptr) {
    enqueue_forward(maze::Coordinate(0, 0), start_cell_position, maze::Direction::NORTH, distance, end_state,
                    false, completion_func);
  }

  /**
   * Add a stationary turn motion to the motion queue. The robot should be stopped before starting this
   * motion.
   *
   * @param angle           Angle to turn.
   * @param completion_func Callback function that will be called when the motion is complete.
   */
  void enqueue_stationary_turn(TurnAngle angle, CompletionCallback completion_func = nullptr);

  /**
   * Adds a turn motion to the motion queue. The robot will perform the turn at the final linear velocity of
   * the previous motion (or 0 if none).
   *
   * @param angle           Angle to turn.
   * @param turn_radius     The radius of the turn. Should be positive. If 0, this function will behave the
   *                        same as enqueue_stationary_turn.
   * @param completion_func Callback function that will be called when the motion is complete.
   */
  void enqueue_turn(TurnAngle angle,
                    units::millimeter_t turn_radius,
                    CompletionCallback completion_func = nullptr);

  // Temporary fix - until we fix turns undershooting, probably due to calculating curve length wrong.
  void enqueue_turn_distance(TurnAngle angle,
                             units::millimeter_t curve_length,
                             CompletionCallback completion_func = nullptr);

  void enqueue_pause();

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
    m_current_motion = nullptr;
  }

 private:
  struct Motion {
    virtual ~Motion() = default;

    enum class Type {
      FORWARD,
      TURN,
      PAUSE,
    };
    virtual Type type() const = 0;

    CompletionCallback completion_func = nullptr;
  };

  struct ForwardMotionExecutionProperties {
    units::meters_per_second_t initial_velocity;
    maze::Coordinate current_cell;
    units::millimeter_t current_cell_position;
    units::millimeter_t remaining_distance;
    bool did_vision_adjust_for_current_cell = false;

    TrapezoidProfile<units::millimeters> linear_profile;
  };

  struct ForwardMotion : public Motion {
    Type type() const override { return Type::FORWARD; }

    maze::Coordinate start_cell;
    units::millimeter_t start_cell_position;
    maze::Direction direction;
    units::millimeter_t distance;
    ForwardMotionEndState end_state;
    bool monitor_vision;

    ForwardMotionExecutionProperties exec_properties;
  };

  struct TurnMotionExecutionProperties {
    units::millimeters_per_second_t linear_velocity;

    TrapezoidProfile<units::degrees> angular_profile;
  };

  struct TurnMotion : public Motion {
    Type type() const override { return Type::TURN; }

    units::degree_t angle_value;
    units::millimeter_t curve_length = 0_mm;

    TurnMotionExecutionProperties exec_properties;
  };

  struct PauseMotion : public Motion {
    Type type() const override { return Type::PAUSE; }
  };

  void start_next_motion(units::meters_per_second_t last_velocity);
  void start_forward_motion(ForwardMotion& motion, units::meters_per_second_t last_velocity);
  void start_turn_motion(TurnMotion& motion, units::meters_per_second_t last_velocity);
  void start_pause(PauseMotion& motion);

  ChassisSpeeds process_motion(units::second_t t);
  ChassisSpeeds process_forward_motion(ForwardMotion& motion, units::second_t t);
  ChassisSpeeds process_turn_motion(TurnMotion& motion, units::second_t t);
  ChassisSpeeds process_pause(PauseMotion& motion, units::second_t t);

  hardware::RobotMeasurements& m_measurements = get_robot_measurements();
  hardware::Drivetrain& m_drivetrain = get_platform_drivetrain();
  hardware::IRSensors& m_ir_sensors = get_platform_ir_sensors();
  maze::Maze& m_maze;
  vision::Vision& m_vision;
  std::unique_ptr<hardware::Timer> m_motion_timer = make_platform_timer();

  std::queue<std::unique_ptr<Motion>> m_motions;
  std::unique_ptr<Motion> m_current_motion;

  SpeedConstraints m_speeds;

  drive::PIDController m_vision_align_pid{
      VISION_ALIGN_P,
      VISION_ALIGN_I,
      VISION_ALIGN_D,
      ROBOT_UPDATE_PERIOD_S,
  };

  enum class MotionState {
    NONE,
    IDLE,
    MOTION,
  } m_motion_state = MotionState::NONE;
};

}  // namespace drive
