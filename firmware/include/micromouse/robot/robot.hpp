#pragma once

#include <micromouse/robot/robot.h>
#include <micromouse/audio/audio_player.hpp>
#include <micromouse/drive/motion_runner.hpp>
#include <micromouse/drive/kinematics.hpp>
#include <micromouse/feedback/feedback_topic.hpp>
#include <micromouse/hardware/battery.hpp>
#include <micromouse/hardware/buttons.hpp>
#include <micromouse/hardware/buzzer.hpp>
#include <micromouse/hardware/component.hpp>
#include <micromouse/hardware/drivetrain.hpp>
#include <micromouse/hardware/feedback.hpp>
#include <micromouse/hardware/imu.hpp>
#include <micromouse/hardware/ir_sensors.hpp>
#include <micromouse/hardware/timer.hpp>
#include <micromouse/maze/coordinate.hpp>
#include <micromouse/maze/maze.hpp>
#include <micromouse/navigation/search_navigator.hpp>
#include <micromouse/navigation/solve_navigator.hpp>
#include <micromouse/navigation/solvers/flood_fill.hpp>
#include <micromouse/singleton.hpp>
#include <micromouse/robot/task.hpp>
#include <micromouse/robot/status_topic.hpp>
#include <micromouse/vision/vision.hpp>
#include <array>
#include <unordered_map>

namespace robot {

class Robot : public Singleton<Robot> {
  drive::SpeedConfig m_speeds{};
  maze::Maze m_maze;
  navigation::FloodFillSolver m_floodfill{m_maze};

  // Hardware Components

  hardware::Battery& m_battery = get_platform_battery();
  hardware::Buzzer& m_buzzer = get_platform_buzzer();
  hardware::Drivetrain& m_drivetrain = get_platform_drivetrain();
  hardware::IMU& m_imu = get_platform_imu();
  hardware::IRSensors& m_ir_sensors = get_platform_ir_sensors();
  hardware::Feedback& m_feedback = get_platform_feedback();
  hardware::Buttons& m_buttons = get_platform_buttons();

  const std::array<hardware::Component*, 7> m_components{
      &m_battery, &m_buzzer, &m_drivetrain, &m_imu, &m_ir_sensors, &m_feedback, &m_buttons,
  };

  // Logic Subsystems

  vision::Vision m_vision;
  audio::AudioPlayer m_audio_player;
  drive::MotionRunner m_motion_runner{m_maze, m_vision, m_speeds.normal_speeds};
  navigation::SearchNavigator m_search_navigator{m_motion_runner, m_vision, m_maze};
  navigation::SolveNavigator m_solve_navigator{m_motion_runner, m_vision, m_maze};

  const std::array<Subsystem*, 5> m_subsystems{
      &m_motion_runner,
      &m_audio_player,
      &m_vision,
      &m_search_navigator,
      &m_solve_navigator,
  };

  bool m_feedback_connected = false;
  Maze::StartLocation m_start_location = Maze::StartLocation::WEST_OF_GOAL;

  Task m_task = Task::STOPPED;

  Task m_next_task = Task::STOPPED;
  bool m_is_next_task = false;

  Task m_armed_task;
  std::unique_ptr<hardware::Timer> m_armed_trigger_timer = make_platform_timer();

  enum class ArmedTriggerSide : bool {
    LEFT,
    RIGHT,
  } m_armed_trigger_side;

  bool m_search_done = false;

  enum class SearchStage : uint8_t {
    START_TO_GOAL = 0,
    GOAL_TO_OUTSIDE_START = 1,
    OUTSIDE_START_TO_GOAL = 2,
    GOAL_TO_START = 3,
  } m_search_stage = SearchStage::START_TO_GOAL;

  enum class SolveStage : uint8_t {
    START_TO_GOAL = 0,
    GOAL_TO_START = 1,
  } m_solve_stage = SolveStage::START_TO_GOAL;

  drive::ChassisSpeeds m_chassis_speeds{};
  std::unique_ptr<hardware::Timer> m_chassis_speeds_timer = make_platform_timer();

  std::unordered_map<StatusTopic, float> m_status_updates_to_publish;

 public:
  void init();
  void periodic();
  void on_connect();
  void on_disconnect();
  void publish_periodic_feedback();
  void publish_status_feedback();
  void delegate_received_feedback(feedback::TopicReceive topic, const uint8_t* data);

  Task current_task() const { return m_task; }

  template <ErrorCode Code>
  void error(Code code) {
    Error error = Error::create<Code>(get_system_timestamp().to<uint32_t>(), code);
    handle_error(error);
  }

  template <StatusTopic Topic>
  void feedback_status_update(float value = 0.f) {
    if (Topic != StatusTopic::NONE) {
      m_status_updates_to_publish[Topic] = value;
    }
  }

 private:
  void handle_command(Command command);
  void handle_error(const Error& error);

  void handle_button_1();
  void handle_button_2();

  void set_start_location(Maze::StartLocation start_location) { m_start_location = start_location; }

  std::array<maze::CoordinateSpan, 4> get_search_targets() {
    return {Maze::GOAL_ENDPOINTS, Maze::outside_start_span(m_start_location), Maze::GOAL_ENDPOINTS,
            Maze::start_span(m_start_location)};
  }

  std::array<maze::CoordinateSpan, 4> get_solve_targets() {
    return {Maze::GOAL_ENDPOINTS, Maze::start_span(m_start_location)};
  }

  // Task things

  void arm_task(Task task);
  void run_task(Task task);

  void end_task();

  void start_next_task();
  void process_current_task();

  void start_task_maze_search(navigation::SearchNavigator::MovementStyle movement_style);
  void process_task_maze_search();

  void start_task_maze_solve(bool fast);
  void process_task_maze_solve(bool fast);

  void start_task_test_drive_straight_from_back_wall_to_sense_spot();
  void start_task_test_drive_straight_one_cell();
  void start_task_test_drive_turn_right_from_sense_spot_to_sense_spot();
  void start_task_test_drive_turn_left_from_sense_spot_to_sense_spot();
  void start_task_test_drive_turn_right_in_place();
  void start_task_test_drive_turn_left_in_place();
  void start_task_test_drive_turn_180_in_place();
  void process_task_test_drive();

  void start_task_test_gyro();

  void start_task_test_drive_straight_four_cells_from_back_wall_with_vision_align();

  void start_task_test_drive_raw_speed();

  void start_task_test_drive_constant_speed();

  void start_task_manual_chassis_speeds();
  void process_task_manual_chassis_speeds();

  void start_task_armed();
  void process_task_armed();

  void start_task_armed_triggering();
  void process_task_armed_triggering();

  void start_task_armed_triggered();
  void process_task_armed_triggered();


  void publish_current_task();
};

bool is_real();

}  // namespace robot

using robot::Robot;
