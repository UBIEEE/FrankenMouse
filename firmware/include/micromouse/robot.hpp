#pragma once

#include <micromouse/robot.h>
#include <micromouse/audio/audio_player.hpp>
#include <micromouse/drive/drive_controller.hpp>
#include <micromouse/feedback_topic.hpp>
#include <micromouse/hardware/battery_status.hpp>
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
#include <micromouse/navigation/navigator.hpp>
#include <micromouse/singleton.hpp>
#include <micromouse/solver/flood_fill.hpp>
#include <micromouse/vision/vision.hpp>

class Robot : public Singleton<Robot> {
  drive::SpeedConfig m_speeds{};
  maze::Maze m_maze;
  FloodFillSolver m_floodfill{m_maze};

  // Hardware Components

  hardware::BatteryStatus& m_battery_status = get_platform_battery_status();
  hardware::Buzzer& m_buzzer = get_platform_buzzer();
  hardware::Drivetrain& m_drivetrain = get_platform_drivetrain();
  hardware::IMU& m_imu = get_platform_imu();
  hardware::IRSensors& m_ir_sensors = get_platform_ir_sensors();
  hardware::Feedback& m_feedback = get_platform_feedback();
  hardware::Buttons& m_buttons = get_platform_buttons();

  const std::array<hardware::Component*, 7> m_components{
      &m_battery_status, &m_buzzer,   &m_drivetrain, &m_imu,
      &m_ir_sensors,     &m_feedback, &m_buttons,
  };

  // Logic Subsystems

  vision::Vision m_vision;
  audio::AudioPlayer m_audio_player;
  drive::DriveController m_drive_controller{m_vision, m_speeds.normal_speeds};
  navigation::Navigator m_navigator{m_drive_controller, m_vision, m_maze};

  Maze::StartLocation m_start_location = Maze::StartLocation::WEST_OF_GOAL;

  const std::array<Subsystem*, 4> m_subsystems{
      &m_drive_controller,
      &m_audio_player,
      &m_vision,
      &m_navigator,
  };

  bool m_feedback_connected = false;

 private:
  std::array<maze::CoordinateSpan, 4> get_search_targets() {
    return {Maze::GOAL_ENDPOINTS, Maze::outside_start_span(m_start_location),
            Maze::GOAL_ENDPOINTS, Maze::start_span(m_start_location)};
  }

  std::array<maze::CoordinateSpan, 4> get_solve_targets() {
    return {Maze::GOAL_ENDPOINTS, Maze::start_span(m_start_location)};
  }

 public:
  void init();
  void periodic();
  void on_connect();
  void on_disconnect();
  void publish_periodic_feedback();
  void publish_extra_feedback();
  void delegate_received_feedback(FeedbackTopicReceive topic,
                                  const uint8_t* data);

 public:
  enum class Task : uint8_t {
    STOPPED = 0,

    //
    // 1-10: maze tasks.
    //

    MAZE_SEARCH = 1,
    MAZE_SLOW_SOLVE = 2,
    MAZE_FAST_SOLVE = 3,

    //
    // 11-20: test drive tasks.
    //

    // Test drive movements, starting from the back wall.
    TEST_DRIVE_STRAIGHT = 11,
    TEST_DRIVE_LEFT_TURN = 12,
    TEST_DRIVE_RIGHT_TURN = 13,
    TEST_DRIVE_TURN_180 = 14,

    TEST_GYRO = 15,
    TEST_DRIVE_STRAIGHT_VISION_ALIGN = 16,

    //
    // 21-30: Manual control tasks.
    //
    
    MANUAL_CHASSIS_SPEEDS = 21,

    //
    // 100+: other
    //

    ARMED = 100,
    ARMED_TRIGGERING,
    ARMED_TRIGGERED,

    VISION_CALIBRATE,

    _COUNT,
  };

 private:
  Task m_task = Task::STOPPED;

  Task m_next_task = Task::STOPPED;
  bool m_is_next_task = false;

  Task m_armed_task;
  std::unique_ptr<hardware::Timer> m_armed_trigger_timer =
      make_platform_timer();

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

 public:
  Task current_task() const { return m_task; }

 private:
  void handle_button_1();
  void handle_button_2();

 private:
  void arm_task(Task task);
  void run_task(Task task);

  void end_task();

  void set_start_location(Maze::StartLocation start_location) {
    m_start_location = start_location;
  }

 private:
  void start_next_task();

  void start_task_maze_search();
  void start_task_maze_solve(bool fast);

  void start_task_test_drive_straight();
  void start_task_test_drive_left_turn();
  void start_task_test_drive_right_turn();
  void start_task_test_drive_turn_180();
  void start_task_test_gyro();
  void start_task_test_drive_straight_vision_align();

  void start_task_armed();
  void start_task_armed_triggering();
  void start_task_armed_triggered();

  void process_current_task();

  void process_task_maze_search();
  void process_task_maze_solve(bool fast);

  void process_task_test_drive();

  void process_task_armed();
  void process_task_armed_triggering();
  void process_task_armed_triggered();

 private:
  void publish_current_task();
};
