#include "Basic/robot.hpp"

#include "Basic/constants.hpp"
#include "custom_stm.h"
#include "main.h"
#include <array>

void Robot::init() {
  for (auto s : m_subsystems) {
    s->init();
  }
}

void Robot::update() {
  // Task management.
  if (m_is_next_task) {
    m_is_next_task = false;
    start_next_task();
  }
  process_current_task();

  // Process subsystems.
  for (auto s : m_subsystems) {
    s->process();
  }
}

void Robot::on_connect() { m_buzzer.play_song(Buzzer::Song::BLE_CONECT); }

void Robot::dashboard_app_ready() {
  send_current_task();

  for (auto s : m_subsystems) {
    s->on_connect_send_feedback();
  }

  m_ble_connected = true;
}

void Robot::on_disconnect() {
  m_buzzer.play_song(Buzzer::Song::BLE_DISCONECT);
  m_ble_connected = false;
}

void Robot::send_feedback() {
  if (!m_ble_connected) return;

  for (auto s : m_subsystems) {
    s->send_feedback();
  }
}

void Robot::handle_button_1() {
  // If the robot's doing something, stop it.
  if (current_task() != Task::STOPPED) {
    run_task(Task::STOPPED);
    return;
  }

  if (m_search_done) {
    arm_task(Task::MAZE_SLOW_SOLVE);
  } else {
    arm_task(Task::MAZE_SEARCH);
  }
}

void Robot::handle_button_2() {
  // If the robot's doing something, stop it.
  if (current_task() != Task::STOPPED) {
    run_task(Task::STOPPED);
    return;
  }

  reset_maze();
}

void Robot::arm_task(Task task) {
  m_armed_task = task;
  run_task(Task::ARMED);
}

void Robot::run_task(Task task) {
  if (m_task == task) return;

  m_next_task    = task;
  m_is_next_task = true;
}

void Robot::end_task() {
  if (m_task == Task::MAZE_SEARCH) {
    m_search_done = true;
  }

  run_task(Task::STOPPED);
}

void Robot::reset_maze() {
  m_maze.reset();
  m_search_done = false;
}

void Robot::start_next_task() {
  // Reset stuff.

  m_buzzer.quiet();
  m_navigator.stop();
  m_drive_controller.stop();
  m_drive.stop();
  m_task = m_next_task;

  {
    const bool idle = (m_task == Task::STOPPED);

    if (!idle) {
      m_drive.reset();
    }

    m_vision.set_enabled(!idle);          // Enable when not idle.
    m_drive.imu().set_standby_mode(idle); // Standby when idle.
  }

  // Start task.

  switch (m_task) {
    using enum Task;
  case STOPPED:
    break;
  case MAZE_SEARCH:
    start_task_maze_search();
    break;
  case MAZE_SLOW_SOLVE:
    start_task_maze_solve(false);
    break;
  case MAZE_FAST_SOLVE:
    start_task_maze_solve(true);
    break;
  case TEST_DRIVE_STRAIGHT:
    start_task_test_drive_straight();
    break;
  case TEST_DRIVE_LEFT_TURN:
    start_task_test_drive_left_turn();
    break;
  case TEST_DRIVE_RIGHT_TURN:
    start_task_test_drive_right_turn();
    break;
  case TEST_GYRO:
    start_task_test_gyro();
    break;
  case TEST_VISION_SIDE:
    start_task_test_vision_side();
    break;
  case ARMED:
    start_task_armed();
    break;
  case ARMED_TRIGGERING:
    start_task_armed_triggering();
    break;
  case ARMED_TRIGGERED:
    start_task_armed_triggered();
    break;
  case VISION_CALIBRATE:
    start_task_vision_calibrate();
    break;
  default:
    ErrorManager::get().fatal_error(Error::UNREACHABLE);
  }

  send_current_task();
}

void Robot::start_task_maze_search() {
  m_search_stage = SearchStage::START_TO_GOAL;

  m_navigator.reset_position(Maze::start(m_start_location),
                             maze::Direction::NORTH,
                             Constants::RobotCellPositions::BACK_WALL_MM);

  m_navigator.search_to(Maze::GOAL_ENDPOINTS, m_flood_fill_solver);
}

void Robot::start_task_maze_solve(bool fast) {
  m_solve_stage = SolveStage::START_TO_GOAL;

  m_navigator.reset_position(Maze::start(m_start_location),
                             maze::Direction::NORTH,
                             Constants::RobotCellPositions::BACK_WALL_MM);

  m_navigator.solve_to(Maze::GOAL_ENDPOINTS, fast);
}

void Robot::start_task_test_drive_straight() {
  m_drive_controller.reset();
  m_drive_controller.enqueue_forward(100.0, false);
}

void Robot::start_task_test_drive_left_turn() {
  m_drive_controller.reset();

  using Positions = Constants::RobotCellPositions;
  using enum DriveController::TurnAngle;

  // Forward to turning spot while accelerating.
  m_drive_controller.enqueue_forward(Positions::TURNING_BEGIN_SPOT_MM - Positions::BACK_WALL_MM);
  // Turn
  m_drive_controller.enqueue_turn(CW_90, Positions::TURNING_RADIUS_MM);
  // Forward to center while decelerating.
  m_drive_controller.enqueue_forward(Positions::CENTERED_MM - Positions::TURNING_END_SPOT_MM, false);
}

void Robot::start_task_test_drive_right_turn() {
  m_drive_controller.reset();

  // TODO
}

void Robot::start_task_test_gyro() { m_drive.control_speed_velocity(0.f, 0.f); }

void Robot::start_task_test_vision_side() {
  m_drive_controller.make_idle();
}

void Robot::start_task_armed() {
  m_buzzer.play_song(Buzzer::Song::ARMED, true);
}

void Robot::start_task_armed_triggering() {
  m_buzzer.play_song(Buzzer::Song::ARMED_TRIGGERING, true);

  m_armed_trigger_timer.reset();
  m_armed_trigger_timer.start();
}

void Robot::start_task_armed_triggered() {
  {
    using enum Buzzer::Song;
    Buzzer::Song song;

    switch (m_next_task) {
    case Task::MAZE_SEARCH:
      song = BEGIN_SEARCH;
      break;
    case Task::MAZE_SLOW_SOLVE:
      song = BEGIN_SLOW_SOLVE;
      break;
    case Task::MAZE_FAST_SOLVE:
      song = BEGIN_FAST_SOLVE;
      break;
    default:
      song = BEGIN_OTHER;
      break;
    }

    m_buzzer.play_song(song);
  }

  m_armed_trigger_timer.reset();
  m_armed_trigger_timer.start();
}

void Robot::start_task_vision_calibrate() {
  m_calibration_timer.reset();
  m_calibration_timer.start();
}

void Robot::process_current_task() {
  switch (m_task) {
    using enum Task;
  case STOPPED:
    break;
  case MAZE_SEARCH:
    process_task_maze_search();
    break;
  case MAZE_SLOW_SOLVE:
    process_task_maze_solve(false);
    break;
  case MAZE_FAST_SOLVE:
    process_task_maze_solve(true);
    break;
  case TEST_DRIVE_STRAIGHT:
  case TEST_DRIVE_LEFT_TURN:
  case TEST_DRIVE_RIGHT_TURN:
    process_task_test_drive();
    break;
  case TEST_GYRO:
    break;
  case TEST_VISION_SIDE:
    break;
  case ARMED:
    process_task_armed();
    break;
  case ARMED_TRIGGERING:
    process_task_armed_triggering();
    break;
  case ARMED_TRIGGERED:
    process_task_armed_triggered();
    break;
  case VISION_CALIBRATE:
    process_task_vision_calibrate();
    break;
  default:
    ErrorManager::get().fatal_error(Error::UNREACHABLE);
  }
}

void Robot::process_task_maze_search() {
  using enum SearchStage;

  if (m_navigator.is_done()) {
    maze::CoordinateSpan next_target;

    // Check which stage was just finished, and set the next target accordingly.
    switch (m_search_stage) {
    case START_TO_GOAL:
      // Goal -> Outside Start
      next_target = Maze::outside_start_span(m_start_location);
      break;
    case GOAL_TO_OUTSIDE_START:
      // Outside Start -> Goal
      next_target = Maze::GOAL_ENDPOINTS;
      break;
    case OUTSIDE_START_TO_GOAL:
      // Goal -> Start
      next_target = Maze::start_span(m_start_location);
      break;
    case GOAL_TO_START:
      end_task();
      return;
    default:
      ErrorManager::get().fatal_error(Error::UNREACHABLE);
    }

    m_navigator.search_to(next_target, m_flood_fill_solver);

    m_search_stage = SearchStage(uint8_t(m_search_stage) + 1);
  }
}

void Robot::process_task_maze_solve(bool fast) {
  if (m_navigator.is_done()) {
    switch (m_solve_stage) {
      using enum SolveStage;
    case START_TO_GOAL:
      m_navigator.solve_to(Maze::start_span(m_start_location), fast);
      m_solve_stage = GOAL_TO_START;
      break;
    case GOAL_TO_START:
      end_task();
      return;
    }
  }
}

void Robot::process_task_test_drive() {}

void Robot::process_task_armed() {

  using enum Vision::Sensor;

  const bool left_blocked = m_vision.get_distance_mm(FAR_LEFT) <
                            Constants::Vision::HAND_TRIGGER_THRESHOLD_MM;

  const bool right_blocked = m_vision.get_distance_mm(FAR_RIGHT) <
                             Constants::Vision::HAND_TRIGGER_THRESHOLD_MM;

  if (!left_blocked && !right_blocked) return;

  m_armed_trigger_side = ArmedTriggerSide::LEFT;
  if (right_blocked) {
    m_armed_trigger_side = ArmedTriggerSide::RIGHT;
  }

  run_task(Task::ARMED_TRIGGERING);
}

void Robot::process_task_armed_triggering() {
  using enum Vision::Sensor;

  const Vision::Sensor sensor =
      (m_armed_trigger_side == ArmedTriggerSide::LEFT) ? FAR_LEFT : FAR_RIGHT;

  const bool blocked = m_vision.get_distance_mm(sensor) <
                       Constants::Vision::HAND_TRIGGER_THRESHOLD_MM;

  // Don't even think about moving until that hand is gone!
  if (blocked) return;

  const bool time_over = m_armed_trigger_timer.elapsed_s() >
                         Constants::Vision::HAND_TRIGGER_TIME_S;

  if (time_over) {
    run_task(Task::ARMED_TRIGGERED);
    return;
  }

  // Go back to armed since they didn't hold their hand there long enough.
  run_task(Task::ARMED);
}

void Robot::process_task_armed_triggered() {
  if (m_armed_trigger_timer.elapsed_s() <
      Constants::Vision::HAND_TRIGGER_TIME_S)
    return;

  // Trigger side points to goal.
  m_start_location = m_armed_trigger_side == ArmedTriggerSide::LEFT
                         ? Maze::StartLocation::WEST_OF_GOAL
                         : Maze::StartLocation::EAST_OF_GOAL;

  // TODO: Calibration

  // run_task(m_armed_task);
  run_task(Task::STOPPED);
}

void Robot::process_task_vision_calibrate() {
  if (m_calibration_timer.elapsed_s() < Constants::Vision::CALIBRATION_TIME_S)
    return;

  m_vision.calibrate();

  end_task();
}

void Robot::send_current_task() {
  uint8_t data[2] = {uint8_t(m_task), uint8_t(m_start_location)};

  Custom_STM_App_Update_Char(CUSTOM_STM_MAIN_TASK_CHAR, data);
}

// External interrupt callback.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
  case BUTTON_1_Pin: // (BOOT0)
    Robot::get().handle_button_1();
    break;
  case BUTTON_2_Pin:
    Robot::get().handle_button_2();
    break;
  case IMU_INT1_Pin:
    Robot::get().drive().imu().int1_handler();
    break;
  }
}

#include "Basic/robot.h"

void Robot_Init(void) { Robot::get().init(); }
void Robot_Update(void) { Robot::get().update(); }
void Robot_OnConnect(void) { Robot::get().on_connect(); }
void Robot_DashboardAppReady(void) { Robot::get().dashboard_app_ready(); }
void Robot_OnDisconnect(void) { Robot::get().on_disconnect(); }
void Robot_SendFeedback(void) { Robot::get().send_feedback(); }

void RobotControl_RunTask(uint8_t task, uint8_t start_location) {
  using enum Robot::Task;

  if (task >= uint8_t(ARMED)) return;

  const Robot::Task final_task = Robot::Task(task);

  Robot::get().run_task(final_task);

  if (final_task == MAZE_SEARCH) {
    if (start_location >= 2) return;
    Robot::get().set_start_location(Maze::StartLocation(start_location));
  }
}

void RobotControl_ResetMaze(void) { Robot::get().reset_maze(); }

void Vision_Calibrate(uint8_t calibrate) {
  if (calibrate == 0) {
    Robot::get().vision().reset_calibration();
    return;
  }
  if (calibrate != 1) return;

  if (Robot::get().vision().is_enabled()) {
    Robot::get().vision().calibrate();
    return;
  }

  Robot::get().run_task(Robot::Task::VISION_CALIBRATE);
}
