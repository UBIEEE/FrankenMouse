#include <micromouse/robot/robot.hpp>

#include <micromouse/robot/cell_positions.hpp>
#include "micromouse/audio/song.hpp"
#include "micromouse/feedback/feedback_topic.hpp"
#include "micromouse/robot/status_topic.hpp"
#include "micromouse/robot/task.hpp"

#define LOG_PREFIX "[robot] "
#include <micromouse/logging.hpp>

using namespace robot;

void Robot::init() {
  m_buttons.register_button_1_callback(std::bind(&Robot::handle_button_1, this));
  m_buttons.register_button_2_callback(std::bind(&Robot::handle_button_2, this));
}

void Robot::periodic() {
  if (m_is_next_task) {
    m_is_next_task = false;
    start_next_task();
  }

  process_current_task();

  for (auto c : m_components) {
    c->periodic();
  }

  for (auto s : m_subsystems) {
    s->periodic();
  }
}

void Robot::on_connect() {
  LogInfo("feedback connected");

  m_audio_player.play_song(audio::Song::BLE_CONNECT);
  m_feedback_connected = true;
}

void Robot::on_disconnect() {
  LogInfo("feedback disconnected");

  m_audio_player.play_song(audio::Song::BLE_DISCONNECT);
  m_feedback_connected = false;

  // Stop the current task.
  run_task(Task::STOPPED);
}

void Robot::publish_periodic_feedback() {
  if (!m_feedback_connected)
    return;

  for (auto s : m_subsystems) {
    s->publish_periodic_feedback();
  }
  for (auto c : m_components) {
    c->publish_periodic_feedback();
  }

  // Send pending status updates
  if (!m_status_updates_to_publish.empty()) {
    const auto [topic, value] = *m_status_updates_to_publish.begin();
    m_feedback.publish<feedback::TopicSend::MAIN_STATUS>(StatusUpdate{topic, value});
    m_status_updates_to_publish.erase(topic);
  }
}

void Robot::publish_status_feedback() {
  if (!m_feedback_connected)
    return;

  LogInfo("publish status feedback");

  for (auto s : m_subsystems) {
    s->publish_status_feedback();
  }
  for (auto c : m_components) {
    c->publish_status_feedback();
  }

  publish_current_task();
}

void Robot::delegate_received_feedback(feedback::TopicReceive topic, const uint8_t* data) {
  LogInfo("feedback topic update: {}", feedback::topic_receive_to_string(topic));

  switch (topic) {
    using enum feedback::TopicReceive;
    case MAIN_TASK:
      if (*data < 128) {
        run_task(Task(*data));
      }
      break;
    case MAIN_COMMAND:
      handle_command(Command(*data));
      break;
    case MAIN_SONG:
      if (*data < 128) {
        m_audio_player.play_song(audio::Song(*data));
      }
      break;
    case DRIVE_PID:
      // Handled earlier by platform code.
      break;
    case DRIVE_CHASSIS_SPEEDS:
      std::memcpy(&m_chassis_speeds, data, sizeof(m_chassis_speeds));
      m_chassis_speeds_timer->reset();
      m_chassis_speeds_timer->start();
      break;
  }
}

void Robot::handle_command(Command command) {
  LogInfo("handling command: {}", command_to_string(command));

  switch (command) {
    using enum Command;
    case RESEND_ALL_FEEDBACK:
      publish_status_feedback();
      break;
    case RESET_MAZE:
      m_maze.reset();
      m_maze.init_start_cell(Maze::StartLocation::WEST_OF_GOAL);
      break;
    case CALIBRATE_VISION:
      m_vision.calibrate();
      break;
    case RESET_VISION_CALIBRATION:
      m_vision.reset_calibration();
      break;
  }
}

void Robot::handle_error(const Error& error) {
  LogInfo("error occurred: {}", error.to_string());

  run_task(Task::STOPPED);

  m_feedback.publish<feedback::TopicSend::MAIN_ERROR>(error);

  // TODO: Buzzer play error sound.
}

void Robot::handle_button_1() {
  LogInfo("button 1 pressed");

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
  LogInfo("button 2 pressed");

  // If the robot's doing something, stop it.
  if (current_task() != Task::STOPPED) {
    run_task(Task::STOPPED);
    return;
  }

  // reset_maze();
}

void Robot::arm_task(Task task) {
  LogInfo("arm task: {}", task_to_string(task));

  m_armed_task = task;
  run_task(Task::ARMED);
}

void Robot::run_task(Task task) {
  if (task == m_task) {
    LogInfo("already running task: {}", task_to_string(task));
    return;
  }

  LogInfo("run task: {}", task_to_string(task));

  m_next_task = task;
  m_is_next_task = true;
}

void Robot::end_task() {
  if (m_task == Task::MAZE_SEARCH) {
    m_search_done = true;
  }

  LogInfo("end task: {}", task_to_string(m_task));

  run_task(Task::STOPPED);
}

void Robot::start_next_task() {
  // Reset stuff.

  m_audio_player.quiet();
  m_motion_runner.stop();
  m_task = m_next_task;

  {
    const bool idle = (m_task == Task::STOPPED);

    if (!idle) {
      m_drivetrain.reset();
    }

    m_ir_sensors.set_enabled(!idle);  // Enable when not idle.
    m_imu.set_standby(idle);          // Standby when idle.
  }

  // Start task.

  switch (m_task) {
    using enum Task;
    case STOPPED:
      break;
    case MAZE_SEARCH:
      start_task_maze_search(navigation::SearchNavigator::MovementStyle::SMOOTH_MOTION);
      break;
    case MAZE_SLOW_SOLVE:
      start_task_maze_solve(false);
      break;
    case MAZE_FAST_SOLVE:
      start_task_maze_solve(true);
      break;
    case MAZE_SEARCH_START_STOP_MOTION:
      start_task_maze_search(navigation::SearchNavigator::MovementStyle::START_AND_STOP_MOTION);
      break;
    case TEST_DRIVE_STRAIGHT_FROM_BACK_WALL_TO_SENSE_SPOT:
      start_task_test_drive_straight_from_back_wall_to_sense_spot();
      break;
    case TEST_DRIVE_STRAIGHT_ONE_CELL:
      start_task_test_drive_straight_one_cell();
      break;
    case TEST_DRIVE_TURN_RIGHT_FROM_SENSE_SPOT_TO_SENSE_SPOT:
      start_task_test_drive_turn_right_from_sense_spot_to_sense_spot();
      break;
    case TEST_DRIVE_TURN_LEFT_FROM_SENSE_SPOT_TO_SENSE_SPOT:
      start_task_test_drive_turn_left_from_sense_spot_to_sense_spot();
      break;
    case TEST_DRIVE_TURN_RIGHT_IN_PLACE:
      start_task_test_drive_turn_right_in_place();
      break;
    case TEST_DRIVE_TURN_LEFT_IN_PLACE:
      start_task_test_drive_turn_left_in_place();
      break;
    case TEST_DRIVE_TURN_180_IN_PLACE:
      start_task_test_drive_turn_180_in_place();
      break;
    case TEST_GYRO:
      start_task_test_gyro();
      break;
    case TEST_DRIVE_STRAIGHT_FOUR_CELLS_FROM_BACK_WALL_WITH_VISION_ALIGN:
      start_task_test_drive_straight_four_cells_from_back_wall_with_vision_align();
      break;
    case TEST_DRIVE_RAW_SPEEDS:
      start_task_test_drive_raw_speed();
      break;
    case TEST_DRIVE_CONSTANT_SPEED:
      start_task_test_drive_constant_speed();
      break;
    case MANUAL_CHASSIS_SPEEDS:
      start_task_manual_chassis_speeds();
      break;
    case IDLE:
      // Do nothing, or something
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
      break;
  }

  publish_current_task();
}

void Robot::process_current_task() {
  switch (m_task) {
    using enum Task;
    case STOPPED:
      break;
    case MAZE_SEARCH:
    case MAZE_SEARCH_START_STOP_MOTION:
      process_task_maze_search();
      break;
    case MAZE_SLOW_SOLVE:
      process_task_maze_solve(false);
      break;
    case MAZE_FAST_SOLVE:
      process_task_maze_solve(true);
      break;
    case TEST_DRIVE_STRAIGHT_FROM_BACK_WALL_TO_SENSE_SPOT:
    case TEST_DRIVE_STRAIGHT_ONE_CELL:
    case TEST_DRIVE_TURN_RIGHT_FROM_SENSE_SPOT_TO_SENSE_SPOT:
    case TEST_DRIVE_TURN_LEFT_FROM_SENSE_SPOT_TO_SENSE_SPOT:
    case TEST_DRIVE_TURN_RIGHT_IN_PLACE:
    case TEST_DRIVE_TURN_LEFT_IN_PLACE:
    case TEST_DRIVE_TURN_180_IN_PLACE:
      process_task_test_drive();
      break;
    case TEST_GYRO:
      break;
    case TEST_DRIVE_STRAIGHT_FOUR_CELLS_FROM_BACK_WALL_WITH_VISION_ALIGN:
      break;
    case TEST_DRIVE_RAW_SPEEDS:
      break;
    case TEST_DRIVE_CONSTANT_SPEED:
      break;
    case MANUAL_CHASSIS_SPEEDS:
      process_task_manual_chassis_speeds();
      break;
    case IDLE:
      // Do nothing, or something
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
      break;
  }
}

void Robot::start_task_maze_search(navigation::SearchNavigator::MovementStyle movement_style) {
  m_search_stage = SearchStage::START_TO_GOAL;

  m_maze.reset();
  m_maze.init_start_cell(Maze::StartLocation::WEST_OF_GOAL);

  if (movement_style == navigation::SearchNavigator::MovementStyle::SMOOTH_MOTION) {
    m_motion_runner.set_speeds(m_speeds.normal_speeds);
  } else {
    m_motion_runner.set_speeds(m_speeds.slow_speeds);
  }
  m_search_navigator.set_movement_style(movement_style);

  m_search_navigator.reset_position(Maze::start(m_start_location), maze::Direction::NORTH,
                                    CellPositions::back_wall());

  m_search_navigator.search_to(Maze::GOAL_ENDPOINTS, m_floodfill);

  m_audio_player.play_song(audio::Song::BEGIN_SEARCH);
}

void Robot::process_task_maze_search() {
  using enum SearchStage;

  if (m_search_navigator.is_done()) {
    maze::CoordinateSpan next_target;

    // Check which stage was just finished, and set the next target accordingly.
    switch (m_search_stage) {
      case START_TO_GOAL:
        // Goal -> Outside Start
        next_target = Maze::outside_start_span(m_start_location);  // TODO?
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
    }

    m_search_navigator.search_to(next_target, m_floodfill);

    m_search_stage = SearchStage(uint8_t(m_search_stage) + 1);
  }
}

void Robot::start_task_maze_solve(bool fast) {
  (void)fast;

  m_solve_stage = SolveStage::START_TO_GOAL;

  fast = true;

  m_motion_runner.set_speeds(fast ? m_speeds.fast_speeds : m_speeds.normal_speeds);
  m_motion_runner.set_speeds(m_speeds.fast_speeds);
  m_solve_navigator.solve_to(Maze::start(m_start_location), maze::Direction::NORTH,
                             CellPositions::back_wall(), Maze::GOAL_ENDPOINTS, m_floodfill);

  m_audio_player.play_song(fast ? audio::Song::BEGIN_FAST_SOLVE : audio::Song::BEGIN_SLOW_SOLVE);
}

void Robot::process_task_maze_solve(bool fast) {
  (void)fast;

  if (m_solve_navigator.is_done()) {
    switch (m_solve_stage) {
      using enum SolveStage;
      case START_TO_GOAL:
        // TODO: Navigate (slow) back to start
        m_solve_stage = GOAL_TO_START;
        break;
      case GOAL_TO_START:
        end_task();
        return;
    }
  }
}

void Robot::start_task_test_drive_straight_from_back_wall_to_sense_spot() {
  const units::millimeter_t forward_distance = CellPositions::SENSING_SPOT - CellPositions::back_wall();

  m_motion_runner.enqueue_forward(CellPositions::back_wall(), forward_distance, false);
}

void Robot::start_task_test_drive_straight_one_cell() {
  m_motion_runner.enqueue_forward(0_mm, maze::Cell::WIDTH, false);
}

void Robot::start_task_test_drive_turn_right_from_sense_spot_to_sense_spot() {
  const units::millimeter_t forward_distance =
      (maze::Cell::WIDTH - CellPositions::SENSING_SPOT) + CellPositions::SEARCH_TURN_START;

  m_motion_runner.enqueue_forward(0_mm, forward_distance, true);
  m_motion_runner.enqueue_turn(drive::MotionRunner::TurnAngle::CW_90, CellPositions::SEARCH_TURN_RADIUS);
  m_motion_runner.enqueue_forward(0_mm, 0_mm, false);
}

void Robot::start_task_test_drive_turn_left_from_sense_spot_to_sense_spot() {
  const units::millimeter_t forward_distance =
      (maze::Cell::WIDTH - CellPositions::SENSING_SPOT) + CellPositions::SEARCH_TURN_START;

  m_motion_runner.enqueue_forward(0_mm, forward_distance, true);
  m_motion_runner.enqueue_turn(drive::MotionRunner::TurnAngle::CCW_90, CellPositions::SEARCH_TURN_RADIUS);
  m_motion_runner.enqueue_forward(0_mm, 0_mm, false);
}

void Robot::start_task_test_drive_turn_right_in_place() {
  m_motion_runner.enqueue_stationary_turn(drive::MotionRunner::TurnAngle::CW_90);
}

void Robot::start_task_test_drive_turn_left_in_place() {
  m_motion_runner.enqueue_stationary_turn(drive::MotionRunner::TurnAngle::CCW_90);
}

void Robot::start_task_test_drive_turn_180_in_place() {
  m_motion_runner.enqueue_stationary_turn(drive::MotionRunner::TurnAngle::CCW_180);
}

void Robot::process_task_test_drive() {
  if (m_motion_runner.is_done()) {
    end_task();
  }
}

void Robot::start_task_test_gyro() {
  drive::ChassisSpeeds speeds{};
  m_drivetrain.set_chassis_speeds(speeds);
}

void Robot::start_task_test_drive_straight_four_cells_from_back_wall_with_vision_align() {
  // TODO
}

void Robot::start_task_test_drive_raw_speed() {
  m_drivetrain.set_motors_manual(0.2, 0.2);
}

void Robot::start_task_test_drive_constant_speed() {
  drive::ChassisSpeeds speeds{.linear_velocity = 200_mmps, .angular_velocity = 0_deg_per_s};
  m_drivetrain.set_chassis_speeds(speeds);
}

void Robot::start_task_manual_chassis_speeds() {
  m_chassis_speeds = drive::ChassisSpeeds{};
  m_chassis_speeds_timer->reset();
  m_chassis_speeds_timer->start();
}

void Robot::process_task_manual_chassis_speeds() {
  const units::second_t elapsed_time = m_chassis_speeds_timer->get();

  if (elapsed_time > 0.5_s) {
    m_chassis_speeds = {};
  }

  m_drivetrain.set_chassis_speeds(m_chassis_speeds);
}

void Robot::start_task_armed() {
  m_audio_player.play_song(audio::Song::ARMED, true);
}

void Robot::process_task_armed() {
  const std::array<float, 4>& readings = m_ir_sensors.get_raw_readings();

  using enum hardware::IRSensors::Sensor;

  const bool left_blocked = readings[MID_LEFT] > 0.8f;
  const bool right_blocked = readings[MID_RIGHT] > 0.8f;

  if (!left_blocked && !right_blocked)
    return;

  m_armed_trigger_side = ArmedTriggerSide::LEFT;
  if (right_blocked && !left_blocked) {
    m_armed_trigger_side = ArmedTriggerSide::RIGHT;
  }

  run_task(Task::ARMED_TRIGGERING);
}

void Robot::start_task_armed_triggering() {
  m_audio_player.play_song(audio::Song::ARMED_TRIGGERING, true);

  m_armed_trigger_timer->reset();
  m_armed_trigger_timer->start();
}

void Robot::process_task_armed_triggering() {
  const std::array<float, 4>& readings = m_ir_sensors.get_raw_readings();

  using enum hardware::IRSensors::Sensor;

  const hardware::IRSensors::Sensor sensor =
      (m_armed_trigger_side == ArmedTriggerSide::LEFT) ? MID_LEFT : MID_RIGHT;

  const bool blocked = readings[sensor] > 0.8f;

  // Don't even think about moving until that hand is gone!
  if (blocked)
    return;

  const bool time_over = m_armed_trigger_timer->get() > 1_s;

  if (time_over) {
    run_task(Task::ARMED_TRIGGERED);
    return;
  }

  // Go back to armed since they didn't hold their hand there long enough.
  run_task(Task::ARMED);
}

void Robot::start_task_armed_triggered() {
  {
    using enum audio::Song;
    audio::Song song;

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

    m_audio_player.play_song(song);
  }

  m_armed_trigger_timer->reset();
  m_armed_trigger_timer->start();
}

void Robot::process_task_armed_triggered() {
  if (m_armed_trigger_timer->get() < 1_s)
    return;

  // Trigger side points to goal.
  m_start_location = m_armed_trigger_side == ArmedTriggerSide::RIGHT ? Maze::StartLocation::WEST_OF_GOAL
                                                                     : Maze::StartLocation::EAST_OF_GOAL;

  // TODO: Calibration

  run_task(m_armed_task);
}

void Robot::publish_current_task() {
  using namespace feedback;
  m_feedback.publish<TopicSend::MAIN_TASK>(m_task);
}

void Robot_Init(void) {
  Robot::get().init();
}

void Robot_Periodic(void) {
  Robot::get().periodic();
}

void Robot_OnConnect(void) {
  Robot::get().on_connect();
}

void Robot_OnDisconnect(void) {
  Robot::get().on_disconnect();
}

void Robot_PublishPeriodicFeedback(void) {
  Robot::get().publish_periodic_feedback();
}

void Robot_ReportError(void) {}

void Robot_DelegateReceivedFeedback(uint8_t topic, uint8_t* data) {
  if (topic >= _FB_TOPIC_RECEIVE_COUNT)
    return;

  Robot::get().delegate_received_feedback(feedback::TopicReceive(topic), data);
}
