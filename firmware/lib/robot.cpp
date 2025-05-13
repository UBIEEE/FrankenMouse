#include <micromouse/robot.hpp>

#include <micromouse/robot_cell_positions.hpp>

void Robot::init() {
  m_buttons.register_btn1_callback(std::bind(&Robot::handle_button_1, this));
  m_buttons.register_btn2_callback(std::bind(&Robot::handle_button_2, this));
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
  m_audio_player.play_song(audio::Song::BLE_CONECT);
  m_feedback_connected = true;
}

void Robot::on_disconnect() {
  m_audio_player.play_song(audio::Song::BLE_DISCONECT);
  m_feedback_connected = false;
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
}

void Robot::publish_extra_feedback() {
  if (!m_feedback_connected)
    return;

  for (auto s : m_subsystems) {
    s->publish_extra_feedback();
  }
  for (auto c : m_components) {
    c->publish_extra_feedback();
  }

  publish_current_task();
}

void Robot::delegate_received_feedback(FeedbackTopicReceive topic,
                                       const uint8_t* data) {
  switch (topic) {
    using enum FeedbackTopicReceive;
    case MAIN_TASK:
      if (data[0] >= static_cast<uint8_t>(Task::_COUNT))
        return;
      run_task(Task(data[0]));
      return;
    case VISION_CALIBRATE:
      if (data[0] == 0) {
        m_vision.reset_calibration();
      } else if (data[0] == 1) {
        m_vision.calibrate();
      }
      return;
    case MAZE_RESET:
      m_maze.reset();
      m_maze.init_start_cell(Maze::StartLocation::WEST_OF_GOAL);
      return;
    case MUSIC_PLAY_SONG:
      if (data[0] >= static_cast<uint8_t>(audio::Song::_COUNT))
        return;
      m_audio_player.play_song(audio::Song(data[0]));
      return;
    case DRIVE_CHASSIS_SPEEDS:
      // TODO:
      return;
    default:
      return;
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

  // reset_maze();
}

void Robot::arm_task(Task task) {
  m_armed_task = task;
  run_task(Task::ARMED);
}

void Robot::run_task(Task task) {
  if (task == m_task)
    return;

  m_next_task = task;
  m_is_next_task = true;
}

void Robot::end_task() {
  if (m_task == Task::MAZE_SEARCH) {
    m_search_done = true;
  }

  run_task(Task::STOPPED);
}

void Robot::start_next_task() {
  // Reset stuff.

  m_audio_player.quiet();
  m_drive_controller.stop();
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
    case TEST_DRIVE_TURN_180:
      start_task_test_drive_turn_180();
      break;
    case TEST_GYRO:
      start_task_test_gyro();
      break;
    case TEST_DRIVE_STRAIGHT_VISION_ALIGN:
      start_task_test_drive_straight_vision_align();
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
    default:
      // TODO Error
      break;
  }

  publish_current_task();
}

void Robot::start_task_maze_search() {
  m_search_stage = SearchStage::START_TO_GOAL;

  m_maze.reset();
  m_maze.init_start_cell(Maze::StartLocation::WEST_OF_GOAL);

  m_navigator.reset_position(Maze::start(m_start_location),
                             maze::Direction::NORTH,
                             RobotCellPositions::back_wall_mm());

  m_navigator.search_to(Maze::GOAL_ENDPOINTS, m_floodfill);
}

void Robot::start_task_maze_solve(bool fast) {
  (void)fast;

  m_solve_stage = SolveStage::START_TO_GOAL;

  m_navigator.reset_position(Maze::start(m_start_location),
                             maze::Direction::NORTH,
                             RobotCellPositions::back_wall_mm());

  // m_navigator.solve_to(Maze::GOAL_ENDPOINTS, fast);
}

void Robot::start_task_test_drive_straight() {
  m_drive_controller.reset();

  const float forward_distance =
      maze::Cell::WIDTH_MM - RobotCellPositions::back_wall_mm();

  m_drive_controller.enqueue_forward(forward_distance, false);
}

void Robot::start_task_test_drive_left_turn() {
  m_drive_controller.reset();

  const float forward_distance =
      maze::Cell::WIDTH_MM - RobotCellPositions::back_wall_mm();

  m_drive_controller.enqueue_forward(forward_distance);
  m_drive_controller.enqueue_turn(drive::DriveController::TurnAngle::CCW_90,
                                  maze::Cell::WIDTH_MM / 2.f);
  m_drive_controller.enqueue_forward(maze::Cell::WIDTH_MM, false);
}

void Robot::start_task_test_drive_right_turn() {
  m_drive_controller.reset();

  const float forward_distance =
      maze::Cell::WIDTH_MM - RobotCellPositions::back_wall_mm();

  m_drive_controller.enqueue_forward(forward_distance);
  m_drive_controller.enqueue_turn(drive::DriveController::TurnAngle::CW_90,
                                  maze::Cell::WIDTH_MM / 2.f);
  m_drive_controller.enqueue_forward(maze::Cell::WIDTH_MM, false);
}

void Robot::start_task_test_drive_turn_180() {
  m_drive_controller.reset();
  m_drive_controller.enqueue_turn(drive::DriveController::TurnAngle::CW_180);
}

void Robot::start_task_test_gyro() {
  drive::ChassisSpeeds speeds{0.f, 0.f};
  m_drivetrain.set_chassis_speeds(speeds);
}

void Robot::start_task_test_drive_straight_vision_align() {
  // TODO
}

void Robot::start_task_armed() {
  m_audio_player.play_song(audio::Song::ARMED, true);
}

void Robot::start_task_armed_triggering() {
  m_audio_player.play_song(audio::Song::ARMED_TRIGGERING, true);

  m_armed_trigger_timer->reset();
  m_armed_trigger_timer->start();
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
    case TEST_DRIVE_STRAIGHT_VISION_ALIGN:
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
    default:
      // TODO Error
      break;
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
        // TODO Error
        return;
    }

    m_navigator.search_to(next_target, m_floodfill);

    m_search_stage = SearchStage(uint8_t(m_search_stage) + 1);
  }
}

void Robot::process_task_maze_solve(bool fast) {
  (void)fast;

  if (m_navigator.is_done()) {
    switch (m_solve_stage) {
      using enum SolveStage;
      case START_TO_GOAL:
        // m_navigator.solve_to(Maze::start_span(m_start_location), fast);
        m_solve_stage = GOAL_TO_START;
        break;
      case GOAL_TO_START:
        end_task();
        return;
    }
  }
}

void Robot::process_task_test_drive() {
  if (m_drive_controller.is_done()) {
    end_task();
  }
}

void Robot::process_task_armed() {
  const float* readings = m_ir_sensors.get_raw_readings();

  using enum hardware::IRSensors::Sensor;

  const bool left_blocked = readings[MID_LEFT] > 0.8f;
  const bool right_blocked = readings[MID_RIGHT] > 0.8f;

  if (!left_blocked && !right_blocked)
    return;

#if 0
  m_armed_trigger_side = ArmedTriggerSide::LEFT;
  if (right_blocked) {
    m_armed_trigger_side = ArmedTriggerSide::RIGHT;
  }
#else
  if (left_blocked || right_blocked) {
    m_armed_trigger_side = ArmedTriggerSide::RIGHT;
  }
#endif

  run_task(Task::ARMED_TRIGGERING);
}

void Robot::process_task_armed_triggering() {
  const float* readings = m_ir_sensors.get_raw_readings();

  using enum hardware::IRSensors::Sensor;

  const hardware::IRSensors::Sensor sensor =
      (m_armed_trigger_side == ArmedTriggerSide::LEFT) ? MID_LEFT : MID_RIGHT;

  const bool blocked = readings[sensor] > 0.8f;

  // Don't even think about moving until that hand is gone!
  if (blocked)
    return;

  const bool time_over = m_armed_trigger_timer->elapsed_s() > 1.f;

  if (time_over) {
    run_task(Task::ARMED_TRIGGERED);
    return;
  }

  // Go back to armed since they didn't hold their hand there long enough.
  run_task(Task::ARMED);
}

void Robot::process_task_armed_triggered() {
  if (m_armed_trigger_timer->elapsed_s() < 1.f)
    return;

  // Trigger side points to goal.
  m_start_location = m_armed_trigger_side == ArmedTriggerSide::LEFT
                         ? Maze::StartLocation::WEST_OF_GOAL
                         : Maze::StartLocation::EAST_OF_GOAL;

  // TODO: Calibration

  run_task(m_armed_task);
}

void Robot::publish_current_task() {
  uint8_t data[1] = {static_cast<uint8_t>(m_task)};
  m_feedback.publish_topic(FeedbackTopicSend::MAIN_TASK, data);
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

void Robot_PublishExtraFeedback(void) {
  Robot::get().publish_extra_feedback();
}

void Robot_ReportError(void) {}

void Robot_DelegateReceivedFeedback(uint8_t topic, uint8_t* data) {
  if (topic >= _FB_TOPIC_RECEIVE_COUNT)
    return;

  Robot::get().delegate_received_feedback(FeedbackTopicReceive(topic), data);
}
