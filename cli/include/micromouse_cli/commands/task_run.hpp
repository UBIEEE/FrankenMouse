#pragma once

#include <micromouse_cli/communication/communication_manager.hpp>
#include <micromouse_cli/commands/command.hpp>
#include <micromouse_cli/diagnostics.hpp>
#include <micromouse_cli/options/argument_parser.hpp>
#include <micromouse_cli/robot/task.hpp>
#include <micromouse_cli/robot/start_position.hpp>
#include <map>
#include <string>
#include <chrono>

class TaskRunCommand final : public Command {
  enum {
    OPTION_HELP,
    OPTION_KEEP_ALIVE,
    OPTION_START_POSITION,
  };

  static inline const std::vector<Option> s_options{
      // clang-format off
      {OPTION_HELP,           OptionName("help",       "h"),                  false, nullptr, "Show this help message"},
      {OPTION_KEEP_ALIVE,     OptionName("keep-alive", "k"),                  false, nullptr, "Keep the process alive until the task is done running"},
      {OPTION_START_POSITION, OptionName("start-position", "start-pos", "p"), true, "position", "Robot start corner, relative to maze center (left|right)"},
      // clang-format on
  };

  static constexpr const char* s_stop = "Stop";
  static constexpr const char* s_maze_search = "MazeSearch";
  static constexpr const char* s_maze_slow_solve = "MazeSlowSolve";
  static constexpr const char* s_maze_fast_solve = "MazeFastSolve";
  static constexpr const char* s_test_drive_straight_from_back_wall_to_sense_spot =
      "TestDriveStraightFromBackWallToSenseSpot";
  static constexpr const char* s_test_drive_straight_one_cell = "TestDriveStraightOneCell";
  static constexpr const char* s_test_drive_turn_right_from_sense_spot_to_sense_spot =
      "TestDriveTurnRightFromSenseSpotToSenseSpot";
  static constexpr const char* s_test_drive_turn_left_from_sense_spot_to_sense_spot =
      "TestDriveTurnLeftFromSenseSpotToSenseSpot";
  static constexpr const char* s_test_drive_turn_right_in_place = "TestDriveTurnRightInPlace";
  static constexpr const char* s_test_drive_turn_left_in_place = "TestDriveTurnLeftInPlace";
  static constexpr const char* s_test_drive_turn_180_in_place = "TestDriveTurn180InPlace";
  static constexpr const char* s_test_gyro = "TestGyro";
  static constexpr const char* s_test_drive_straight_four_cells_from_back_wall_with_vision_align =
      "TestDriveStraightFourCellsFromBackWallWithVisionAlign";
  static constexpr const char* s_idle = "Idle";

  static inline const std::map<std::string, RobotTask> s_tasks{
      {s_stop, RobotTask::STOPPED},
      {s_maze_search, RobotTask::MAZE_SEARCH},
      {s_maze_slow_solve, RobotTask::MAZE_SLOW_SOLVE},
      {s_maze_fast_solve, RobotTask::MAZE_FAST_SOLVE},
      {s_test_drive_straight_from_back_wall_to_sense_spot,
       RobotTask::TEST_DRIVE_STRAIGHT_FROM_BACK_WALL_TO_SENSE_SPOT},
      {s_test_drive_straight_one_cell, RobotTask::TEST_DRIVE_STRAIGHT_ONE_CELL},
      {s_test_drive_turn_right_from_sense_spot_to_sense_spot,
       RobotTask::TEST_DRIVE_TURN_RIGHT_FROM_SENSE_SPOT_TO_SENSE_SPOT},
      {s_test_drive_turn_left_from_sense_spot_to_sense_spot,
       RobotTask::TEST_DRIVE_TURN_LEFT_FROM_SENSE_SPOT_TO_SENSE_SPOT},
      {s_test_drive_turn_right_in_place, RobotTask::TEST_DRIVE_TURN_RIGHT_IN_PLACE},
      {s_test_drive_turn_left_in_place, RobotTask::TEST_DRIVE_TURN_LEFT_IN_PLACE},
      {s_test_drive_turn_180_in_place, RobotTask::TEST_DRIVE_TURN_180_IN_PLACE},
      {s_test_gyro, RobotTask::TEST_GYRO},
      {s_test_drive_straight_four_cells_from_back_wall_with_vision_align,
       RobotTask::TEST_DRIVE_STRAIGHT_FOUR_CELLS_FROM_BACK_WALL_WITH_VISION_ALIGN},
      {s_idle, RobotTask::IDLE},
  };

  static inline const std::vector<std::string> s_non_options{
      s_stop,
      s_maze_search,
      s_maze_slow_solve,
      s_maze_fast_solve,
      s_test_drive_straight_from_back_wall_to_sense_spot,
      s_test_drive_straight_one_cell,
      s_test_drive_turn_right_from_sense_spot_to_sense_spot,
      s_test_drive_turn_left_from_sense_spot_to_sense_spot,
      s_test_drive_turn_right_in_place,
      s_test_drive_turn_left_in_place,
      s_test_drive_turn_180_in_place,
      s_test_gyro,
      s_test_drive_straight_four_cells_from_back_wall_with_vision_align,
  };

 public:
  static const char* name() { return "task-run"; }
  static PromptInfo prompt_info() {
    return PromptInfo{.usage_text = "task-run <task> [options]",
                      .short_description_text = "Run a task on the MicroMouse",
                      .long_description_text =
                          "Run a task on the MicroMouse. By default, this command will "
                          "start running a task and then exit. Use the --keep-alive option "
                          "to keep the process alive until the task is done running. The "
                          "task will be stopped if the command is interrupted.",
                      .options = s_options,
                      .non_options_title = "Tasks",
                      .non_options = s_non_options};
  }

 private:
  ArgumentParser m_arg_parser;
  CommunicationManager& m_communication_manager;

  bool m_args_valid = false;

  bool m_is_done = true;
  bool m_keep_alive = false;
  RobotTask m_task = RobotTask::STOPPED;
  RobotStartPosition m_start_position = RobotStartPosition::LEFT_OF_GOAL;

  std::chrono::steady_clock::time_point m_start_time;

 public:
  TaskRunCommand(const CommandArguments args, CommunicationManager& communication_manager);
  ~TaskRunCommand();

  void init() override;
  void process() override;
  void end(bool interrupted) override;

  CommandStatus status() const override {
    return m_is_done ? CommandStatus::DONE : CommandStatus::CONTINUING;
  }

 private:
  // Returns true if the command should keep running.
  bool validate_args();
};
