#pragma once

#include <micromouse_cli/communication/communication_manager.hpp>
#include <micromouse_cli/commands/command.hpp>
#include <micromouse_cli/diagnostics.hpp>
#include <micromouse_cli/options/argument_parser.hpp>
#include <micromouse_cli/robot/task.hpp>
#include <micromouse_cli/robot/start_position.hpp>
#include <cstdio>
#include <map>
#include <string>

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

  static inline const std::map<std::string, RobotTask> s_tasks{
      {s_stop, RobotTask::STOPPED},
  };

  static inline const std::vector<std::string> s_non_options{
      s_stop,
  };

 public:
  static const char* name() { return "task-run"; }
  static PromptInfo prompt_info() {
    return PromptInfo{.usage_text = "task-run <task> [options]",
                      .short_description_text = "Play a song on the MicroMouse",
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

  bool m_is_done = true;
  bool m_keep_alive = false;
  RobotTask m_task = RobotTask::STOPPED;
  RobotStartPosition m_start_position = RobotStartPosition::LEFT_OF_GOAL;

 public:
  TaskRunCommand(const CommandArguments args, CommunicationManager& communication_manager);
  ~TaskRunCommand();

  void process() override;

  CommandStatus status() const override { return m_is_done ? CommandStatus::DONE : CommandStatus::CONTINUING; }

 private:
  // Returns true if the command should keep running.
  bool validate_args();
};
