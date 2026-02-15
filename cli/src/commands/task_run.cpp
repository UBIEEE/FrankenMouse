#include <micromouse_cli/commands/task_run.hpp>

TaskRunCommand::TaskRunCommand(const CommandArguments args, CommunicationManager& communication_manager)
    : Command(args), m_arg_parser(args, s_options), m_communication_manager(communication_manager) {
  m_args_valid = validate_args();
}

TaskRunCommand::~TaskRunCommand() {}

void TaskRunCommand::init() {
  if (!m_args_valid)
    return;

  m_communication_manager.write<FeedbackTopicWrite::MAIN_TASK>({m_task, m_start_position});
}

void TaskRunCommand::process() {
  // TODO: Wait a second for robot to start running the task
  m_is_done = (m_communication_manager.get_value<FeedbackTopicNotify::MAIN_TASK>() == RobotTask::STOPPED);
}

void TaskRunCommand::end(bool interrupted) {
  if (interrupted && m_keep_alive && m_task != RobotTask::STOPPED) {
    m_communication_manager.write<FeedbackTopicWrite::MAIN_TASK>({RobotTask::STOPPED, m_start_position});
  }
}

bool TaskRunCommand::validate_args() {
  if (m_arg_parser.has_error())
    return false;

  const std::unordered_set<int>& options = m_arg_parser.parsed_options();
  const std::__1::unordered_map<int, std::__1::string_view>& parsed_option_values =
      m_arg_parser.parsed_option_values();
  std::span<const std::string_view> non_option_args = m_arg_parser.non_option_args();

  // Help

  if (options.contains(OPTION_HELP)) {
    help(name(), prompt_info(), stdout);
    return false;
  }

  // Task

  if (non_option_args.empty()) {
    report_error(name(), "missing task argument");
    return false;
  } else if (non_option_args.size() > 1) {
    report_error(name(), "too many arguments");
    return false;
  }

  std::string task_name(non_option_args[0]);

  auto it = s_tasks.find(task_name);
  if (it == s_tasks.end()) {
    report_error(name(), "unknown task: %s", non_option_args[0].data());
    return false;
  }

  m_task = it->second;

  // Start position

  if (options.contains(OPTION_START_POSITION)) {
    const std::string_view pos = parsed_option_values.at(OPTION_START_POSITION);
    if (pos == "LEFT" || pos == "left") {
      m_start_position = RobotStartPosition::LEFT_OF_GOAL;
    } else if (pos == "RIGHT" || pos == "right") {
      m_start_position = RobotStartPosition::RIGHT_OF_GOAL;
    } else {
      report_error(name(), "invalid start position: %s", pos.data());
      return false;
    }
  }

  // Keep alive

  m_keep_alive = options.contains(OPTION_KEEP_ALIVE);
  m_is_done = (m_keep_alive == false);

  return true;
}
