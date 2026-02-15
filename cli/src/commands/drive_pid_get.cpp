#include <micromouse_cli/commands/drive_pid_get.hpp>

DrivePIDGetCommand::DrivePIDGetCommand(const CommandArguments args,
                                       CommunicationManager& communication_manager)
    : Command(args), m_arg_parser(args, s_options), m_communication_manager(communication_manager) {
  m_args_valid = validate_args();
}

DrivePIDGetCommand::~DrivePIDGetCommand() {}

void DrivePIDGetCommand::init() {
  if (!m_args_valid)
    return;

  const float* pid = m_communication_manager.drive_data().pid_data;
  fprintf(stdout, "linear:\n\tkP: %f\n\tkI: %f\n\tkD: %f\n", pid[0], pid[1], pid[2]);
  fprintf(stdout, "angular:\n\tkP: %f\n\tkI: %f\n\tkD: %f\n", pid[3], pid[4], pid[5]);
}

bool DrivePIDGetCommand::validate_args() {
  if (m_arg_parser.has_error())
    return false;

  const std::unordered_set<int>& options = m_arg_parser.parsed_options();

  // Help

  if (options.contains(OPTION_HELP)) {
    help(name(), prompt_info(), stdout);
    return false;
  }

  return true;
}
