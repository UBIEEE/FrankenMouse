#include <micromouse_cli/commands/drive_pid_set.hpp>

DrivePIDSetCommand::DrivePIDSetCommand(const CommandArguments args,
                                       CommunicationManager& communication_manager)
    : Command(args), m_arg_parser(args, s_options), m_communication_manager(communication_manager) {
  m_args_valid = validate_args();
}

DrivePIDSetCommand::~DrivePIDSetCommand() {}

void DrivePIDSetCommand::init() {
  if (!m_args_valid)
    return;

  m_communication_manager.write<FeedbackTopicWrite::DRIVE_PID>(m_pid);
}

bool DrivePIDSetCommand::validate_args() {
  if (m_arg_parser.has_error())
    return false;

  const std::unordered_set<int>& options = m_arg_parser.parsed_options();
  std::span<const std::string_view> non_option_args = m_arg_parser.non_option_args();

  // Help

  if (options.contains(OPTION_HELP)) {
    help(name(), prompt_info(), stdout);
    return false;
  }

  const size_t num_args = non_option_args.size();

  // Component

  if (num_args < 1) {
    report_error(name(), "missing component");
    return false;
  }

  std::string component_name(non_option_args[0]);
  auto component_it = s_components.find(component_name);
  if (component_it == s_components.end()) {
    report_error(name(), "unknown component: %s, expected one of: linear, angular", component_name.c_str());
    return false;
  }

  m_component = component_it->second;

  if (num_args < 4) {
    report_error(name(), "missing one or more PID values");
    return false;
  }

  size_t i = 0;
  std::string value;
  try {
    for (i = 0; i < 3; i++) {
      value = non_option_args[1 + i];
      size_t offset = m_component == Component::LINEAR ? 0 : 3;
      m_pid[offset + i] = std::stof(value);
    }
  } catch (std::invalid_argument& e) {
    report_error(name(), "invalid k%c value: %s", "PID"[i], value.c_str());
    return false;
  }

  return true;
}
