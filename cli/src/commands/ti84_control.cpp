#include <micromouse_cli/commands/ti84_control.hpp>

TI84ControlCommand::TI84ControlCommand(const CommandArguments args)
    : Command(args), m_arg_parser(m_args, s_options) {
  if (m_arg_parser.has_error()) {
    m_is_done = true;
    return;
  }

  const std::unordered_set<int>& options = m_arg_parser.parsed_options();

  if (options.contains(OPTION_HELP)) {
    m_is_done = true;
    help(name(), prompt_info());
    return;
  }

  // TODO: Connect
}

TI84ControlCommand::~TI84ControlCommand() {}

CommandProcessResult TI84ControlCommand::process() {
  return CommandProcessResult::DONE;
}
