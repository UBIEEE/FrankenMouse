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
    help();
    return;
  }

  // TODO: Connect
}

TI84ControlCommand::~TI84ControlCommand() {}

CommandProcessResult TI84ControlCommand::process() {
  return CommandProcessResult::DONE;
}

void TI84ControlCommand::help() {
  // clang-format off
  printf("usage: %s [options]\n", name());
  puts("");
  puts("This command is used to control the MicroMouse using a TI-84 Plus CE");
  puts("");
  puts("options:");
  puts("  --help          Show this help message");
  puts("  --port=<port>   Specify the serial port to connect to");
  // clang-format on
}
