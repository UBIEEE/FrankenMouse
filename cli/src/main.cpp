#include <micromouse_cli/options/argument_parser.hpp>
#include <micromouse_cli/prompt.hpp>

#include <micromouse_cli/commands/clear.hpp>
#include <micromouse_cli/commands/exit.hpp>
#include <micromouse_cli/commands/help.hpp>
#include <micromouse_cli/commands/ti84_control.hpp>

#include <unistd.h>
#include <csignal>
#include <string>
#include <vector>

static volatile sig_atomic_t signal_received = 0;

static void register_commands(Prompt& prompt);
static bool process_command(Command* command);

int main(int argc, const char** argv) {
  signal(SIGINT, [](int) {
    write(STDOUT_FILENO, "Signal received, stopping current command...\n", 45);
    signal_received = 1;
  });

  std::vector<std::string> args{argv, argv + argc};

  Prompt prompt;
  register_commands(prompt);

  Command* command;
  bool done = false;
  while (!done && (command = prompt.readline())) {
    done = process_command(command);
    delete command;
  }

  return 0;
}

static void register_commands(Prompt& prompt) {
  prompt.register_command<HelpCommand>();
  prompt.register_command<ClearCommand>();
  prompt.register_command<ExitCommand>();
  prompt.register_command<TI84ControlCommand>();
}

static bool process_command(Command* command) {
  signal_received = 0;

  while (!signal_received && !command->is_done()) {
    CommandProcessResult result = command->process();
    if (result == CommandProcessResult::EXIT_ALL)
      return true;
    if (result != CommandProcessResult::CONTINUE)
      break;
  }

  return false;
}
