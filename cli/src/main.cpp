#include <cstdio>
#include <span>

#include <micromouse_cli/prompt.hpp>
#include <thread>

#include <unistd.h>
#include <csignal>

static volatile sig_atomic_t signal_received = 0;

enum {
  COMMAND_EXIT = 1,
  COMMAND_HELP,
  COMMAND_CLEAR,
};

int main(int argc, const char** argv) {
  signal(SIGINT, [](int) {
    write(STDOUT_FILENO, "Signal received, stopping current command...\n", 45);
    signal_received = 1;
  });

  std::vector<std::string> args{argv, argv + argc};
  (void)args;  // TODO

  Prompt prompt;

  std::vector<const char*> help_options{"eric"};
  prompt.register_command(COMMAND_HELP, "help", help_options);
  prompt.register_command(COMMAND_EXIT, "exit");
  prompt.register_command(COMMAND_CLEAR, "clear");

  std::optional<Prompt::CommandInvocation> input;
  while ((input = prompt.readline())) {
    signal_received = 0;

    if (input->command == COMMAND_EXIT)
      break;

    switch (input->command) {
      case COMMAND_HELP:
        puts("help");  // TODO
        break;
      case COMMAND_CLEAR:
        printf("\033[2J\033[;H");  // Clear terminal and set cursor to 1,1
        fflush(stdout);
        break;
      case Prompt::COMMAND_UNKNOWN:
        fprintf(stderr, "%s: unknown command: '%s'\n", argv[0],
                input->args[0].c_str());
        break;
    }
  }

  return 0;
}
