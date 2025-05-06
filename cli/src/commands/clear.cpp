#include <micromouse_cli/commands/clear.hpp>

ClearCommand::ClearCommand(const CommandArguments args) : Command(args) {
  // Clear terminal and set cursor to 1,1
  (void)fprintf(stdout, "\033[2J\033[;H");
  (void)fflush(stdout);
}
