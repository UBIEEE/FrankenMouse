#pragma once

#include <cstdio>
#include <micromouse_cli/commands/command.hpp>

class ClearCommand final : public Command {
  COMMAND_NAME_AND_PROMPT_INFO("clear", "clear", "Clear the screen", {})
  COMMAND_INSTANT();

 public:
  ClearCommand(const CommandArguments args) : Command(args) {
    // Clear terminal and set cursor to 1,1
    (void)fprintf(stdout, "\033[2J\033[;H");
    (void)fflush(stdout);
  }
};
