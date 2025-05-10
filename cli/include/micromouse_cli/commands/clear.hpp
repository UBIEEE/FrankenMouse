#pragma once

#include <cstdio>
#include <micromouse_cli/commands/command.hpp>
#include <micromouse_cli/macros.hpp>

class ClearCommand final : public Command {
  COMMAND_NAME_AND_PROMPT_INFO("clear", "clear", "Clear the screen", {})
  COMMAND_INSTANT();

 public:
  ClearCommand(const CommandArguments args) : Command(args) {
    (void)fprintf(stdout, CLEAR_SCREEN());
    (void)fflush(stdout);
  }
};
