#pragma once

#include <cstdlib>
#include <micromouse_cli/commands/command.hpp>

class ExitCommand final : public Command {
  COMMAND_NAME_AND_PROMPT_INFO("exit",
                               "exit",
                               "End the connection and exit the shell",
                               {})

 public:
  ExitCommand(const CommandArguments args) : Command(args) {}

  CommandProcessResult process() override {
    return CommandProcessResult::EXIT_ALL;
  }
};
