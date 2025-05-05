#pragma once

#include <cstdlib>
#include <micromouse_cli/commands/command.hpp>

class ExitCommand final : public Command {
 public:
  ExitCommand(const CommandArguments args) : Command(args) {}

  static const char* name() { return "exit"; }

  CommandProcessResult process() override {
    return CommandProcessResult::EXIT_ALL;
  }
};
