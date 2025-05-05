#pragma once

#include <cstdio>
#include <micromouse_cli/commands/command.hpp>

class ClearCommand final : public Command {
 public:
  ClearCommand(const CommandArguments args) : Command(args) {}

  static const char* name() { return "clear"; }

  CommandProcessResult process() override {
    // Clear terminal and set cursor to 1,1
    (void)fprintf(stdout, "\033[2J\033[;H");
    (void)fflush(stdout);
    return CommandProcessResult::DONE;
  }
};
