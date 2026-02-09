#pragma once

#include <cstdlib>
#include <micromouse_cli/commands/command.hpp>

class ExitCommand final : public Command {
 public:
  static const char* name() { return "exit"; }
  static PromptInfo prompt_info() {
    return PromptInfo{
        .usage_text = "exit",
        .short_description_text = "End the connection and exit the shell",
        .options = {},
    };
  }

  explicit ExitCommand(const CommandArguments args) : Command(args) {}

  CommandStatus status() const override { return CommandStatus::EXIT_ALL; }
};
