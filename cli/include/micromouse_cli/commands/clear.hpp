#pragma once

#include <cstdio>
#include <micromouse_cli/commands/command.hpp>
#include <micromouse_cli/macros.hpp>

class ClearCommand final : public Command {
 public:
  static const char* name() { return "clear"; }
  static PromptInfo prompt_info() {
    return PromptInfo{
        .usage_text = "clear",
        .short_description_text = "Clear the screen",
        .options = {},
    };
  }

  explicit ClearCommand(const CommandArguments args) : Command(args) {
    (void)fprintf(stdout, CLEAR_SCREEN());
    (void)fflush(stdout);
  }

  CommandStatus status() const override { return CommandStatus::DONE; }
};
