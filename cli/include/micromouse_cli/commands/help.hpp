#pragma once

#include <micromouse_cli/commands/command.hpp>

class HelpCommand final : public Command {
 public:
  HelpCommand(const CommandArguments args);

  static const char* name() { return "help"; }

  bool is_done() const override { return true; }
};
