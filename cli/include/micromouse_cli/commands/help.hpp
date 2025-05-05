#pragma once

#include <micromouse_cli/commands/command.hpp>

class HelpCommand final : public Command {
  static inline std::vector<const char*> s_options {
    "eric",
  };

 public:
  HelpCommand(const CommandArguments args) : Command(args) {}

  static const char* name() { return "help"; }
  static std::span<const char*> options() { return s_options; }

  CommandProcessResult process() override;
};
