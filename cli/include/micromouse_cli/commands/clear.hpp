#pragma once

#include <cstdio>
#include <micromouse_cli/commands/command.hpp>

class ClearCommand final : public Command {
 public:
  ClearCommand(const CommandArguments args);

  static const char* name() { return "clear"; }

  bool is_done() const override { return true; }
};
