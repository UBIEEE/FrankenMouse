#pragma once

#include <cstdio>
#include <micromouse_cli/commands/command.hpp>
#include <micromouse_cli/ble_manager.hpp>

class RSSICommand final : public Command {
 public:
  RSSICommand(const CommandArguments args, BLEManager& ble_manager);

  static const char* name() { return "rssi"; }

  bool is_done() const override { return true; }
};
