#pragma once

#include <cstdio>
#include <micromouse_cli/ble_manager.hpp>
#include <micromouse_cli/commands/command.hpp>
#include <micromouse_cli/diagnostics.hpp>

class RSSICommand final : public Command {
  COMMAND_NAME_AND_PROMPT_INFO("rssi",
                               "rssi",
                               "Show the RSSI value of the current connection",
                               {})
  COMMAND_INSTANT();

 public:
  RSSICommand(const CommandArguments args, BLEManager& ble_manager)
      : Command(args) {
    report_status(name(), "%d", ble_manager.peripheral_rssi());
  }
};
