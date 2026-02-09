#pragma once

#include <cstdio>
#include <micromouse_cli/ble_manager.hpp>
#include <micromouse_cli/commands/command.hpp>
#include <micromouse_cli/diagnostics.hpp>

class RSSICommand final : public Command {
 public:
  static const char* name() { return "rssi"; }
  static PromptInfo prompt_info() {
    return PromptInfo{
        .usage_text = "rssi",
        .short_description_text = "Show the RSSI value of the current connection",
        .options = {},
    };
  }

  // Instant
  bool is_done() const override { return true; }

 public:
  RSSICommand(const CommandArguments args, BLEManager& ble_manager)
      : Command(args) {
    report_status(name(), "%d", ble_manager.peripheral_rssi());
  }
};
