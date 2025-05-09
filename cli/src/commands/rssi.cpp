#include <micromouse_cli/commands/rssi.hpp>

#include <micromouse_cli/diagnostics.hpp>

RSSICommand::RSSICommand(const CommandArguments args, BLEManager& ble_manager)
    : Command(args) {
  report_status(name(), "%d", ble_manager.peripheral_rssi());
}
