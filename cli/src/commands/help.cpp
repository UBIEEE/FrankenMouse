#include <micromouse_cli/commands/help.hpp>

#include <cstdio>

HelpCommand::HelpCommand(const CommandArguments args) : Command(args) {
  // clang-format off
  puts("What you see here is a shell for controlling the MicroMouse.");
  puts("");
  puts("When this shell is running, a BLE connection is established to the MicroMouse.");
  puts("");
  puts("There are a number of commands for controlling and reading data from the MicroMouse.");
  puts("To display detailed usage information for a command, enter the following:");
  puts("  <command> --help");
  puts("");
  puts("General Commands:");
  puts("  help            Show this help message");
  puts("  exit            End the connection and exit the shell");
  puts("  clear           Clear the screen");
  puts("");
  puts("Control Commands:");
  puts("  ti84-control    Control the MicroMouse using a TI-84 Plus CE calculator");
  puts("");
  puts("Diagnostic Commands:");
  puts("");
  puts("Connection Commands:");
  puts("  rssi           Show the RSSI value of the current connection");
  // clang-format on
}
