#include <micromouse_cli/commands/help.hpp>

#include <cstdio>

CommandProcessResult HelpCommand::process() {
  puts("help");  // TODO
  return CommandProcessResult::DONE;
}
