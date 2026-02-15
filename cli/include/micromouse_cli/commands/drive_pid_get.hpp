#pragma once

#include <micromouse_cli/communication/communication_manager.hpp>
#include <micromouse_cli/commands/command.hpp>
#include <micromouse_cli/diagnostics.hpp>
#include <micromouse_cli/options/argument_parser.hpp>
#include <micromouse_cli/print_utils.hpp>
#include <cstdio>
#include <map>
#include <string>

class DrivePIDGetCommand final : public Command {
  enum {
    OPTION_HELP,
  };

  static inline const std::vector<Option> s_options{
      // clang-format off
      {OPTION_HELP,           OptionName("help", "h"), false, nullptr, "Show this help message"},
      // clang-format on
  };

 public:
  static const char* name() { return "drive-pid-get"; }
  static PromptInfo prompt_info() {
    return PromptInfo{
        .usage_text = "drive-pid-get [options]",
        .short_description_text = "Get the current linear and angular drive PID values from the MicroMouse",
        .options = s_options,
    };
  }

 private:
  ArgumentParser m_arg_parser;
  CommunicationManager& m_communication_manager;

  bool m_args_valid = false;

 public:
  DrivePIDGetCommand(const CommandArguments args, CommunicationManager& communication_manager);
  ~DrivePIDGetCommand();

  void init() override;

  CommandStatus status() const override { return CommandStatus::DONE; }

 private:
  // Returns true if the command should keep running.
  bool validate_args();
};
