#pragma once

#include <micromouse_cli/communication/communication_manager.hpp>
#include <micromouse_cli/commands/command.hpp>
#include <micromouse_cli/diagnostics.hpp>
#include <micromouse_cli/options/argument_parser.hpp>
#include <micromouse_cli/print_utils.hpp>
#include <cstdio>
#include <map>
#include <string>

class DrivePIDSetCommand final : public Command {
  enum {
    OPTION_HELP,
  };

  static inline const std::vector<Option> s_options{
      // clang-format off
      {OPTION_HELP,           OptionName("help", "h"), false, nullptr, "Show this help message"},
      // clang-format on
  };

  enum Component {
    LINEAR,
    ANGULAR,
  };

  static inline const std::map<std::string, Component> s_components{
      {"linear", Component::LINEAR},
      {"angular", Component::ANGULAR},
  };

  static inline const std::vector<std::string> s_non_options{
      "linear",
      "angular",
  };

 public:
  static const char* name() { return "drive-pid-set"; }
  static PromptInfo prompt_info() {
    return PromptInfo{
        .usage_text = "drive-pid-set <component> <kP> <kI> <kD> [options]",
        .short_description_text = "Set the linear or angular drive PID values on the MicroMouse",
        .options = s_options,
        .non_options_title = "Components",
        .non_options = s_non_options};
  }

 private:
  ArgumentParser m_arg_parser;
  CommunicationManager& m_communication_manager;

  Component m_component;

  bool m_args_valid = false;
  float m_pid[6] = {0};  // linear (kP, kI, kD), angular (kP, kI, kD)

 public:
  DrivePIDSetCommand(const CommandArguments args, CommunicationManager& communication_manager);
  ~DrivePIDSetCommand();

  void init() override;

  CommandStatus status() const override { return CommandStatus::DONE; }

 private:
  // Returns true if the command should keep running.
  bool validate_args();
};
