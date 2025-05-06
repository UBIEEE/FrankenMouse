#pragma once

#include <micromouse_cli/commands/command.hpp>
#include <micromouse_cli/options/argument_parser.hpp>

/**
 * @brief This command is used to control the MicroMouse using a TI-84 Plus CE
 *        calculator. This process reads data messages sent from the calculator
 *        over serial, then sends instructions to the MicroMouse over BLE.
 */
class TI84ControlCommand final : public Command {
  static const inline std::vector<const char*> s_completion_options{
      "--help",
      "--port=",
  };

  enum {
    OPTION_HELP,
    OPTION_PORT,
  };

  static const inline std::vector<Option> s_options{
      {OPTION_HELP, OptionName("help", "h"), false},
      {OPTION_PORT, OptionName("port", "p"), true},
  };

  ArgumentParser m_arg_parser;
  bool m_is_done = false;

 public:
  TI84ControlCommand(const CommandArguments args);
  ~TI84ControlCommand();

  static const char* name() { return "ti84-control"; }
  static std::span<const char* const> options() { return s_completion_options; }

  bool is_done() const override { return m_is_done; }

  CommandProcessResult process() override;

 private:
  void help();
};
