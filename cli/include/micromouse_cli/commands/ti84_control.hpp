#pragma once

#include <micromouse_cli/commands/command.hpp>
#include <micromouse_cli/options/argument_parser.hpp>

/**
 * @brief This command is used to control the MicroMouse using a TI-84 Plus CE
 *        calculator. This process reads data messages sent from the calculator
 *        over serial, then sends instructions to the MicroMouse over BLE.
 */
class TI84ControlCommand final : public Command {
  enum {
    OPTION_HELP,
    OPTION_PORT,
  };

  static inline const std::vector<Option> s_options{
      // clang-format off
      {OPTION_HELP, OptionName("help", "h"), false, nullptr, "Show this help message"},
      {OPTION_PORT, OptionName("port", "p"), true, "number", "Specify the serial port to connect to"},
      // clang-format on
  };

  COMMAND_NAME_AND_PROMPT_INFO("ti84-control",
                               "ti84-control [options]",
                               "Control the MicroMouse using a TI-84 Plus CE",
                               s_options)

 private:
  ArgumentParser m_arg_parser;
  bool m_is_done = false;

 public:
  TI84ControlCommand(const CommandArguments args);
  ~TI84ControlCommand();

  bool is_done() const override { return m_is_done; }

  CommandProcessResult process() override;
};
