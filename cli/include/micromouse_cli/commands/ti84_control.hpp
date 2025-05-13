#pragma once

#include <micromouse_cli/commands/command.hpp>
#include <micromouse_cli/options/argument_parser.hpp>
#include <micromouse_cli/drive/chassis_speeds.hpp>

/**
 * @brief This command is used to control the MicroMouse using a TI-84 Plus CE
 *        calculator. This process reads data messages sent from the calculator
 *        over serial, then sends instructions to the MicroMouse over BLE.
 */
class TI84ControlCommand final : public Command {
  enum {
    OPTION_HELP,
    OPTION_BAUDRATE,
  };

  static inline const std::vector<Option> s_options{
      // clang-format off
      {OPTION_HELP,     OptionName("help", "h"),     false, nullptr, "Show this help message"},
      {OPTION_BAUDRATE, OptionName("baudrate", "b"), true,  "rate",  "Baudrate to use (default: 9600)"},
      // clang-format on
  };

 public:
  COMMAND_NAME("ti84-control")

  static PromptInfo prompt_info() {
    return PromptInfo{
        .options = s_options,
        .usage_text = "ti84-control [port] [options]",
        .short_description_text = "Control the MicroMouse using a TI-84 Plus CE",
        .can_accept_file_paths = true,
    };
  }

 private:
  ArgumentParser m_arg_parser;
  bool m_connected = false;

  std::string m_port;
  int m_baudrate = 9600;

  int m_serial_fd = -1;

 public:
  TI84ControlCommand(const CommandArguments args);
  ~TI84ControlCommand();

  bool is_done() const override { return !m_connected; }

  CommandProcessResult process() override;

 private:
  // Returns true if the command should keep running.
  bool validate_args();

  bool open_serial_port();
  void close_serial_port();

  bool configure_serial_port();

  static bool is_standard_baudrate(int baudrate);

 private:
  static bool validate_control_message(uint8_t data);

  static drive::ChassisSpeeds to_chassis_speeds(uint8_t data);
  static void display_control_message(uint8_t data, const drive::ChassisSpeeds& speeds);
};
