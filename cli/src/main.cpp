#if WITH_BLE
#include <micromouse_cli/communication/ble_communication_manager.hpp>
#endif
#if WITH_ROS2
#include <micromouse_cli/communication/ros2_communication_manager.hpp>
#endif
#include <micromouse_cli/diagnostics.hpp>
#include <micromouse_cli/options/argument_parser.hpp>
#include <micromouse_cli/prompt.hpp>

#include <micromouse_cli/commands/clear.hpp>
#include <micromouse_cli/commands/exit.hpp>
#include <micromouse_cli/commands/rssi.hpp>
#include <micromouse_cli/commands/song_play.hpp>
#include <micromouse_cli/commands/ti84_control.hpp>

#include <unistd.h>
#include <cassert>
#include <csignal>
#include <memory>
#include <string>
#include <vector>

class Main {
  enum {
    OPTION_HELP,
    OPTION_COMMUNICATION_MODE,
#if WITH_BLE
    OPTION_PERIPHERAL_NAME,
    OPTION_ADAPTER,
    OPTION_DUMMY_PERIPHERAL,
#endif
  };

  static const inline std::vector<Option> s_options{
      // clang-format off
      {OPTION_HELP,               OptionName("help", "h"),                        false, nullptr, nullptr},
      {OPTION_COMMUNICATION_MODE, OptionName("communication-mode", "mode", "m"),  true,  nullptr, nullptr},
#if WITH_BLE
      {OPTION_PERIPHERAL_NAME,    OptionName("ble-peripheral-name", "name", "p"), true,  nullptr, nullptr},
      {OPTION_ADAPTER,            OptionName("ble-adapter", "adapter", "a"),      true,  nullptr, nullptr},
      {OPTION_DUMMY_PERIPHERAL,   OptionName("ble-dummy"),                        false, nullptr, nullptr},
#endif
      // clang-format on
  };

  static volatile inline sig_atomic_t s_signal_received = 0;

  enum class CommunicationMode {
    BLE,
    ROS2,
  } m_communication_mode =
#if WITH_BLE
      CommunicationMode::BLE;
#else
      CommunicationMode::ROS2;
#endif

#if WITH_BLE
  std::string_view m_peripheral_name = BLE_DEFAULT_PERIPHERAL_NAME;
  int m_adapter_index = BLE_DEFAULT_ADAPTER_INDEX;
  bool m_dummy_peripheral = false;
#endif

  std::unique_ptr<Prompt> m_prompt;
  std::unique_ptr<CommunicationManager> m_communication_manager;

  std::span<std::string> m_args;
  const char* m_program_name;
  ArgumentParser m_arg_parser;

  Command* m_command = nullptr;

 public:
  Main(std::span<std::string> args)
      : m_args(args), m_program_name(m_args[0].c_str()), m_arg_parser(m_args, s_options) {
    signal(SIGINT, [](int) { s_signal_received = 1; });
  }

  int run() {
    if (!validate_args())
      return 1;

#if WITH_BLE
    if (m_communication_mode == CommunicationMode::BLE) {
      m_communication_manager =
          std::make_unique<BLECommunicationManager>(m_peripheral_name, m_adapter_index, m_dummy_peripheral);
    }
#endif
#if WITH_ROS2
    if (m_communication_mode == CommunicationMode::ROS2) {
      rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
      m_communication_manager = std::make_unique<ROS2CommunicationManager>();
    }
#endif
    if (!m_communication_manager)
      return 1;  // TODO
    if (!m_communication_manager->is_initialized())
      return 1;

    m_prompt = std::make_unique<Prompt>(*m_communication_manager);

    register_commands();

    bool should_exit = false;
    while (!should_exit) {
      // Make sure it's connected.

      should_exit = process_ble_connection();
      if (should_exit)
        break;

      // Prompt for input and process the command.

      using enum Prompt::Result;

      Prompt::Result result = m_prompt->readline(&m_command);
      if (result == SIGNAL_OR_ERROR)
        return 0;  // Exit the program
      if (result == ROBOT_NOT_CONNECTED)
        continue;  // Try to reconnect

      should_exit = process_command();
      delete m_command;
    }

#if WITH_ROS2
    if (m_communication_mode == CommunicationMode::ROS2) {
      rclcpp::shutdown();
    }
#endif

    return 0;
  }

  void print_usage() {
    // clang-format off
    printf("Usage: %s [options]\n", m_args.front().c_str());
    puts("");
    puts("mm is a shell for controlling the MicroMouse.");
    puts("A connection is established to the MicroMouse when this shell is running.");
    puts("In the shell, run the `help` command to see a list of available commands.");
    puts("");
    puts("Options:");
    puts("    --help                       Show this help message");
    puts("    --communication-mode=<mode>  Set the communication mode (options: BLE (default), ROS2)");
    puts("");
    puts("BLE Options:");
    puts("    --ble-peripheral-name=<name> Set the name of the BLE peripheral to connect to");
    puts("    --ble-adapter=<id>           Set the index of the BLE adapter to use");
    puts("    --ble-dummy                  (Debug) Pretend like the connection is established");
    // clang-format on
  }

  bool validate_args() {
    const auto& options = m_arg_parser.parsed_options();
    const auto& parsed_option_values = m_arg_parser.parsed_option_values();

    if (m_arg_parser.has_error() || options.contains(OPTION_HELP)) {
      print_usage();
      return false;
    }

    if (options.contains(OPTION_COMMUNICATION_MODE)) {
      const std::string_view mode = parsed_option_values.at(OPTION_COMMUNICATION_MODE);

      const bool ble = (mode == "BLE" || mode == "ble");
      const bool ros2 = (mode == "ROS2" || mode == "ros2");

      if (ble) {
        m_communication_mode = CommunicationMode::BLE;
#if !WITH_BLE
        report_error(m_program_name, "BLE communication mode is not supported in this build");
        return false;
#endif
      } else if (ros2) {
        m_communication_mode = CommunicationMode::ROS2;
#if !WITH_ROS2
        report_error(m_program_name, "ROS2 communication mode is not supported in this build");
        return false;
#endif
      } else {
        report_error(m_program_name, "invalid communication mode: %s", mode.data());
        return false;
      }
    }

#if WITH_BLE
    if (options.contains(OPTION_PERIPHERAL_NAME)) {
      if (m_communication_mode == CommunicationMode::ROS2) {
        report_warning(m_program_name, "--peripheral-name option is ignored in ROS2 communication mode");
      } else {
        m_peripheral_name = parsed_option_values.at(OPTION_PERIPHERAL_NAME);
      }
    }
    if (options.contains(OPTION_ADAPTER)) {
      if (m_communication_mode == CommunicationMode::ROS2) {
        report_warning(m_program_name, "--adapter option is ignored in ROS2 communication mode");
      } else {
        std::string_view adapter = parsed_option_values.at(OPTION_ADAPTER);
        try {
          int adapter_index = std::stoi(adapter.data());
          if (adapter_index < 0) {
            throw std::invalid_argument("negative index");
          }
          m_adapter_index = adapter_index;
        } catch (std::invalid_argument& e) {
          report_error(m_program_name, "invalid adapter index: %s", adapter.data());
          return false;
        }
      }
    }

    if (options.contains(OPTION_DUMMY_PERIPHERAL)) {
      if (m_communication_mode == CommunicationMode::ROS2) {
        report_warning(m_program_name, "--dummy option is ignored in ROS2 communication mode");
      } else {
        m_dummy_peripheral = true;
      }
    }
#endif

    return true;
  }

  void register_commands() {
    m_prompt->register_command<ClearCommand>();
    m_prompt->register_command<ExitCommand>();
    m_prompt->register_command<TI84ControlCommand>();
    m_prompt->register_command<RSSICommand>();
    m_prompt->register_command<SongPlayCommand>();
  }

  // Returns true if the program should exit.
  bool process_ble_connection() {
    if (m_communication_manager->is_connected())
      return false;

    while (!m_communication_manager->is_connected()) {
      if (s_signal_received)
        return true;
      m_communication_manager->process_events();
    }

    return false;
  }

  // Returns true if the program should exit.
  bool process_command() {
    assert(m_command != nullptr);

    s_signal_received = 0;

    CommandStatus final_status;

    while (!s_signal_received && m_communication_manager->is_connected()) {
      if ((final_status = m_command->status()) != CommandStatus::CONTINUING)
        break;
      m_command->process();
    }

    return final_status == CommandStatus::EXIT_ALL;
  }
};

int main(int argc, const char** argv) {
  std::vector<std::string> args(argv, argv + argc);
  Main main(args);
  return main.run();
}
