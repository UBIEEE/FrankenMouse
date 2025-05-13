#include <micromouse_cli/ble_manager.hpp>
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
    OPTION_PERIPH_NAME,
    OPTION_ADAPTER,
    OPTION_DUMMY_PERIPHERAL,
  };

  static const inline std::vector<Option> s_options{
      // clang-format off
      {OPTION_HELP,             OptionName("help", "h"),                    false},
      {OPTION_PERIPH_NAME,      OptionName("peripheral-name", "name", "p"), true},
      {OPTION_ADAPTER,          OptionName("adapter", "a"),                 true},
      {OPTION_DUMMY_PERIPHERAL, OptionName("dummy"),                        false},
      // clang-format on
  };

  static volatile inline sig_atomic_t s_signal_received = 0;

  std::string_view m_periph_name = DEFAULT_PERIPHERAL_NAME;
  int m_adapter_idx = DEFAULT_ADAPTER_INDEX;
  bool m_dummy_peripheral = false;

  std::unique_ptr<Prompt> m_prompt;
  std::unique_ptr<BLEManager> m_ble_manager;

  std::span<std::string> m_args;
  const char* m_program_name;
  ArgumentParser m_arg_parser;

  Command* m_command = nullptr;

 public:
  Main(std::span<std::string> args)
      : m_args(args),
        m_program_name(m_args[0].c_str()),
        m_arg_parser(m_args, s_options) {
    signal(SIGINT, [](int) { s_signal_received = 1; });
  }

  int run() {
    if (!validate_args())
      return 1;

    m_ble_manager = std::make_unique<BLEManager>(m_periph_name, m_adapter_idx,
                                                 m_dummy_peripheral);
    if (!m_ble_manager->is_initialized())
      return 1;

    m_prompt = std::make_unique<Prompt>(*m_ble_manager);

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
      if (result == BLE_NOT_CONNECTED)
        continue;  // Try to reconnect

      should_exit = process_command();
      delete m_command;
    }

    return 0;
  }

  void print_usage() {
    // clang-format off
    printf("Usage: %s [options]\n", m_args.front().c_str());
    puts("");
    puts("mm is a shell for controlling the MicroMouse wirelessly.");
    puts("When the shell is running, a BLE connection is established to the MicroMouse.");
    puts("In the shell, run the `help` command to see a list of available commands.");
    puts("");
    puts("Options:");
    puts("    --help          Show this help message");
    puts("    --name=<name>   Specify the name of the BLE peripheral to connect to");
    puts("    --adapter=<id>  Specify the index of the BLE adapter to use");
    puts("    --dummy         (Debug) Pretend like the connection is established");
    // clang-format on
  }

  bool validate_args() {
    const auto& options = m_arg_parser.parsed_options();
    const auto& parsed_option_values = m_arg_parser.parsed_option_values();

    if (m_arg_parser.has_error() || options.contains(OPTION_HELP)) {
      print_usage();
      return false;
    }

    if (options.contains(OPTION_PERIPH_NAME)) {
      m_periph_name = parsed_option_values.at(OPTION_PERIPH_NAME);
    }
    if (options.contains(OPTION_ADAPTER)) {
      std::string_view adapter = parsed_option_values.at(OPTION_ADAPTER);
      try {
        m_adapter_idx = std::stoi(adapter.data());
        if (m_adapter_idx < 0)
          throw std::invalid_argument("negative index");
      } catch (std::invalid_argument& e) {
        report_error(m_program_name, "invalid adapter index: %s",
                     adapter.data());
        return false;
      }
    }

    if (options.contains(OPTION_DUMMY_PERIPHERAL)) {
      m_dummy_peripheral = true;
    }

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
    if (m_ble_manager->is_connected())
      return false;

    while (!m_ble_manager->is_connected()) {
      if (s_signal_received)
        return true;
      m_ble_manager->process_events();
    }

    return false;
  }

  // Returns true if the program should exit.
  bool process_command() {
    assert(m_command != nullptr);

    s_signal_received = 0;

    while (!s_signal_received && m_ble_manager->is_connected() &&
           !m_command->is_done()) {
      CommandProcessResult result = m_command->process();
      if (result == CommandProcessResult::EXIT_ALL)
        return true;
      if (result != CommandProcessResult::CONTINUE)
        break;
    }

    return false;
  }
};

int main(int argc, const char** argv) {
  std::vector<std::string> args(argv, argv + argc);
  Main main(args);
  return main.run();
}
