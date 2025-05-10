#include <micromouse_cli/commands/ti84_control.hpp>

#include <micromouse_cli/diagnostics.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <filesystem>

union ControlMessage {
  uint8_t data;
  struct {
    uint8_t forward : 1;
    uint8_t backward : 1;
    uint8_t turn_cw : 1;
    uint8_t turn_ccw : 1;
    uint8_t speed : 4;
  };
};

TI84ControlCommand::TI84ControlCommand(const CommandArguments args)
    : Command(args), m_arg_parser(m_args, s_options) {
  if (!validate_args())
    return;

  if (!open_serial_port())
    return;

  if (!configure_serial_port()) {
    close_serial_port();
    return;
  }

  m_connected = true;
}

TI84ControlCommand::~TI84ControlCommand() {
  close_serial_port();
}

CommandProcessResult TI84ControlCommand::process() {
  if (!m_connected)
    return CommandProcessResult::DONE;

  char buf[1];

  int bytes_read = read(m_serial_fd, &buf, sizeof(buf));
  if (bytes_read < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // No data available, continue waiting
      return CommandProcessResult::CONTINUE;
    }

    report_error(name(), "failed to read from serial port: %s",
                 strerror(errno));
    return CommandProcessResult::DONE;

  } else if (bytes_read == 0) {
    report_warning(name(), "read 0 bytes?");
    return CommandProcessResult::CONTINUE;
  }

  uint8_t data = static_cast<uint8_t>(buf[0]);
  display_control_message(data);

  // TODO: Convert to ChassisSpeeds and send to the MicroMouse

  return CommandProcessResult::CONTINUE;
}

bool TI84ControlCommand::validate_args() {
  if (m_arg_parser.has_error())
    return false;

  const std::unordered_set<int>& options = m_arg_parser.parsed_options();
  std::span<const std::string_view> non_option_args =
      m_arg_parser.non_option_args();

  // Help

  if (options.contains(OPTION_HELP)) {
    help(name(), prompt_info());
    return false;
  }

  // Port

  if (non_option_args.empty()) {
    report_error(name(), "missing port argument");
    return false;
  } else if (non_option_args.size() > 1) {
    report_error(name(), "too many arguments");
    return false;
  }

  m_port = non_option_args[0];

  // Baudrate

  if (options.contains(OPTION_BAUDRATE)) {
    std::string_view baudrate_str =
        m_arg_parser.parsed_option_values().at(OPTION_BAUDRATE);

    try {
      m_baudrate = std::stoi(baudrate_str.data());
    } catch (std::invalid_argument& e) {
      report_error(name(), "invalid baudrate: %s", baudrate_str.data());
      return false;
    }

    if (!is_standard_baudrate(m_baudrate)) {
      report_error(name(), "non-standard baudrate: %d", m_baudrate);
      return false;
    }
  }
  return true;
}

bool TI84ControlCommand::open_serial_port() {
  // Open the port for reading only, don't assign controlling terminal, and
  // enable non-blocking mode
  m_serial_fd = open(m_port.c_str(), O_RDONLY /*O_RDWR*/ | O_NOCTTY | O_NDELAY);
  if (m_serial_fd < 0) {
    report_error(name(), "failed to open port %s: %s", m_port.c_str(),
                 strerror(errno));
    return false;
  }

  report_status(name(), "opened serial port %s", m_port.c_str());
  return true;
}

void TI84ControlCommand::close_serial_port() {
  if (m_serial_fd < 0)
    return;

  report_status(name(), "closing serial port %s", m_port.c_str());
  close(m_serial_fd);
  m_serial_fd = -1;
}

bool TI84ControlCommand::configure_serial_port() {
  struct termios tty;
  if (tcgetattr(m_serial_fd, &tty) != 0) {
    report_error(name(), "failed to get serial port attributes: %s",
                 strerror(errno));
    return false;
  }

  // Set Baud Rate (Input and Output)
  if (cfsetispeed(&tty, m_baudrate) < 0 || cfsetospeed(&tty, m_baudrate) < 0) {
    report_error(name(), "failed to set baud rate: %s", strerror(errno));
    return false;
  }

  // Control Modes
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  // Local Modes
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;

  // Input Modes
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  // Output Modes
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VMIN] = 1;   // Wait for at least 1 character
  tty.c_cc[VTIME] = 0;  // No timeout (wait indefinitely)

  if (tcsetattr(m_serial_fd, TCSANOW, &tty) != 0) {
    report_error(name(), "failed to set serial port attributes: %s",
                 strerror(errno));
    return false;
  }

  report_status(name(), "serial port configured: %d baud, 8N1", m_baudrate);
  return true;
}

bool TI84ControlCommand::is_standard_baudrate(int baudrate) {
  switch (baudrate) {
    case B50:
    case B75:
    case B110:
    case B134:
    case B150:
    case B200:
    case B300:
    case B600:
    case B1200:
    case B1800:
    case B2400:
    case B4800:
    case B9600:
    case B19200:
    case B38400:
      return true;
  }
  return false;
}

void TI84ControlCommand::display_control_message(uint8_t data) {
  ControlMessage msg{.data = data};

  const char* linear_str = "X";
  const char* angular_str = "X";

  if (msg.forward) {
    linear_str = "↑";
  } else if (msg.backward) {
    linear_str = "↓";
  }
  if (msg.turn_cw) {
    angular_str = "↻";
  } else if (msg.turn_ccw) {
    angular_str = "↺";
  }

  printf("\rRaw Byte: 0x%02x Speed: %d Linear: %s Angular: %s", data, msg.speed,
         linear_str, angular_str);
  fflush(stdout);
}

// ChassisSpeeds TI84ControlCommand::to_chassis_speeds(uint8_t data) {}
