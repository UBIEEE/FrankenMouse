#include <micromouse_cli/commands/ti84_control.hpp>

#include <micromouse_cli/diagnostics.hpp>
#include <micromouse_cli/macros.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cassert>
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

  ControlMessage() : data(0) {}
  /*implicit*/ ControlMessage(uint8_t data) : data(data) {}
  ControlMessage(uint8_t directions, uint8_t speed)
      : data((directions & 0xF) | (speed << 4)) {}

  /*implicit*/ operator uint8_t() const { return data; }
};

TI84ControlCommand::TI84ControlCommand(const CommandArguments args, BLEManager& ble_manager)
    : Command(args), m_arg_parser(m_args, s_options), m_ble_manager(ble_manager) {
  if (!validate_args())
    return;

  if (!open_serial_port())
    return;

  if (!configure_serial_port()) {
    close_serial_port();
    return;
  }

  m_ble_manager.write<BLETopicWrite::MAIN_TASK>(Task::MANUAL_CHASSIS_SPEEDS);
  m_connected = true;
}

TI84ControlCommand::~TI84ControlCommand() {
  close_serial_port();

  if (m_connected) {
    m_ble_manager.write<BLETopicWrite::MAIN_TASK>(Task::STOPPED);
  }
}

CommandProcessResult TI84ControlCommand::process() {
  if (!m_connected)
    return CommandProcessResult::DONE;

  uint8_t data;

  int bytes_read = read(m_serial_fd, &data, sizeof(data));
  if (bytes_read < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // No data available, continue waiting
      return CommandProcessResult::CONTINUE;
    }

    report_error(name(), "failed to read from serial port: %s",
                 strerror(errno));
    return CommandProcessResult::DONE;

  } else if (bytes_read != sizeof(data)) {
    report_warning(name(), "read %d bytes?", bytes_read);
    return CommandProcessResult::CONTINUE;
  }

  if (!validate_control_message(data)) {
    data = ControlMessage(0, 1);  // Stop
  }

  drive::ChassisSpeeds speeds = to_chassis_speeds(data);
  display_control_message(data, speeds);

  m_ble_manager.write<BLETopicWrite::DRIVE_CHASSIS_SPEEDS>(speeds);

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
    help(name(), prompt_info(), stdout);
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
  (void)close(m_serial_fd);
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

bool TI84ControlCommand::validate_control_message(uint8_t data) {
  ControlMessage msg = data;

  bool invalid = false;
  const char* reason = nullptr;

  if (msg.forward && msg.backward) {
    invalid = true;
    reason = "forward and backward are mutually exclusive";
  }
  if (msg.turn_cw && msg.turn_ccw) {
    invalid = true;
    reason = "turn_cw and turn_ccw are mutually exclusive";
  }
  if (msg.speed < 1 || msg.speed > 5) {
    invalid = true;
    reason = "speed must be between 1 and 5";
  }

  if (invalid) {
    report_warning(name(), "invalid control message: 0x%02x (%s)", data,
                   reason);
    return false;
  }

  return true;
}

void TI84ControlCommand::display_control_message(
    uint8_t data,
    const drive::ChassisSpeeds& speeds) {
  ControlMessage msg = data;

  (void)printf(CLEAR_LINE());

  (void)printf("Raw Byte: " BOLD("0x%02x "), data);
  (void)printf("Speed: " BOLD("%d "), msg.speed);

  const char* motion_str = "X";

  if (msg.forward) {
    motion_str = "↑";
    if (msg.turn_cw)
      motion_str = "↗";
    else if (msg.turn_ccw)
      motion_str = "↖";
  } else if (msg.backward) {
    motion_str = "↓";
    if (msg.turn_cw)
      motion_str = "↘";
    else if (msg.turn_ccw)
      motion_str = "↙";
  } else if (msg.turn_cw) {
    motion_str = "↻";
  } else if (msg.turn_ccw) {
    motion_str = "↺";
  }

  (void)printf("Motion: " BOLD("%s "), motion_str);

  (void)printf("Linear: " BOLD("%.1f mm/s "), speeds.linear_velocity_mmps);
  (void)printf("Angular: " BOLD("%.1f deg/s "), speeds.angular_velocity_dps);

  (void)fflush(stdout);
}

static const float s_linear_speed_presets_mmps[5] = {
    50.f, 100.f, 150.f, 200.f, 250.f,
};

static const float s_angular_speed_presets_dps[5] = {
    45.f, 90.f, 135.f, 180.f, 360.f,
};

drive::ChassisSpeeds TI84ControlCommand::to_chassis_speeds(uint8_t data) {
  ControlMessage msg = data;

  int linear_coeff = 0;
  if (msg.forward)
    linear_coeff = +1;
  else if (msg.backward)
    linear_coeff = -1;

  int angular_coeff = 0;
  if (msg.turn_ccw)
    angular_coeff = +1;
  else if (msg.turn_cw)
    angular_coeff = -1;

  assert(msg.speed > 0 && msg.speed < 6);

  drive::ChassisSpeeds speeds;
  speeds.linear_velocity_mmps =
      linear_coeff * s_linear_speed_presets_mmps[msg.speed - 1];
  speeds.angular_velocity_dps =
      angular_coeff * s_angular_speed_presets_dps[msg.speed - 1];

  return speeds;
}
