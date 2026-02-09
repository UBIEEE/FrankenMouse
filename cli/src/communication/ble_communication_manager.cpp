#ifdef WITH_BLE

#include <micromouse_cli/communication/ble_communication_manager.hpp>

#include <cassert>

using namespace std::placeholders;
using namespace std::string_literals;

enum BLEService {
  SERVICE_UNKNOWN = -1,
  SERVICE_MAIN,
  SERVICE_VISION,
  SERVICE_DRIVE,
  SERVICE_MAZE,
  _SERVICE_COUNT,
};

using BLECharacteristic = FeedbackTopic;

static const SimpleBLE::BluetoothUUID s_services[_SERVICE_COUNT] = {
    [SERVICE_MAIN] = "00000000-cc7a-482a-984a-7f2ed5b3e58f"s,
    [SERVICE_VISION] = "00000001-cc7a-482a-984a-7f2ed5b3e58f"s,
    [SERVICE_DRIVE] = "00000002-cc7a-482a-984a-7f2ed5b3e58f"s,
    [SERVICE_MAZE] = "00000003-cc7a-482a-984a-7f2ed5b3e58f"s,
};

static const SimpleBLE::BluetoothUUID s_characteristics[_FB_TOPIC_COUNT] = {
    // Main (0000-0009)
    [FB_TOPIC_MAIN_TASK] = "00000000-8e22-4541-9d4c-21edae82ed19"s,
    [FB_TOPIC_MAIN_COMMAND] = "00000001-8e22-4541-9d4c-21edae82ed19"s,
    [FB_TOPIC_MAIN_ERROR] = "00000002-8e22-4541-9d4c-21edae82ed19"s,
    [FB_TOPIC_MAIN_SONG] = "00000003-8e22-4541-9d4c-21edae82ed19"s,
    [FB_TOPIC_MAIN_STATUS] = "00000004-8e22-4541-9d4c-21edae82ed19"s,

    // Vision (000A-000E)
    [FB_TOPIC_VISION_RAW_READINGS] = "0000000a-8e22-4541-9d4c-21edae82ed19"s,
    [FB_TOPIC_VISION_DISTANCES] = "0000000b-8e22-4541-9d4c-21edae82ed19"s,

    // Drive (000F-0013)
    [FB_TOPIC_DRIVE_MOTOR_DATA] = "0000000f-8e22-4541-9d4c-21edae82ed19"s,
    [FB_TOPIC_DRIVE_IMU_DATA] = "00000010-8e22-4541-9d4c-21edae82ed19"s,
    [FB_TOPIC_DRIVE_PID] = "00000011-8e22-4541-9d4c-21edae82ed19"s,
    [FB_TOPIC_DRIVE_CHASSIS_SPEEDS] = "00000012-8e22-4541-9d4c-21edae82ed19"s,

    // Maze (0014-0018)
    [FB_TOPIC_MAZE_CELL] = "00000014-8e22-4541-9d4c-21edae82ed19"s,
    [FB_TOPIC_MAZE_MOUSE_POSITION] = "00000015-8e22-4541-9d4c-21edae82ed19"s,
};

BLECommunicationManager::BLECommunicationManager(std::string_view peripheral_name,
                                                 size_t adapter_index,
                                                 bool dummy)
    : m_peripheral_search_name(peripheral_name), m_adapter_index(adapter_index), m_dummy(dummy) {
  if (m_dummy) {
    m_initialized = true;
    return;
  }

  if (!(m_initialized = initialize_adapter()))
    return;

  configure_scan_callbacks();

  begin_scan();
}

BLECommunicationManager::~BLECommunicationManager() {
  if (!m_initialized || m_dummy)
    return;

  if (!is_connected()) {
    end_scan();
    return;
  }

  bool success = m_peripheral->disconnect();
  if (!success) {
    report_error(name(), "failed to disconnect from '%s'", m_peripheral_search_name.c_str());
  }

  m_peripheral = std::nullopt;
}

void BLECommunicationManager::process_events() {
  if (!m_initialized || m_dummy)
    return;

  if (!m_peripheral.has_value())
    return;
  if (is_connected())
    return;

  bool success = m_peripheral->connect();
  if (!success) {
    report_error(name(), "failed to connect to '%s'", m_peripheral_search_name.c_str());
    m_peripheral = std::nullopt;
    return;
  }
}

bool BLECommunicationManager::initialize_adapter() {
  if (!SimpleBLE::Adapter::bluetooth_enabled()) {
    report_error(name(), "bluetooth is not enabled");
    return false;
  }

  auto adapters = SimpleBLE::Adapter::get_adapters();
  if (adapters.empty()) {
    report_error(name(), "no bluetooth adapters found");
    return false;
  }

  if (m_adapter_index >= adapters.size()) {
    report_error(name(), "adapter index %zu out of range [0, %zu)", m_adapter_index, adapters.size());
    return false;
  }

  m_adapter = adapters[m_adapter_index];

  report_status(name(), "using adapter '%s' [%s]", m_adapter.identifier().c_str(),
                m_adapter.address().c_str());

  return true;
}

void BLECommunicationManager::configure_scan_callbacks() {
  m_adapter.set_callback_on_scan_found(std::bind(&BLECommunicationManager::on_scan_found, this, _1));
}

void BLECommunicationManager::begin_scan() {
  if (!m_initialized || m_dummy)
    return;

  report_status(name(), "scanning for '%s'", m_peripheral_search_name.c_str());

  m_adapter.scan_start();
}

void BLECommunicationManager::end_scan() {
  if (!m_initialized || m_dummy)
    return;

  report_status(name(), "stopping scan");

  m_adapter.scan_stop();
}

void BLECommunicationManager::on_scan_found(SimpleBLE::Peripheral peripheral) {
  const std::string peripheral_name = peripheral.identifier();
  const std::string peripheral_address = peripheral.address();

  if (peripheral_name != m_peripheral_search_name)
    return;

  if (!peripheral.is_connectable()) {
    report_warning(name(), "found '%s', but not connectable", peripheral_name.c_str());
    return;
  }

  report_status(name(), "found '%s' [%s]", peripheral_name.c_str(), peripheral_address.c_str());

  m_peripheral = peripheral;
  end_scan();

  m_peripheral->set_callback_on_connected(std::bind(&BLECommunicationManager::on_connected, this));

  m_peripheral->set_callback_on_disconnected(std::bind(&BLECommunicationManager::on_disconnected, this));
}

void BLECommunicationManager::on_connected() {
  if (m_on_connect_callback)
    m_on_connect_callback();

  configure_peripheral_notifications();
  write<FeedbackTopicWrite::MAIN_COMMAND>(RobotCommand::RESEND_ALL_FEEDBACK);

  report_status(name(), "connected to '%s'", m_peripheral_search_name.c_str());
}

void BLECommunicationManager::on_disconnected() {
  if (m_on_disconnect_callback)
    m_on_disconnect_callback();

  report_status(name(), "disconnected from '%s'", m_peripheral_search_name.c_str());

  m_peripheral = std::nullopt;
  begin_scan();
}

bool BLECommunicationManager::configure_peripheral_notifications() {
  if (!is_connected())
    return false;

  notify<FeedbackTopicNotify::MAIN_TASK>(m_main_data.task);
  notify<FeedbackTopicNotify::MAIN_ERROR>(
      [&](const RobotError& error) { m_main_data.errors[error.timestamp] = error; });
  notify<FeedbackTopicNotify::MAIN_SONG>(m_main_data.song);
  notify<FeedbackTopicNotify::MAIN_STATUS>([&](const std::pair<RobotStatusTopic, uint8_t>& status) {
    m_main_data.statusTopics[status.first] = status.second;
  });

  notify<FeedbackTopicNotify::VISION_RAW_READINGS>(m_vision_data.raw_readings);
  notify<FeedbackTopicNotify::VISION_DISTANCES>(m_vision_data.distances);

  notify<FeedbackTopicNotify::DRIVE_MOTOR_DATA>(m_drive_data.motor_data);
  notify<FeedbackTopicNotify::DRIVE_IMU_DATA>(m_drive_data.imu_data);
  notify<FeedbackTopicNotify::DRIVE_PID>(m_drive_data.pid_data);
  notify<FeedbackTopicNotify::DRIVE_CHASSIS_SPEEDS>(m_drive_data.chassis_speeds);

  notify<FeedbackTopicNotify::MAZE_CELL>([&](const std::pair<Coordinate, Cell>& cell_with_position) {
    const auto& [position, cell] = cell_with_position;
    m_maze_data.cells[position] = cell;
  });

  notify<FeedbackTopicNotify::MAZE_MOUSE_POSITION>(m_maze_data.mouse_position);

  return true;
}

static BLEService get_characteristic_service(FeedbackTopic char_id) {
  switch (char_id) {
    case FB_TOPIC_MAIN_TASK:
    case FB_TOPIC_MAIN_COMMAND:
    case FB_TOPIC_MAIN_ERROR:
    case FB_TOPIC_MAIN_SONG:
    case FB_TOPIC_MAIN_STATUS:
      return SERVICE_MAIN;
    case FB_TOPIC_VISION_RAW_READINGS:
    case FB_TOPIC_VISION_DISTANCES:
      return SERVICE_VISION;
    case FB_TOPIC_DRIVE_MOTOR_DATA:
    case FB_TOPIC_DRIVE_IMU_DATA:
    case FB_TOPIC_DRIVE_PID:
    case FB_TOPIC_DRIVE_CHASSIS_SPEEDS:
      return SERVICE_DRIVE;
    case FB_TOPIC_MAZE_CELL:
    case FB_TOPIC_MAZE_MOUSE_POSITION:
      return SERVICE_MAZE;
    default:
      assert(false);  // TODO
      return SERVICE_UNKNOWN;
  }
}

void BLECommunicationManager::write_topic(FeedbackTopicWrite topic_id, const uint8_t* data, size_t size) {
  if (m_dummy)
    return;
  if (!is_connected())
    return;

  BLECharacteristic char_id = static_cast<BLECharacteristic>(topic_id);

  BLEService service_id = get_characteristic_service(char_id);
  assert(service_id != SERVICE_UNKNOWN);

  SimpleBLE::BluetoothUUID service_uuid = s_services[service_id];
  SimpleBLE::BluetoothUUID char_uuid = s_characteristics[char_id];

  bool result = m_peripheral->write_request(service_uuid, char_uuid, SimpleBLE::ByteArray(data, size));
  if (!result) {
    report_error(name(), "failed to publish characteristic %d", char_id);
  }
}

void BLECommunicationManager::notify_topic(FeedbackTopicNotify topic_id, NotificationFunc callback) {
  if (m_dummy)
    return;
  if (!is_connected())
    return;

  BLECharacteristic char_id = static_cast<BLECharacteristic>(topic_id);

  BLEService service_id = get_characteristic_service(char_id);
  assert(service_id != SERVICE_UNKNOWN);

  SimpleBLE::BluetoothUUID service_uuid = s_services[service_id];
  SimpleBLE::BluetoothUUID char_uuid = s_characteristics[char_id];

  bool result = m_peripheral->notify(service_uuid, char_uuid, callback);
  if (!result) {
    report_error(name(), "failed to subscribe to characteristic %d", char_id);
  }
}

#endif
