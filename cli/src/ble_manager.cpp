#include <micromouse_cli/ble_manager.hpp>

#include <cassert>

using namespace std::placeholders;
using namespace std::string_literals;

static const SimpleBLE::BluetoothUUID s_services[_SERVICE_COUNT] = {
    [SERVICE_MUSIC] = "00000000-cc7a-482a-984a-7f2ed5b3e58f"s,
    [SERVICE_VISION] = "00000001-cc7a-482a-984a-7f2ed5b3e58f"s,
    [SERVICE_MAIN] = "00000002-cc7a-482a-984a-7f2ed5b3e58f"s,
    [SERVICE_DRIVE] = "00000003-cc7a-482a-984a-7f2ed5b3e58f"s,
    [SERVICE_MAZE] = "00000004-cc7a-482a-984a-7f2ed5b3e58f"s,
};

static const SimpleBLE::BluetoothUUID s_characteristics[_CHAR_COUNT] = {
    // Music (0000-0004)
    [CHAR_MUSIC_PLAY_SONG] = "00000000-8e22-4541-9d4c-21edae82ed19"s,
    [CHAR_MUSIC_IS_PLAYING] = "00000001-8e22-4541-9d4c-21edae82ed19"s,

    // Vision (0005-0009)
    [CHAR_VISION_RAW_READINGS] = "00000005-8e22-4541-9d4c-21edae82ed19"s,
    [CHAR_VISION_DISTANCES] = "00000006-8e22-4541-9d4c-21edae82ed19"s,
    [CHAR_VISION_CALIBRATE] = "00000007-8e22-4541-9d4c-21edae82ed19"s,

    // Main (000A-000E)
    [CHAR_MAIN_TASK] = "0000000a-8e22-4541-9d4c-21edae82ed19"s,
    [CHAR_MAIN_APP_READY] = "0000000b-8e22-4541-9d4c-21edae82ed19"s,
    [CHAR_MAIN_ERROR_CODE] = "0000000c-8e22-4541-9d4c-21edae82ed19"s,

    // Drive (000F-0013)
    [CHAR_DRIVE_MOTOR_DATA] = "0000000f-8e22-4541-9d4c-21edae82ed19"s,
    [CHAR_DRIVE_IMU_DATA] = "00000010-8e22-4541-9d4c-21edae82ed19"s,
    [CHAR_DRIVE_PID_CONSTANTS] = "00000011-8e22-4541-9d4c-21edae82ed19"s,

    // Maze (0014-0018)
    [CHAR_MAZE_RESET] = "00000014-8e22-4541-9d4c-21edae82ed19"s,
    [CHAR_MAZE_CELL] = "00000015-8e22-4541-9d4c-21edae82ed19"s,
};

BLEManager::BLEManager(std::string_view peripheral_name,
                       int adapter_index,
                       bool dummy)
    : m_peripheral_search_name(peripheral_name),
      m_adapter_index(adapter_index),
      m_dummy(dummy) {
  if (m_dummy) {
    m_initialized = true;
    return;
  }

  if (!(m_initialized = initialize_adapter()))
    return;

  configure_scan_callbacks();

  begin_scan();
}

BLEManager::~BLEManager() {
  if (!m_initialized || m_dummy)
    return;

  if (!is_connected()) {
    end_scan();
    return;
  }

  bool success = m_peripheral->disconnect();
  if (!success) {
    report_error(name(), "failed to disconnect from '%s'",
                 m_peripheral_search_name.c_str());
  }

  m_peripheral = std::nullopt;
}

void BLEManager::process_events() {
  if (!m_initialized || m_dummy)
    return;

  if (!m_peripheral.has_value())
    return;
  if (is_connected())
    return;

  bool success = m_peripheral->connect();
  if (!success) {
    report_error(name(), "failed to connect to '%s'",
                 m_peripheral_search_name.c_str());
    m_peripheral = std::nullopt;
    return;
  }
}

bool BLEManager::initialize_adapter() {
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
    report_error(name(), "adapter index %d out of range [0, %zu)",
                 m_adapter_index, adapters.size());
    return false;
  }

  m_adapter = adapters[m_adapter_index];

  report_status(name(), "using adapter '%s' [%s]",
                m_adapter.identifier().c_str(), m_adapter.address().c_str());

  return true;
}

void BLEManager::configure_scan_callbacks() {
  m_adapter.set_callback_on_scan_found(
      std::bind(&BLEManager::on_scan_found, this, _1));
}

void BLEManager::begin_scan() {
  if (!m_initialized || m_dummy)
    return;

  report_status(name(), "scanning for '%s'", m_peripheral_search_name.c_str());

  m_adapter.scan_start();
}

void BLEManager::end_scan() {
  if (!m_initialized || m_dummy)
    return;

  report_status(name(), "stopping scan");

  m_adapter.scan_stop();
}

void BLEManager::on_scan_found(SimpleBLE::Peripheral peripheral) {
  const std::string peripheral_name = peripheral.identifier();
  const std::string peripheral_address = peripheral.address();

  if (peripheral_name != m_peripheral_search_name)
    return;

  if (!peripheral.is_connectable()) {
    report_warning(name(), "found '%s', but not connectable",
                   peripheral_name.c_str());
    return;
  }

  report_status(name(), "found '%s' [%s]", peripheral_name.c_str(),
                peripheral_address.c_str());

  m_peripheral = peripheral;
  end_scan();

  m_peripheral->set_callback_on_connected(
      std::bind(&BLEManager::on_connected, this));

  m_peripheral->set_callback_on_disconnected(
      std::bind(&BLEManager::on_disconnected, this));
}

void BLEManager::on_connected() {
  if (m_on_connect_callback)
    m_on_connect_callback();

  configure_peripheral_notifications();
  write<BLETopicWrite::MAIN_APP_READY>(1);

  report_status(name(), "connected to '%s'", m_peripheral_search_name.c_str());
}

void BLEManager::on_disconnected() {
  if (m_on_disconnect_callback)
    m_on_disconnect_callback();

  report_status(name(), "disconnected from '%s'",
                m_peripheral_search_name.c_str());

  m_peripheral = std::nullopt;
  begin_scan();
}

void BLEManager::configure_peripheral_notifications() {
  if (!is_connected())
    return;

  notify<BLETopicNotify::MAIN_TASK>(m_main_data.task);
  notify<BLETopicNotify::MAIN_ERROR>(m_main_data.error_code);

  notify<BLETopicNotify::DRIVE_MOTOR_DATA>(m_drive_data.motor_data);
  notify<BLETopicNotify::DRIVE_IMU_DATA>(m_drive_data.imu_data);
  notify<BLETopicNotify::DRIVE_PID>(m_drive_data.pid_data);

  notify<BLETopicNotify::VISION_RAW_READINGS>(m_vision_data.raw_readings);
  notify<BLETopicNotify::VISION_DISTANCES>(m_vision_data.distances);
  notify<BLETopicNotify::VISION_CALIBRATE>(m_vision_data.is_calibrated);

  notify<BLETopicNotify::MAZE_CELL>(m_maze_data.cell);

  notify<BLETopicNotify::MUSIC_IS_PLAYING>(m_music_data.is_playing);
}

BLEService BLEManager::get_characteristic_service(BLECharacteristic char_id) {
  switch (char_id) {
    case CHAR_MUSIC_PLAY_SONG:
    case CHAR_MUSIC_IS_PLAYING:
      return SERVICE_MUSIC;
    case CHAR_VISION_RAW_READINGS:
    case CHAR_VISION_DISTANCES:
    case CHAR_VISION_CALIBRATE:
      return SERVICE_VISION;
    case CHAR_MAIN_TASK:
    case CHAR_MAIN_APP_READY:
    case CHAR_MAIN_ERROR_CODE:
      return SERVICE_MAIN;
    case CHAR_DRIVE_MOTOR_DATA:
    case CHAR_DRIVE_IMU_DATA:
    case CHAR_DRIVE_PID_CONSTANTS:
      return SERVICE_DRIVE;
    case CHAR_MAZE_RESET:
    case CHAR_MAZE_CELL:
      return SERVICE_MAZE;
    default:
      assert(false);
      return SERVICE_UNKNOWN;
  }
}

void BLEManager::write(BLECharacteristic char_id,
                       const uint8_t* data,
                       size_t size) {
  if (m_dummy)
    return;
  if (!is_connected())
    return;

  BLEService service_id = get_characteristic_service(char_id);
  assert(service_id != SERVICE_UNKNOWN);

  SimpleBLE::BluetoothUUID service_uuid = s_services[service_id];
  SimpleBLE::BluetoothUUID char_uuid = s_characteristics[char_id];

  bool result = m_peripheral->write_request(service_uuid, char_uuid,
                                            SimpleBLE::ByteArray(data, size));
  if (!result) {
    report_error(name(), "failed to publish characteristic %d", char_id);
  }
}

void BLEManager::notify(BLECharacteristic char_id, NotificationFunc callback) {
  if (m_dummy)
    return;
  if (!is_connected())
    return;

  BLEService service_id = get_characteristic_service(char_id);
  assert(service_id != SERVICE_UNKNOWN);

  SimpleBLE::BluetoothUUID service_uuid = s_services[service_id];
  SimpleBLE::BluetoothUUID char_uuid = s_characteristics[char_id];

  bool result =
      m_peripheral->notify(service_uuid, char_uuid, std::move(callback));
  if (!result) {
    report_error(name(), "failed to subscribe to characteristic %d", char_id);
  }
}
