#include <micromouse_cli/ble_manager.hpp>

#include <micromouse_cli/diagnostics.hpp>

using namespace std::placeholders;

BLEManager::BLEManager(std::string_view peripheral_name, int adapter_index)
    : m_peripheral_search_name(peripheral_name),
      m_adapter_index(adapter_index) {
  if (!(m_initialized = initialize_adapter()))
    return;

  configure_scan_callbacks();

  begin_scan();
}

BLEManager::~BLEManager() {
  if (!m_initialized)
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
  if (!m_initialized)
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
  if (!m_initialized)
    return;

  report_status(name(), "scanning for '%s'", m_peripheral_search_name.c_str());

  m_adapter.scan_start();
}

void BLEManager::end_scan() {
  if (!m_initialized)
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
