#pragma once

#include <simpleble/SimpleBLE.h>

#include <functional>
#include <optional>
#include <string>
#include <string_view>

#define DEFAULT_PERIPHERAL_NAME "PetersMicroMouse"
#define DEFAULT_ADAPTER_INDEX 0

class BLEManager final {
  std::string m_peripheral_search_name;
  int m_adapter_index;

  SimpleBLE::Adapter m_adapter;
  mutable std::optional<SimpleBLE::Safe::Peripheral> m_peripheral;

  bool m_dummy = false;
  bool m_initialized = false;

 public:
  using OnConnectCallback = std::function<void()>;
  using OnDisconnectCallback = std::function<void()>;

  void set_on_connect_callback(OnConnectCallback callback) {
    m_on_connect_callback = std::move(callback);
  }
  void set_on_disconnect_callback(OnDisconnectCallback callback) {
    m_on_disconnect_callback = std::move(callback);
  }

 private:
  OnConnectCallback m_on_connect_callback;
  OnDisconnectCallback m_on_disconnect_callback;

 public:
  BLEManager(std::string_view peripheral_name = DEFAULT_PERIPHERAL_NAME,
             int adapter_index = DEFAULT_ADAPTER_INDEX,
             bool dummy = false);
  ~BLEManager();

  static const char* name() { return "ble"; }

  bool is_initialized() const { return m_initialized; }
  bool is_connected() const {
    if (m_dummy)
      return true;
    if (!m_peripheral.has_value())
      return false;
    return m_peripheral->is_connected().value_or(false);
  }

  int peripheral_rssi() const {
    if (!m_peripheral.has_value())
      return 0;
    return m_peripheral->rssi().value_or(0);
  }

  /**
   * @brief Since SimpleBLE does not allow for connecting to a peripheral from a
   *        callback (feature not a bug), this function must be called
   *        continuously when scanning to connect to the found peripheral.
   */
  void process_events();

 private:
  bool initialize_adapter();
  void configure_scan_callbacks();

  void begin_scan();
  void end_scan();

  void on_scan_found(SimpleBLE::Peripheral peripheral);
  void on_connected();
  void on_disconnected();
};
