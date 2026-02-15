#pragma once

#ifndef WITH_BLE
#error "BLE not enabled, so don't include this file!"
#endif

#include <micromouse_cli/communication/communication_manager.hpp>
#include <simpleble/SimpleBLE.h>

#define BLE_DEFAULT_PERIPHERAL_NAME "PetersMicroMouse"
#define BLE_DEFAULT_ADAPTER_INDEX 0

class BLECommunicationManager final : public CommunicationManager {
  std::string m_peripheral_search_name;
  size_t m_adapter_index;

  SimpleBLE::Adapter m_adapter;
  mutable std::optional<SimpleBLE::Safe::Peripheral> m_peripheral;

  bool m_dummy = false;
  bool m_initialized = false;

 private:
  OnConnectCallback m_on_connect_callback;
  OnDisconnectCallback m_on_disconnect_callback;

 public:
  explicit BLECommunicationManager(std::string_view peripheral_name = BLE_DEFAULT_PERIPHERAL_NAME,
                                   size_t adapter_index = BLE_DEFAULT_ADAPTER_INDEX,
                                   bool dummy = false);
  ~BLECommunicationManager();

  static const char* name() { return "ble"; }

  bool is_initialized() const override { return m_initialized; }
  bool is_connected() const override {
    if (m_dummy)
      return true;
    if (!m_peripheral.has_value())
      return false;
    return m_peripheral->is_connected().value_or(false);
  }

  int peripheral_rssi() const override {
    if (!m_peripheral.has_value())
      return 0;
    return m_peripheral->rssi().value_or(0);
  }

  /**
   * @brief Since SimpleBLE does not allow for connecting to a peripheral from a
   *        callback (feature not a bug), this function must be called
   *        continuously when scanning to connect to the found peripheral.
   */
  void process_connection() override;

  void set_on_connect_callback(OnConnectCallback callback) override {
    m_on_connect_callback = std::move(callback);
  }
  void set_on_disconnect_callback(OnDisconnectCallback callback) override {
    m_on_disconnect_callback = std::move(callback);
  }

 private:
  bool initialize_adapter();
  void configure_scan_callbacks();

  void begin_scan();
  void end_scan();

  void on_scan_found(SimpleBLE::Peripheral peripheral);
  void on_connected();
  void on_disconnected();

  bool configure_peripheral_notifications();

 private:
  void write_topic(FeedbackTopicWrite topic_id, const uint8_t* data, size_t size) override;

  using NotificationFunc = std::function<void(SimpleBLE::ByteArray payload)>;

  void notify_topic(FeedbackTopicNotify topic_id, NotificationFunc callback);

  template <FeedbackTopicNotify Topic>
  static void notification_callback(typename FeedbackTopicNotifyData<Topic>::type& data,
                                    SimpleBLE::ByteArray payload) {
    using ValueType = FeedbackTopicNotifyData<Topic>::type;
    constexpr size_t ValueSize = sizeof(ValueType);
    if (payload.size() != ValueSize) {
      report_warning(name(), "notification for topic %d has invalid size %zu", static_cast<int>(Topic),
                     payload.size());
      return;
    }

    (void)std::memcpy(&data, payload.data(), ValueSize);
  };

  template <FeedbackTopicNotify Topic>
  void notify(typename FeedbackTopicNotifyData<Topic>::type& data) {
    notify_topic(Topic, [&](SimpleBLE::ByteArray payload) { notification_callback<Topic>(data, payload); });
  }

  template <FeedbackTopicNotify Topic>
  void notify(std::function<void(const typename FeedbackTopicNotifyData<Topic>::type& data)> callback) {
    notify_topic(Topic, [&](SimpleBLE::ByteArray payload) {
      typename FeedbackTopicNotifyData<Topic>::type data;
      notification_callback<Topic>(data, payload);
      callback(data);
    });
  }
};
