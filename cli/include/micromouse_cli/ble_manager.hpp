#pragma once

#include <simpleble/SimpleBLE.h>

#include <micromouse_cli/audio/song.hpp>
#include <micromouse_cli/diagnostics.hpp>
#include <micromouse_cli/drive/chassis_speeds.hpp>
#include <micromouse_cli/main/task.hpp>

#include <functional>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>

#define DEFAULT_PERIPHERAL_NAME "PetersMicroMouse"
#define DEFAULT_ADAPTER_INDEX 0

enum BLEService {
  SERVICE_UNKNOWN = -1,
  SERVICE_MUSIC,
  SERVICE_VISION,
  SERVICE_MAIN,
  SERVICE_DRIVE,
  SERVICE_MAZE,
  _SERVICE_COUNT,
};

enum BLECharacteristic {
  CHAR_MUSIC_PLAY_SONG,
  CHAR_MUSIC_IS_PLAYING,

  CHAR_VISION_RAW_READINGS,
  CHAR_VISION_DISTANCES,
  CHAR_VISION_CALIBRATE,

  CHAR_MAIN_TASK,
  CHAR_MAIN_APP_READY,
  CHAR_MAIN_ERROR_CODE,

  CHAR_DRIVE_MOTOR_DATA,
  CHAR_DRIVE_IMU_DATA,
  CHAR_DRIVE_PID_CONSTANTS,
  CHAR_DRIVE_CHASSIS_SPEEDS,

  CHAR_MAZE_RESET,
  CHAR_MAZE_CELL,

  _CHAR_COUNT,
};

// Topics sent to the robot.
enum class BLETopicWrite {
  MAIN_TASK = CHAR_MAIN_TASK,
  MAIN_APP_READY = CHAR_MAIN_APP_READY,

  DRIVE_PID = CHAR_DRIVE_PID_CONSTANTS,
  DRIVE_CHASSIS_SPEEDS = CHAR_DRIVE_CHASSIS_SPEEDS,

  VISION_CALIBRATE = CHAR_VISION_CALIBRATE,

  MAZE_RESET = CHAR_MAZE_RESET,

  MUSIC_PLAY_SONG = CHAR_MUSIC_PLAY_SONG,
};

template <BLETopicWrite Topic, typename Default = void>
struct BLETopicWriteData;

template <>
struct BLETopicWriteData<BLETopicWrite::MAIN_TASK> {
  using type = Task;
};
template <>
struct BLETopicWriteData<BLETopicWrite::DRIVE_PID> {
  using type = float[3 + 3];
};
template <>
struct BLETopicWriteData<BLETopicWrite::DRIVE_CHASSIS_SPEEDS> {
  using type = drive::ChassisSpeeds;
};
template <>
struct BLETopicWriteData<BLETopicWrite::MUSIC_PLAY_SONG> {
  using type = Song;
};

template <BLETopicWrite Topic>
struct BLETopicWriteData<Topic, void> {
  using type = uint8_t;
};

// Topics received from the robot.
enum class BLETopicNotify {
  MAIN_TASK = CHAR_MAIN_TASK,
  MAIN_ERROR = CHAR_MAIN_ERROR_CODE,

  DRIVE_MOTOR_DATA = CHAR_DRIVE_MOTOR_DATA,
  DRIVE_IMU_DATA = CHAR_DRIVE_IMU_DATA,
  DRIVE_PID = CHAR_DRIVE_PID_CONSTANTS,
  DRIVE_CHASSIS_SPEEDS = CHAR_DRIVE_CHASSIS_SPEEDS,

  VISION_RAW_READINGS = CHAR_VISION_RAW_READINGS,
  VISION_DISTANCES = CHAR_VISION_DISTANCES,
  VISION_CALIBRATE = CHAR_VISION_CALIBRATE,

  MAZE_CELL = CHAR_MAZE_CELL,
  // MAZE_MOUSE_POSITION = ,

  MUSIC_IS_PLAYING = CHAR_MUSIC_IS_PLAYING,
};

template <BLETopicNotify Topic, typename Default = void>
struct BLETopicNotifyData;

template <>
struct BLETopicNotifyData<BLETopicNotify::MAIN_TASK> {
  using type = Task;
};
template <>
struct BLETopicNotifyData<BLETopicNotify::DRIVE_MOTOR_DATA> {
  using type = float[4 + 3];
};
template <>
struct BLETopicNotifyData<BLETopicNotify::DRIVE_IMU_DATA> {
  using type = float[3 + 3];
};
template <>
struct BLETopicNotifyData<BLETopicNotify::DRIVE_PID> {
  using type = float[3 + 3];
};
template <>
struct BLETopicNotifyData<BLETopicNotify::DRIVE_CHASSIS_SPEEDS> {
  using type = drive::ChassisSpeeds;
};
template <>
struct BLETopicNotifyData<BLETopicNotify::VISION_RAW_READINGS> {
  using type = float[4];
};
template <>
struct BLETopicNotifyData<BLETopicNotify::VISION_DISTANCES> {
  using type = float[4];
};
template <>
struct BLETopicNotifyData<BLETopicNotify::VISION_CALIBRATE> {
  using type = bool;
};
template <>
struct BLETopicNotifyData<BLETopicNotify::MUSIC_IS_PLAYING> {
  using type = bool;
};

template <BLETopicNotify Topic>
struct BLETopicNotifyData<Topic, void> {
  using type = uint8_t;
};

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

  //
  // Write
  //

  template <BLETopicWrite Topic>
  void write(const BLETopicWriteData<Topic>::type& value) {
    using ValueType = BLETopicWriteData<Topic>::type;

    const uint8_t* data = reinterpret_cast<const uint8_t*>(&value);
    write(static_cast<BLECharacteristic>(Topic), data, sizeof(ValueType));
  }

  //
  // Notify
  //

  struct MusicNotifyData {
    bool is_playing = false;
  };
  const MusicNotifyData& music_data() const { return m_music_data; }

  struct VisionNotifyData {
    float raw_readings[4] = {0};
    float distances[4] = {0};
    bool is_calibrated = false;
  };
  const VisionNotifyData& vision_data() const { return m_vision_data; }

  struct DriveNotifyData {
    float motor_data[4 + 3] = {0};
    float imu_data[3 + 3] = {0};
    float pid_data[3 + 3] = {0};
    drive::ChassisSpeeds chassis_speeds = {};
  };
  const DriveNotifyData& drive_data() const { return m_drive_data; }

  struct MainNotifyData {
    Task task = Task::STOPPED;
    uint8_t error_code = 0;
  };
  const MainNotifyData& main_data() const { return m_main_data; }

  struct MazeNotifyData {
    uint8_t cell = 0;
  };
  const MazeNotifyData& maze_data() const { return m_maze_data; }

  template <BLETopicNotify Topic>
  const BLETopicNotifyData<Topic>::type& get_data() const {
    using enum BLETopicNotify;

    if constexpr (Topic == MAIN_TASK) {
      return m_main_data.task;
    } else if constexpr (Topic == MAIN_ERROR) {
      return m_main_data.error_code;
    } else if constexpr (Topic == DRIVE_MOTOR_DATA) {
      return m_drive_data.motor_data;
    } else if constexpr (Topic == DRIVE_IMU_DATA) {
      return m_drive_data.imu_data;
    } else if constexpr (Topic == DRIVE_PID) {
      return m_drive_data.pid_data;
    } else if constexpr (Topic == DRIVE_CHASSIS_SPEEDS) {
      return m_drive_data.chassis_speeds;
    } else if constexpr (Topic == VISION_RAW_READINGS) {
      return m_vision_data.raw_readings;
    } else if constexpr (Topic == VISION_DISTANCES) {
      return m_vision_data.distances;
    } else if constexpr (Topic == VISION_CALIBRATE) {
      return m_vision_data.is_calibrated;
    } else if constexpr (Topic == MAZE_CELL) {
      return m_maze_data.cell;
    } else if constexpr (Topic == MUSIC_IS_PLAYING) {
      return m_music_data.is_playing;
    } else
      static_assert(false, "Unsupported notify type");
  }

 private:
  bool initialize_adapter();
  void configure_scan_callbacks();

  void begin_scan();
  void end_scan();

  void on_scan_found(SimpleBLE::Peripheral peripheral);
  void on_connected();
  void on_disconnected();

  void configure_peripheral_notifications();

 private:
  /**
   * @brief Return the service for which the characteristic belongs to.
   */
  static BLEService get_characteristic_service(BLECharacteristic char_id);

  /**
   * @brief Write to a characteristic.
   *
   * @param char_id ID of the characteristic to write to.
   * @param data    Pointer to the data to write.
   * @param size    Number of bytes to write.
   */
  void write(BLECharacteristic char_id, const uint8_t* data, size_t size);

  using NotificationFunc = std::function<void(SimpleBLE::ByteArray payload)>;

  /**
   * @brief Register a callback for a characteristic notification.
   *
   * @param char_id  ID of the notify characteristic.
   * @param callback Callback function called when a notification is received.
   */
  void notify(BLECharacteristic char_id, NotificationFunc callback);

  template <BLETopicNotify Topic>
  NotificationFunc make_notification_callback(
      BLETopicNotifyData<Topic>::type& data) {
    return [&](SimpleBLE::ByteArray payload) {
      using ValueType = BLETopicNotifyData<Topic>::type;
      constexpr size_t ValueSize = sizeof(ValueType);
      if (payload.size() != ValueSize) {
        report_warning(name(), "notification for topic %d has invalid size %zu",
                       static_cast<int>(Topic), payload.size());
        return;
      }

      (void)std::memcpy(&data, payload.data(), ValueSize);
    };
  }

  template <BLETopicNotify Topic>
  void notify(BLETopicNotifyData<Topic>::type& data) {
    notify(static_cast<BLECharacteristic>(Topic),
           make_notification_callback<Topic>(data));
  }

 private:
  MusicNotifyData m_music_data;
  DriveNotifyData m_drive_data;
  VisionNotifyData m_vision_data;
  MainNotifyData m_main_data;
  MazeNotifyData m_maze_data;
};
