#pragma once

#include <simpleble/SimpleBLE.h>

#include <micromouse_cli/diagnostics.hpp>
#include <micromouse_cli/drive/chassis_speeds.hpp>
#include <micromouse_cli/maze/cell.hpp>
#include <micromouse_cli/maze/coordinate.hpp>
#include <micromouse_cli/robot/command.hpp>
#include <micromouse_cli/robot/error.hpp>
#include <micromouse_cli/robot/song.hpp>
#include <micromouse_cli/robot/status_topic.hpp>
#include <micromouse_cli/robot/task.hpp>

#include <functional>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#define DEFAULT_PERIPHERAL_NAME "PetersMicroMouse"
#define DEFAULT_ADAPTER_INDEX 0

enum BLEService {
  SERVICE_UNKNOWN = -1,
  SERVICE_MAIN,
  SERVICE_VISION,
  SERVICE_DRIVE,
  SERVICE_MAZE,
  _SERVICE_COUNT,
};

enum BLECharacteristic {
  CHAR_MAIN_TASK,
  CHAR_MAIN_COMMAND,
  CHAR_MAIN_ERROR,
  CHAR_MAIN_SONG,
  CHAR_MAIN_STATUS,

  CHAR_VISION_RAW_READINGS,
  CHAR_VISION_DISTANCES,

  CHAR_DRIVE_MOTOR_DATA,
  CHAR_DRIVE_IMU_DATA,
  CHAR_DRIVE_PID,
  CHAR_DRIVE_CHASSIS_SPEEDS,

  CHAR_MAZE_CELL,
  CHAR_MAZE_MOUSE_POSITION,

  _CHAR_COUNT,
};

// Topics sent to the robot.
enum class BLETopicWrite {
  MAIN_TASK = CHAR_MAIN_TASK,
  MAIN_COMMAND = CHAR_MAIN_COMMAND,
  MAIN_SONG = CHAR_MAIN_SONG,

  DRIVE_PID = CHAR_DRIVE_PID,
  DRIVE_CHASSIS_SPEEDS = CHAR_DRIVE_CHASSIS_SPEEDS,
};

template <BLETopicWrite Topic, typename Default = void>
struct BLETopicWriteData;

template <>
struct BLETopicWriteData<BLETopicWrite::MAIN_TASK> {
  using type = std::pair<RobotTask, uint8_t>;
};
template <>
struct BLETopicWriteData<BLETopicWrite::MAIN_COMMAND> {
  using type = RobotCommand;
};
template <>
struct BLETopicWriteData<BLETopicWrite::MAIN_SONG> {
  using type = RobotSong;
};
template <>
struct BLETopicWriteData<BLETopicWrite::DRIVE_PID> {
  using type = float[3 + 3];
};
template <>
struct BLETopicWriteData<BLETopicWrite::DRIVE_CHASSIS_SPEEDS> {
  using type = drive::ChassisSpeeds;
};

template <BLETopicWrite Topic>
struct BLETopicWriteData<Topic, void> {
  using type = uint8_t;
};

// Topics received from the robot.
enum class BLETopicNotify {
  MAIN_TASK = CHAR_MAIN_TASK,
  MAIN_ERROR = CHAR_MAIN_ERROR,
  MAIN_SONG = CHAR_MAIN_SONG,
  MAIN_STATUS = CHAR_MAIN_STATUS,

  VISION_RAW_READINGS = CHAR_VISION_RAW_READINGS,
  VISION_DISTANCES = CHAR_VISION_DISTANCES,

  DRIVE_MOTOR_DATA = CHAR_DRIVE_MOTOR_DATA,
  DRIVE_IMU_DATA = CHAR_DRIVE_IMU_DATA,
  DRIVE_PID = CHAR_DRIVE_PID,
  DRIVE_CHASSIS_SPEEDS = CHAR_DRIVE_CHASSIS_SPEEDS,

  MAZE_CELL = CHAR_MAZE_CELL,
  MAZE_MOUSE_POSITION = CHAR_MAZE_MOUSE_POSITION,
};

template <BLETopicNotify Topic, typename Default = void>
struct BLETopicNotifyData;

template <>
struct BLETopicNotifyData<BLETopicNotify::MAIN_TASK> {
  using type = std::pair<RobotTask, uint8_t>;
};
template <>
struct BLETopicNotifyData<BLETopicNotify::MAIN_ERROR> {
  using type = RobotError;
};
template <>
struct BLETopicNotifyData<BLETopicNotify::MAIN_SONG> {
  using type = RobotSong;
};
template <>
struct BLETopicNotifyData<BLETopicNotify::MAIN_STATUS> {
  using type = std::pair<RobotStatusTopic, uint8_t>;
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
struct BLETopicNotifyData<BLETopicNotify::MAZE_CELL> {
  using type = std::pair<Coordinate, Cell>;
};
template <>
struct BLETopicNotifyData<BLETopicNotify::MAZE_MOUSE_POSITION> {
  using type = Coordinate;
};

template <BLETopicNotify Topic>
struct BLETopicNotifyData<Topic, void> {
  using type = uint8_t;
};

class BLEManager final {
  std::string m_peripheral_search_name;
  size_t m_adapter_index;

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
  explicit BLEManager(
      std::string_view peripheral_name = DEFAULT_PERIPHERAL_NAME,
      size_t adapter_index = DEFAULT_ADAPTER_INDEX,
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

  struct MainNotifyData {
    std::pair<RobotTask, uint8_t> task = {RobotTask::STOPPED, 0};
    std::map<uint32_t, RobotError> errors;
    RobotSong song;
    std::map<RobotStatusTopic, uint8_t> statusTopics;
  };
  const MainNotifyData& main_data() const { return m_main_data; }

  struct VisionNotifyData {
    float raw_readings[4] = {0};
    float distances[4] = {0};
  };
  const VisionNotifyData& vision_data() const { return m_vision_data; }

  struct DriveNotifyData {
    float motor_data[4 + 3] = {0};
    float imu_data[3 + 3] = {0};
    float pid_data[3 + 3] = {0};
    drive::ChassisSpeeds chassis_speeds = {};
  };
  const DriveNotifyData& drive_data() const { return m_drive_data; }

  struct MazeNotifyData {
    Cell cells[16 * 16] = {};
    Coordinate mouse_position;
  };
  const MazeNotifyData& maze_data() const { return m_maze_data; }

  template <BLETopicNotify Topic>
  const BLETopicNotifyData<Topic>::type& get_data() const {
    using enum BLETopicNotify;

    if constexpr (Topic == MAIN_TASK) {
      return m_main_data.task;
    } else if constexpr (Topic == MAIN_ERROR) {
      static_assert(false, "Use main_data().errors to access all errors");
    } else if constexpr (Topic == MAIN_SONG) {
      return m_main_data.song;
    } else if constexpr (Topic == MAIN_STATUS) {
      static_assert(false,
                    "Use main_data().statusTopics to access all status topics");
    } else if constexpr (Topic == VISION_RAW_READINGS) {
      return m_vision_data.raw_readings;
    } else if constexpr (Topic == VISION_DISTANCES) {
      return m_vision_data.distances;
    } else if constexpr (Topic == DRIVE_MOTOR_DATA) {
      return m_drive_data.motor_data;
    } else if constexpr (Topic == DRIVE_IMU_DATA) {
      return m_drive_data.imu_data;
    } else if constexpr (Topic == DRIVE_PID) {
      return m_drive_data.pid_data;
    } else if constexpr (Topic == DRIVE_CHASSIS_SPEEDS) {
      return m_drive_data.chassis_speeds;
    } else if constexpr (Topic == MAZE_CELL) {
      static_assert(false, "Use maze_data().cells to access all maze cells");
    } else if constexpr (Topic == MAZE_MOUSE_POSITION) {
      return m_maze_data.mouse_position;
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
  static void notification_callback(BLETopicNotifyData<Topic>::type& data,
                                    SimpleBLE::ByteArray payload) {
    using ValueType = BLETopicNotifyData<Topic>::type;
    constexpr size_t ValueSize = sizeof(ValueType);
    if (payload.size() != ValueSize) {
      report_warning(name(), "notification for topic %d has invalid size %zu",
                     static_cast<int>(Topic), payload.size());
      return;
    }

    (void)std::memcpy(&data, payload.data(), ValueSize);
  };

  template <BLETopicNotify Topic>
  void notify(BLETopicNotifyData<Topic>::type& data) {
    notify(static_cast<BLECharacteristic>(Topic),
           [&](SimpleBLE::ByteArray payload) {
             notification_callback<Topic>(data, payload);
           });
  }

  template <BLETopicNotify Topic>
  void notify(
      std::function<void(const typename BLETopicNotifyData<Topic>::type& data)>
          callback) {
    notify(static_cast<BLECharacteristic>(Topic),
           [&](SimpleBLE::ByteArray payload) {
             typename BLETopicNotifyData<Topic>::type data;
             notification_callback<Topic>(data, payload);
             callback(data);
           });
  }

 private:
  MainNotifyData m_main_data;
  DriveNotifyData m_drive_data;
  VisionNotifyData m_vision_data;
  MazeNotifyData m_maze_data;
};
