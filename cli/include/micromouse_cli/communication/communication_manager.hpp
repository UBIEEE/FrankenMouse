#pragma once

#include <micromouse_cli/diagnostics.hpp>
#include <micromouse_cli/drive/chassis_speeds.hpp>
#include <micromouse_cli/maze/cell.hpp>
#include <micromouse_cli/maze/coordinate.hpp>
#include <micromouse_cli/robot/command.hpp>
#include <micromouse_cli/robot/error.hpp>
#include <micromouse_cli/robot/song.hpp>
#include <micromouse_cli/robot/status_topic.hpp>
#include <micromouse_cli/robot/task.hpp>
#include <micromouse_cli/robot/start_position.hpp>

#include <cstdint>
#include <functional>
#include <map>
#include <utility>

enum FeedbackTopic {
  FB_TOPIC_MAIN_TASK,
  FB_TOPIC_MAIN_COMMAND,
  FB_TOPIC_MAIN_ERROR,
  FB_TOPIC_MAIN_SONG,
  FB_TOPIC_MAIN_STATUS,

  FB_TOPIC_VISION_RAW_READINGS,
  FB_TOPIC_VISION_DISTANCES,

  FB_TOPIC_DRIVE_MOTOR_DATA,
  FB_TOPIC_DRIVE_IMU_DATA,
  FB_TOPIC_DRIVE_PID,
  FB_TOPIC_DRIVE_CHASSIS_SPEEDS,

  FB_TOPIC_MAZE_CELL,
  FB_TOPIC_MAZE_MOUSE_POSITION,

  _FB_TOPIC_COUNT,
};

// Topics sent to the robot.
enum class FeedbackTopicWrite {
  MAIN_TASK = FB_TOPIC_MAIN_TASK,
  MAIN_COMMAND = FB_TOPIC_MAIN_COMMAND,
  MAIN_SONG = FB_TOPIC_MAIN_SONG,

  DRIVE_PID = FB_TOPIC_DRIVE_PID,
  DRIVE_CHASSIS_SPEEDS = FB_TOPIC_DRIVE_CHASSIS_SPEEDS,
};

template <FeedbackTopicWrite Topic, typename Default = void>
struct FeedbackTopicWriteData;

struct MainTaskData {
  RobotTask task = RobotTask::STOPPED;
  RobotStartPosition start_position = RobotStartPosition::LEFT_OF_GOAL;

  MainTaskData() {}

  /*implicit*/ MainTaskData(RobotTask task) : task(task) {}

  MainTaskData(RobotTask task, RobotStartPosition start_position)
      : task(task), start_position(start_position) {}

  /*implicit*/ operator RobotTask() const { return task; }
};

template <>
struct FeedbackTopicWriteData<FeedbackTopicWrite::MAIN_TASK> {
  using type = MainTaskData;
};
template <>
struct FeedbackTopicWriteData<FeedbackTopicWrite::MAIN_COMMAND> {
  using type = RobotCommand;
};
template <>
struct FeedbackTopicWriteData<FeedbackTopicWrite::MAIN_SONG> {
  using type = RobotSong;
};
template <>
struct FeedbackTopicWriteData<FeedbackTopicWrite::DRIVE_PID> {
  using type = float[3 + 3];
};
template <>
struct FeedbackTopicWriteData<FeedbackTopicWrite::DRIVE_CHASSIS_SPEEDS> {
  using type = drive::ChassisSpeeds;
};

template <FeedbackTopicWrite Topic>
struct FeedbackTopicWriteData<Topic, void> {
  using type = uint8_t;
};

// Topics received from the robot.
enum class FeedbackTopicNotify {
  MAIN_TASK = FB_TOPIC_MAIN_TASK,
  MAIN_ERROR = FB_TOPIC_MAIN_ERROR,
  MAIN_SONG = FB_TOPIC_MAIN_SONG,
  MAIN_STATUS = FB_TOPIC_MAIN_STATUS,

  VISION_RAW_READINGS = FB_TOPIC_VISION_RAW_READINGS,
  VISION_DISTANCES = FB_TOPIC_VISION_DISTANCES,

  DRIVE_MOTOR_DATA = FB_TOPIC_DRIVE_MOTOR_DATA,
  DRIVE_IMU_DATA = FB_TOPIC_DRIVE_IMU_DATA,
  DRIVE_PID = FB_TOPIC_DRIVE_PID,
  DRIVE_CHASSIS_SPEEDS = FB_TOPIC_DRIVE_CHASSIS_SPEEDS,

  MAZE_CELL = FB_TOPIC_MAZE_CELL,
  MAZE_MOUSE_POSITION = FB_TOPIC_MAZE_MOUSE_POSITION,
};

template <FeedbackTopicNotify Topic, typename Default = void>
struct FeedbackTopicNotifyData;

template <>
struct FeedbackTopicNotifyData<FeedbackTopicNotify::MAIN_TASK> {
  using type = MainTaskData;
};
template <>
struct FeedbackTopicNotifyData<FeedbackTopicNotify::MAIN_ERROR> {
  using type = RobotError;
};
template <>
struct FeedbackTopicNotifyData<FeedbackTopicNotify::MAIN_SONG> {
  using type = RobotSong;
};
template <>
struct FeedbackTopicNotifyData<FeedbackTopicNotify::MAIN_STATUS> {
  using type = std::pair<RobotStatusTopic, uint8_t>;
};
template <>
struct FeedbackTopicNotifyData<FeedbackTopicNotify::VISION_RAW_READINGS> {
  using type = float[4];
};
template <>
struct FeedbackTopicNotifyData<FeedbackTopicNotify::VISION_DISTANCES> {
  using type = float[4];
};
template <>
struct FeedbackTopicNotifyData<FeedbackTopicNotify::DRIVE_MOTOR_DATA> {
  using type = float[3 + 3];
};
template <>
struct FeedbackTopicNotifyData<FeedbackTopicNotify::DRIVE_IMU_DATA> {
  using type = float[3 + 3];
};
template <>
struct FeedbackTopicNotifyData<FeedbackTopicNotify::DRIVE_PID> {
  using type = float[3 + 3];
};
template <>
struct FeedbackTopicNotifyData<FeedbackTopicNotify::DRIVE_CHASSIS_SPEEDS> {
  using type = drive::ChassisSpeeds;
};
template <>
struct FeedbackTopicNotifyData<FeedbackTopicNotify::MAZE_CELL> {
  using type = std::pair<Coordinate, Cell>;
};
template <>
struct FeedbackTopicNotifyData<FeedbackTopicNotify::MAZE_MOUSE_POSITION> {
  using type = Coordinate;
};

template <FeedbackTopicNotify Topic>
struct FeedbackTopicNotifyData<Topic, void> {
  using type = uint8_t;
};

class CommunicationManager {
 protected:
  CommunicationManager() = default;

 public:
  virtual ~CommunicationManager() = default;

  virtual bool is_initialized() const = 0;
  virtual bool is_connected() const = 0;

  virtual int peripheral_rssi() const { return 0; }

  virtual void process_connection() {}

  using OnConnectCallback = std::function<void()>;
  using OnDisconnectCallback = std::function<void()>;

  virtual void set_on_connect_callback(OnConnectCallback) {}
  virtual void set_on_disconnect_callback(OnDisconnectCallback) {}

  //
  // Write
  //

  /**
   * Write a value to a feedback topic.
   *
   * @tparam Topic Feedback topic to write to.
   * @param value Value to send.
   */
  template <FeedbackTopicWrite Topic>
  void write(const typename FeedbackTopicWriteData<Topic>::type& value) {
    using ValueType = FeedbackTopicWriteData<Topic>::type;

    const uint8_t* data = reinterpret_cast<const uint8_t*>(&value);
    write_topic(Topic, data, sizeof(ValueType));
  }

  //
  // Notify
  //

  struct MainNotifyData {
    union {
      MainTaskData task_data {};
      struct {
        RobotTask task;
        RobotStartPosition start_position;
      };
    };
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
    float motor_data[3 + 3] = {0};
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

  /**
   * Get the value of a feedback topic.
   *
   * @tparam Topic Feedback topic to get the value of.
   *
   * @return The value of the feedback topic.
   */
  template <FeedbackTopicNotify Topic>
  const typename FeedbackTopicNotifyData<Topic>::type get_value() const {
    using enum FeedbackTopicNotify;

    if constexpr (Topic == MAIN_TASK) {
      return m_main_data.task; // implicit conversion, nobody cares about start position
    } else if constexpr (Topic == MAIN_ERROR) {
      static_assert(false, "Use main_data().errors to access all errors");
    } else if constexpr (Topic == MAIN_SONG) {
      return m_main_data.song;
    } else if constexpr (Topic == MAIN_STATUS) {
      static_assert(false, "Use main_data().statusTopics to access all status topics");
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

 protected:
  using NotificationFunc = std::function<void(const uint8_t* data, size_t size)>;

  /**
   * Write data to a feedback topic. This function must be implemented by derived classes.
   *
   * @param topic_id Feedback topic to write to.
   * @param data     Pointer to the data to write.
   * @param size     Number of bytes to write.
   */
  virtual void write_topic(FeedbackTopicWrite topic_id, const uint8_t* data, size_t size) = 0;

 protected:
  MainNotifyData m_main_data;
  DriveNotifyData m_drive_data;
  VisionNotifyData m_vision_data;
  MazeNotifyData m_maze_data;
};
