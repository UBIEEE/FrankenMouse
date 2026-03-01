#pragma once

#include <micromouse/feedback/feedback_topic.h>
#include <micromouse/robot/task.hpp>
#include <micromouse/maze/maze.hpp>
#include <micromouse/robot/command.hpp>
#include <micromouse/audio/song.hpp>
#include <micromouse/drive/kinematics.hpp>
#include <micromouse/robot/error.hpp>
#include <micromouse/robot/status_topic.hpp>
#include <micromouse/hardware/drivetrain.hpp>
#include <micromouse/hardware/imu.hpp>
#include <cstdint>
#include <array>

namespace feedback {

// Topics received from clients.
enum class TopicReceive : uint8_t {
  MAIN_TASK = FB_TOPIC_RECEIVE_MAIN_TASK,
  MAIN_COMMAND = FB_TOPIC_RECEIVE_MAIN_COMMAND,
  MAIN_SONG = FB_TOPIC_RECEIVE_MAIN_SONG,

  DRIVE_PID = FB_TOPIC_RECEIVE_DRIVE_PID,
  DRIVE_CHASSIS_SPEEDS = FB_TOPIC_RECEIVE_DRIVE_CHASSIS_SPEEDS,
};

struct MainTaskData {
  robot::Task task = robot::Task::STOPPED;
  maze::Maze::StartLocation start_position = maze::Maze::StartLocation::WEST_OF_GOAL;

  MainTaskData() {}

  /*implicit*/ MainTaskData(robot::Task task) : task(task) {}

  MainTaskData(robot::Task task, maze::Maze::StartLocation start_position)
      : task(task), start_position(start_position) {}

  /*implicit*/ operator robot::Task() const { return task; }
};

template <TopicReceive Topic, typename Default = void>
struct TopicReceiveData;

template <>
struct TopicReceiveData<TopicReceive::MAIN_TASK> {
  using type = MainTaskData;
};
template <>
struct TopicReceiveData<TopicReceive::MAIN_COMMAND> {
  using type = robot::Command;
};
template <>
struct TopicReceiveData<TopicReceive::MAIN_SONG> {
  using type = audio::Song;
};
template <>
struct TopicReceiveData<TopicReceive::DRIVE_PID> {
  using type = float[3 + 3];
};
template <>
struct TopicReceiveData<TopicReceive::DRIVE_CHASSIS_SPEEDS> {
  using type = drive::ChassisSpeeds;
};

template <TopicReceive Topic>
struct TopicReceiveData<Topic, void> {
  using type = uint8_t;
};

// Topics published by the robot.
enum class TopicSend : uint8_t {
  MAIN_TASK = FB_TOPIC_SEND_MAIN_TASK,
  MAIN_ERROR = FB_TOPIC_SEND_MAIN_ERROR,
  MAIN_SONG = FB_TOPIC_SEND_MAIN_SONG,
  MAIN_STATUS = FB_TOPIC_SEND_MAIN_STATUS,

  VISION_RAW_READINGS = FB_TOPIC_SEND_VISION_RAW_READINGS,
  VISION_DISTANCES = FB_TOPIC_SEND_VISION_DISTANCES,

  DRIVE_MOTOR_DATA = FB_TOPIC_SEND_DRIVE_MOTOR_DATA,
  DRIVE_IMU_DATA = FB_TOPIC_SEND_DRIVE_IMU_DATA,
  DRIVE_PID = FB_TOPIC_SEND_DRIVE_PID,
  DRIVE_CHASSIS_SPEEDS = FB_TOPIC_SEND_DRIVE_CHASSIS_SPEEDS,

  MAZE_CELL = FB_TOPIC_SEND_MAZE_CELL,
  MAZE_MOUSE_POSITION = FB_TOPIC_SEND_MAZE_MOUSE_POSITION,
};

template <TopicSend Topic, typename Default = void>
struct TopicSendData;

template <>
struct TopicSendData<TopicSend::MAIN_TASK> {
  using type = MainTaskData;
};
template <>
struct TopicSendData<TopicSend::MAIN_ERROR> {
  using type = robot::Error;
};
template <>
struct TopicSendData<TopicSend::MAIN_SONG> {
  using type = audio::Song;
};
template <>
struct TopicSendData<TopicSend::MAIN_STATUS> {
  using type = std::pair<robot::StatusTopic, uint8_t>;
};
template <>
struct TopicSendData<TopicSend::VISION_RAW_READINGS> {
  using type = std::array<float, 4>;
  static_assert(sizeof(type) == sizeof(float[4]));
};
template <>
struct TopicSendData<TopicSend::VISION_DISTANCES> {
  using type = std::array<units::millimeter_t, 4>;
  static_assert(sizeof(type) == sizeof(float[4]));
};
template <>
struct TopicSendData<TopicSend::DRIVE_MOTOR_DATA> {
  using type = hardware::Drivetrain::MotorData;
  static_assert(sizeof(type) == sizeof(float[4+3]));
};
template <>
struct TopicSendData<TopicSend::DRIVE_IMU_DATA> {
  using type = hardware::IMU::Data;
  static_assert(sizeof(type) == sizeof(float[3+3]));
};
template <>
struct TopicSendData<TopicSend::DRIVE_PID> {
  using type = float[3 + 3];
};
template <>
struct TopicSendData<TopicSend::DRIVE_CHASSIS_SPEEDS> {
  using type = drive::ChassisSpeeds;
};
template <>
struct TopicSendData<TopicSend::MAZE_CELL> {
  using type = std::pair<maze::Coordinate, maze::Cell>;
};
template <>
struct TopicSendData<TopicSend::MAZE_MOUSE_POSITION> {
  using type = maze::Coordinate;
};

template <TopicSend Topic>
struct TopicSendData<Topic, void> {
  using type = uint8_t;
};

}  // namespace feedback
