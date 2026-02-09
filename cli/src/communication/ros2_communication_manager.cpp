#ifdef WITH_ROS2

#include <micromouse_cli/communication/ros2_communication_manager.hpp>

ROS2CommunicationManager::ROS2CommunicationManager() : rclcpp::Node("mm") {
  configure_publishers();
  configure_subscribers();
}

ROS2CommunicationManager::~ROS2CommunicationManager() {}

void ROS2CommunicationManager::configure_publishers() {
  m_main_task_pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/client/main/task", 10);
  m_main_command_pub = this->create_publisher<std_msgs::msg::UInt8>("/client/main/command", 10);
  m_main_song_pub = this->create_publisher<std_msgs::msg::UInt8>("/client/main/song", 10);
  m_vision_calibrate_pub = this->create_publisher<std_msgs::msg::UInt8>("/client/vision/calibrate", 10);
  m_drive_pid_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/client/drive/pid", 10);
  m_drive_chassis_speeds_pub =
      this->create_publisher<geometry_msgs::msg::Twist>("/client/drive/chassis_speeds", 10);
}

void ROS2CommunicationManager::configure_subscribers() {
  m_main_task_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/robot/main/task", 10, [this](const std_msgs::msg::UInt8MultiArray& msg) {
        if (msg.data.size() == 2) {
          RobotTask task = static_cast<RobotTask>(msg.data[0]);
          RobotStartPosition start_position = static_cast<RobotStartPosition>(msg.data[1]);
          m_main_data.task = task;
          m_main_data.start_position = start_position;
        }
      });

  m_main_error_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/robot/main/error", 10, [this](const std_msgs::msg::UInt8MultiArray& msg) {
        if (msg.data.size() == 6) {
          uint32_t timestamp = *reinterpret_cast<const uint32_t*>(&msg.data[0]);
          RobotErrorCategory category = static_cast<RobotErrorCategory>(msg.data[4]);
          uint8_t code = msg.data[5];
          m_main_data.errors[timestamp] = RobotError{timestamp, category, code};
        }
      });

  m_main_song_sub = this->create_subscription<std_msgs::msg::UInt8>(
      "/robot/main/song", 10, [this](const std_msgs::msg::UInt8& msg) {
        RobotSong song = static_cast<RobotSong>(msg.data);
        m_main_data.song = song;
      });

  m_main_status_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/robot/main/status", 10, [this](const std_msgs::msg::UInt8MultiArray& msg) {
        if (msg.data.size() == 2) {
          RobotStatusTopic topic = static_cast<RobotStatusTopic>(msg.data[0]);
          uint8_t value = msg.data[1];
          m_main_data.statusTopics[topic] = value;
        }
      });

  m_vision_raw_readings_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/robot/vision/raw_readings", 10, [this](const std_msgs::msg::Float32MultiArray& msg) {
        if (msg.data.size() == 4) {
          std::copy(msg.data.begin(), msg.data.end(), m_vision_data.raw_readings);
        }
      });

  m_vision_distances_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/robot/vision/distances", 10, [this](const std_msgs::msg::Float32MultiArray& msg) {
        if (msg.data.size() == 4) {
          std::copy(msg.data.begin(), msg.data.end(), m_vision_data.distances);
        }
      });

  m_drive_motor_data_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/robot/drive/motors", 10, [this](const std_msgs::msg::Float32MultiArray& msg) {
        if (msg.data.size() == 7) {
          std::copy(msg.data.begin(), msg.data.end(), m_drive_data.motor_data);
        }
      });

  m_drive_imu_data_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/robot/drive/imu", 10, [this](const std_msgs::msg::Float32MultiArray& msg) {
        if (msg.data.size() == 6) {
          std::copy(msg.data.begin(), msg.data.end(), m_drive_data.imu_data);
        }
      });

  m_drive_pid_data_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/robot/drive/pid", 10, [this](const std_msgs::msg::Float32MultiArray& msg) {
        if (msg.data.size() == 6) {
          std::copy(msg.data.begin(), msg.data.end(), m_drive_data.pid_data);
        }
      });

  m_drive_chassis_speeds_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/robot/drive/chassis_speeds", 10, [this](const geometry_msgs::msg::Twist& msg) {
        m_drive_data.chassis_speeds.linear_velocity_mmps = msg.linear.x;
        m_drive_data.chassis_speeds.angular_velocity_dps = msg.angular.z;
      });

  m_maze_cell_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/robot/maze/cell", 10, [this](const std_msgs::msg::UInt8MultiArray& msg) {
        if (msg.data.size() == 2) {
          Coordinate coord(msg.data[0]);
          Cell cell(msg.data[1]);
          m_maze_data.cells[coord] = cell;
        }
      });

  m_maze_coordinates_sub = this->create_subscription<std_msgs::msg::UInt8>(
      "/robot/maze/coordinates", 10, [this](const std_msgs::msg::UInt8& msg) {
        Coordinate coord(msg.data);
        m_maze_data.mouse_position = coord;
      });
}

void ROS2CommunicationManager::write_topic(FeedbackTopicWrite topic_id, const uint8_t* data, size_t size) {
  switch (topic_id) {
    using enum FeedbackTopicWrite;
    case MAIN_TASK:
      if (size == 2) {
        std_msgs::msg::UInt8MultiArray msg;
        msg.data = {data[0], data[1]};
        m_main_task_pub->publish(msg);
      }
      break;
    case MAIN_COMMAND:
      if (size == 1) {
        std_msgs::msg::UInt8 msg;
        msg.data = data[0];
        m_main_command_pub->publish(msg);
      }
      break;
    case MAIN_SONG:
      if (size == 1) {
        std_msgs::msg::UInt8 msg;
        msg.data = data[0];
        m_main_song_pub->publish(msg);
      }
      break;
    case DRIVE_PID:
      if (size == 6 * sizeof(float)) {
        std_msgs::msg::Float32MultiArray msg;
        msg.data.resize(6);
        std::memcpy(msg.data.data(), data, size);
        m_drive_pid_pub->publish(msg);
      }
      break;
    case DRIVE_CHASSIS_SPEEDS:
      if (size == sizeof(drive::ChassisSpeeds)) {
        const drive::ChassisSpeeds& speeds = *reinterpret_cast<const drive::ChassisSpeeds*>(data);
        geometry_msgs::msg::Twist msg;
        msg.linear.x = speeds.linear_velocity_mmps;
        msg.angular.z = speeds.angular_velocity_dps;
        m_drive_chassis_speeds_pub->publish(msg);
      }
      break;
  }
}

#endif
