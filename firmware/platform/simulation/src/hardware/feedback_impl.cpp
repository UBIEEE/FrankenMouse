#include <simulation/hardware/drivetrain_impl.hpp>
#include <simulation/hardware/feedback_impl.hpp>

#include <micromouse/robot/robot.hpp>

using namespace std::placeholders;

using namespace robot;

FeedbackImpl::FeedbackImpl() : Node("micromouse_feedback") {
  m_main_task_sub = this->create_subscription<std_msgs::msg::UInt8>(
      "/client/main/task", 10, [this](const std_msgs::msg::UInt8& msg) {
        Robot::get().delegate_received_feedback(feedback::TopicReceive::MAIN_TASK,
                                                &msg.data);
      });

  m_main_command_sub = this->create_subscription<std_msgs::msg::UInt8>(
      "/client/main/command", 10, [this](const std_msgs::msg::UInt8& msg) {
        Robot::get().delegate_received_feedback(
            feedback::TopicReceive::MAIN_COMMAND, &msg.data);
      });

  m_main_song_sub = this->create_subscription<std_msgs::msg::UInt8>(
      "/client/main/song", 10, [this](const std_msgs::msg::UInt8& msg) {
        Robot::get().delegate_received_feedback(feedback::TopicReceive::MAIN_SONG,
                                                &msg.data);
      });

  m_drive_pid_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/client/drive/pid", 10,
      [this](const std_msgs::msg::Float32MultiArray& msg) {
        (void)msg;
        // No PID needed in simulation!
      });

  m_drive_chassis_speeds_sub =
      this->create_subscription<geometry_msgs::msg::Twist>(
          "/client/drive/chassis_speeds", 10,
          [this](const geometry_msgs::msg::Twist& msg) {
            const float speeds[6] = {
                static_cast<float>(msg.linear.x),
                static_cast<float>(msg.linear.y),
                static_cast<float>(msg.linear.z),
                static_cast<float>(msg.angular.x),
                static_cast<float>(msg.angular.y),
                static_cast<float>(msg.angular.z),
            };

            Robot::get().delegate_received_feedback(
                feedback::TopicReceive::DRIVE_CHASSIS_SPEEDS,
                reinterpret_cast<const uint8_t*>(speeds));
          });

  m_main_task_pub =
      this->create_publisher<std_msgs::msg::UInt8>("/robot/main/task", 10);
  m_main_error_pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
      "/robot/main/error", 10);
  m_main_song_pub =
      this->create_publisher<std_msgs::msg::UInt8>("/robot/main/song", 10);
  m_main_status_pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
      "/robot/main/status", 10);
  m_vision_raw_readings_pub =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
          "/robot/vision/raw_readings", 10);
  m_vision_distances_pub =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
          "/robot/vision/distances", 10);
  m_drive_motor_data_pub =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
          "/robot/drive/motors", 10);
  m_drive_imu_data_pub =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
          "/robot/drive/imu", 10);
  m_drive_pid_data_pub =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
          "/robot/drive/pid", 10);
  m_drive_chassis_speeds_pub =
      this->create_publisher<geometry_msgs::msg::Twist>(
          "/robot/drive/chassis_speeds", 10);
  m_maze_cell_pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
      "/robot/maze/cell", 10);
  m_maze_coordinates_pub = this->create_publisher<std_msgs::msg::UInt8>(
      "/robot/maze/coordinates", 10);
}

void FeedbackImpl::publish_topic(feedback::TopicSend topic, const uint8_t* data) {
  switch (topic) {
    using enum feedback::TopicSend;
    case MAIN_TASK: {
      std_msgs::msg::UInt8 msg;
      msg.data = data[0];
      m_main_task_pub->publish(msg);
      break;
    }
    case MAIN_ERROR: {
      std_msgs::msg::UInt8MultiArray msg;
      msg.data = std::vector<uint8_t>(data, data + 6);
      m_main_error_pub->publish(msg);
      break;
    }
    case MAIN_SONG: {
      std_msgs::msg::UInt8 msg;
      msg.data = data[0];
      m_main_song_pub->publish(msg);
      break;
    }
    case MAIN_STATUS: {
      std_msgs::msg::UInt8MultiArray msg;
      msg.data = std::vector<uint8_t>(data, data + 2);
      m_main_status_pub->publish(msg);
      break;
    }
    case VISION_RAW_READINGS: {
      std_msgs::msg::Float32MultiArray msg;
      msg.data = std::vector<float>((float*)data, (float*)data + 4);
      m_vision_raw_readings_pub->publish(msg);
      break;
    }
    case VISION_DISTANCES: {
      std_msgs::msg::Float32MultiArray msg;
      msg.data = std::vector<float>((float*)data, (float*)data + 4);
      m_vision_distances_pub->publish(msg);
      break;
    }
    case DRIVE_MOTOR_DATA: {
      std_msgs::msg::Float32MultiArray msg;
      msg.data = std::vector<float>((float*)data, (float*)data + 7);
      m_drive_motor_data_pub->publish(msg);
      break;
    }
    case DRIVE_IMU_DATA: {
      std_msgs::msg::Float32MultiArray msg;
      msg.data = std::vector<float>((float*)data, (float*)data + 6);
      m_drive_imu_data_pub->publish(msg);
      break;
    }
    case DRIVE_PID: {
      std_msgs::msg::Float32MultiArray msg;
      msg.data = std::vector<float>((float*)data, (float*)data + 6);
      m_drive_pid_data_pub->publish(msg);
      break;
    }
    case DRIVE_CHASSIS_SPEEDS: {
      geometry_msgs::msg::Twist msg;

      const float* float_data = reinterpret_cast<const float*>(data);
      msg.linear.x = float_data[0];
      msg.linear.y = float_data[1];
      msg.linear.z = float_data[2];
      msg.angular.x = float_data[3];
      msg.angular.y = float_data[4];
      msg.angular.z = float_data[5];

      m_drive_chassis_speeds_pub->publish(msg);
      break;
    }
    case MAZE_CELL: {
      std_msgs::msg::UInt8MultiArray msg;
      msg.data = std::vector<uint8_t>(data, data + 2);
      m_maze_cell_pub->publish(msg);
      break;
    }
    case MAZE_MOUSE_POSITION: {
      std_msgs::msg::UInt8 msg;
      msg.data = data[0];
      m_maze_coordinates_pub->publish(msg);
      break;
    }
    default:
      // TODO:
      break;
  }
}

std::shared_ptr<FeedbackImpl> get_simulation_feedback() {
  static auto feedback = std::make_shared<FeedbackImpl>();
  return feedback;
}

hardware::Feedback& get_platform_feedback() {
  return *get_simulation_feedback();
}
