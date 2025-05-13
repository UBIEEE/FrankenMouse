#include <simulation/hardware/feedback_impl.hpp>

#include <micromouse/robot.hpp>

using namespace std::placeholders;

FeedbackImpl::FeedbackImpl() : Node("micromouse_feedback") {
  m_main_task_sub = this->create_subscription<std_msgs::msg::UInt8>(
      "/dashboard/main/task", 10, [this](const std_msgs::msg::UInt8& msg) {
        Robot::get().delegate_received_feedback(FeedbackTopicReceive::MAIN_TASK,
                                                &msg.data);
      });

  m_main_ready_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/dashboard/main/ready", 10, [this](const std_msgs::msg::Bool& msg) {
        Robot::get().publish_extra_feedback();
      });

  m_drive_pid_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/dashboard/drive/pid", 10,
      [this](const std_msgs::msg::Float32MultiArray& msg) { (void)msg; });

  m_vision_calibrate_sub = this->create_subscription<std_msgs::msg::UInt8>(
      "/dashboard/vision/calibrate", 10,
      [this](const std_msgs::msg::UInt8& msg) {
        Robot::get().delegate_received_feedback(
            FeedbackTopicReceive::VISION_CALIBRATE, &msg.data);
      });

  m_maze_reset_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/dashboard/maze/reset", 10, [this](const std_msgs::msg::Bool& msg) {
        uint8_t data = msg.data;
        Robot::get().delegate_received_feedback(
            FeedbackTopicReceive::MAZE_RESET, &data);
      });

  m_music_song_sub = this->create_subscription<std_msgs::msg::UInt8>(
      "/dashboard/music/song", 10, [this](const std_msgs::msg::UInt8& msg) {
        Robot::get().delegate_received_feedback(
            FeedbackTopicReceive::MUSIC_PLAY_SONG, &msg.data);
      });

  m_main_task_pub =
      this->create_publisher<std_msgs::msg::UInt8>("/robot/main/task", 10);
  m_main_error_pub =
      this->create_publisher<std_msgs::msg::UInt8>("/robot/main/error", 10);
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
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
          "/robot/drive/chassis_speeds", 10);
  m_vision_raw_readings_pub =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
          "/robot/vision/raw_readings", 10);
  m_vision_distances_pub =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
          "/robot/vision/distances", 10);
  m_vision_is_calibrated_pub = this->create_publisher<std_msgs::msg::Bool>(
      "/robot/vision/is_calibrated", 10);
  m_music_playing_pub = this->create_publisher<std_msgs::msg::Bool>(
      "/robot/music/is_playing", 10);
}

void FeedbackImpl::publish_topic(FeedbackTopicSend topic, uint8_t* data) {
  switch (topic) {
    using enum FeedbackTopicSend;
    case MAIN_TASK: {
      std_msgs::msg::UInt8 msg;
      msg.data = data[0];
      m_main_task_pub->publish(msg);
    } break;
    case MAIN_ERROR: {
      std_msgs::msg::UInt8 msg;
      msg.data = data[0];
      m_main_error_pub->publish(msg);
    } break;
    case DRIVE_MOTOR_DATA: {
      std_msgs::msg::Float32MultiArray msg;
      msg.data = std::vector<float>((float*)data, (float*)data + 7);
      m_drive_motor_data_pub->publish(msg);
    } break;
    case DRIVE_IMU_DATA: {
      std_msgs::msg::Float32MultiArray msg;
      msg.data = std::vector<float>((float*)data, (float*)data + 6);
      m_drive_imu_data_pub->publish(msg);
    } break;
    case DRIVE_PID: {
      std_msgs::msg::Float32MultiArray msg;
      msg.data = std::vector<float>((float*)data, (float*)data + 6);
      m_drive_pid_data_pub->publish(msg);
    } break;
    case DRIVE_CHASSIS_SPEEDS: {
      std_msgs::msg::Float32MultiArray msg;
      msg.data = std::vector<float>((float*)data, (float*)data + 2);
      m_chassis_speeds_pub->publish(msg);
    }
    case VISION_RAW_READINGS: {
      std_msgs::msg::Float32MultiArray msg;
      msg.data = std::vector<float>((float*)data, (float*)data + 4);
      m_vision_raw_readings_pub->publish(msg);
    } break;
    case VISION_DISTANCES: {
      std_msgs::msg::Float32MultiArray msg;
      msg.data = std::vector<float>((float*)data, (float*)data + 4);
      m_vision_distances_pub->publish(msg);
    } break;
    case VISION_CALIBRATE: {
      std_msgs::msg::Bool msg;
      msg.data = data[0];
      m_vision_is_calibrated_pub->publish(msg);
    } break;
    case MUSIC_IS_PLAYING: {
      std_msgs::msg::Bool msg;
      msg.data = data[0];
      m_music_playing_pub->publish(msg);
    } break;
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
