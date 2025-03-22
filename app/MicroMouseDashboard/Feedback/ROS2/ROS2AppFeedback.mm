#include "ROS2AppFeedback.h"

#import <AppKit/AppKit.h>
#import <Foundation/Foundation.h>
#import <SpriteKit/SpriteKit.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

static NSArray* toNSArray(std::vector<float> data) {
  size_t size = data.size();
  NSMutableArray* array = [NSMutableArray arrayWithCapacity:size];

  for (NSUInteger i = 0; i < size; i++) {
    NSNumber* number = [NSNumber numberWithFloat:data.at(i)];
    [array addObject:number];
  }
  return array;
}

class Feedback : public rclcpp::Node {
  ROS2AppFeedback* m_self;
  UpdateMainTaskCallback m_main_task_cb;
  AddErrorCallback m_add_error_cb;
  UpdateDriveMotorDataCallback m_drive_motor_data_cb;
  UpdateDriveIMUDataCallback m_drive_imu_data_cb;
  UpdateDrivePIDDataCallback m_drive_pid_data_cb;
  UpdateVisionRawDataCallback m_vision_raw_readings_cb;
  UpdateVisionNormDataCallback m_vision_distances_cb;
  UpdateVisionIsCalibratedCallback m_vision_is_calibrated_cb;
  UpdateMusicPlayingCallback m_music_playing_cb;

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_main_task_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_main_ready_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_drive_pid_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_vision_calibrate_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_maze_reset_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_music_song_pub;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr m_main_task_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr m_main_error_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_drive_motor_data_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_drive_imu_data_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_drive_pid_data_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_vision_raw_readings_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_vision_distances_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_vision_is_calibrated_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_music_playing_sub;

 public:
  Feedback(ROS2AppFeedback* _self, UpdateMainTaskCallback main_task_cb,
           AddErrorCallback add_error_cb, UpdateDriveMotorDataCallback drive_motor_data_cb,
           UpdateDriveIMUDataCallback drive_imu_data_cb,
           UpdateDrivePIDDataCallback drive_pid_data_cb,
           UpdateVisionRawDataCallback vision_raw_readings_cb,
           UpdateVisionNormDataCallback vision_distances_cb,
           UpdateVisionIsCalibratedCallback vision_is_calibrated_cb,
           UpdateMusicPlayingCallback music_playing_cb)
      : Node("dashboard"),
        m_self(_self),
        m_main_task_cb(main_task_cb),
        m_add_error_cb(add_error_cb),
        m_drive_motor_data_cb(drive_motor_data_cb),
        m_drive_imu_data_cb(drive_imu_data_cb),
        m_drive_pid_data_cb(drive_pid_data_cb),
        m_vision_raw_readings_cb(vision_raw_readings_cb),
        m_vision_distances_cb(vision_distances_cb),
        m_vision_is_calibrated_cb(vision_is_calibrated_cb),
        m_music_playing_cb(music_playing_cb) {
    m_main_task_pub = this->create_publisher<std_msgs::msg::UInt8>("/dashboard/main/task", 10);
    m_main_ready_pub = this->create_publisher<std_msgs::msg::Bool>("/dashboard/main/ready", 10);
    m_drive_pid_pub =
        this->create_publisher<std_msgs::msg::Float32MultiArray>("/dashboard/drive/pid", 10);
    m_vision_calibrate_pub =
        this->create_publisher<std_msgs::msg::UInt8>("/dashboard/vision/calibrate", 10);
    m_maze_reset_pub = this->create_publisher<std_msgs::msg::Bool>("/dashboard/maze/reset", 10);
    m_music_song_pub = this->create_publisher<std_msgs::msg::UInt8>("/dashboard/music/song", 10);

    m_main_task_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "/robot/main/task", 10, [this](const std_msgs::msg::UInt8& msg) {
          if (m_main_task_cb) {
            m_main_task_cb(m_self, msg.data);
          }
        });

    m_main_error_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "/robot/main/error", 10, [this](const std_msgs::msg::UInt8& msg) {
          if (m_add_error_cb) {
            m_add_error_cb(m_self, msg.data);
          }
        });

    m_drive_motor_data_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/robot/drive/motors", 10, [this](const std_msgs::msg::Float32MultiArray& msg) {
          if (m_drive_motor_data_cb) {
            m_drive_motor_data_cb(m_self, toNSArray(msg.data));
          }
        });

    m_drive_imu_data_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/robot/drive/imu", 10, [this](const std_msgs::msg::Float32MultiArray& msg) {
          if (m_drive_imu_data_cb) {
            m_drive_imu_data_cb(m_self, toNSArray(msg.data));
          }
        });

    m_drive_pid_data_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/robot/drive/pid", 10, [this](const std_msgs::msg::Float32MultiArray& msg) {
          if (m_drive_pid_data_cb) {
            m_drive_pid_data_cb(m_self, toNSArray(msg.data));
          }
        });

    m_vision_raw_readings_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/robot/vision/raw_readings", 10, [this](const std_msgs::msg::Float32MultiArray& msg) {
          if (m_vision_raw_readings_cb) {
            m_vision_raw_readings_cb(m_self, toNSArray(msg.data));
          }
        });

    m_vision_distances_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/robot/vision/distances", 10,
        [this](const std_msgs::msg::Float32MultiArray& msg) {
          if (m_vision_distances_cb) {
            m_vision_distances_cb(m_self, toNSArray(msg.data));
          }
        });

    m_vision_is_calibrated_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/robot/vision/is_calibrated", 10, [this](const std_msgs::msg::Bool& msg) {
          if (m_vision_is_calibrated_cb) {
            m_vision_is_calibrated_cb(m_self, msg.data);
          }
        });

    m_music_playing_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/robot/music/is_playing", 10, [this](const std_msgs::msg::Bool& msg) {
          if (m_music_playing_cb) {
            m_music_playing_cb(m_self, msg.data);
          }
        });
  }

  void publish_main_task(uint8_t task) {
    std_msgs::msg::UInt8 msg;
    msg.data = task;
    m_main_task_pub->publish(msg);
  }

  void publish_app_ready() {
    std_msgs::msg::Bool msg;
    msg.data = true;
    m_main_ready_pub->publish(msg);
  }

  void publish_drive_pid(std::vector<float> values) {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = values;
    m_drive_pid_pub->publish(msg);
  }

  void publish_vision_calibrate() {
    std_msgs::msg::UInt8 msg;
    msg.data = 1;
    m_vision_calibrate_pub->publish(msg);
  }

  void publish_vision_calibrate_reset() {
    std_msgs::msg::UInt8 msg;
    msg.data = 0;
    m_vision_calibrate_pub->publish(msg);
  }

  void publish_maze_reset() {
    std_msgs::msg::Bool msg;
    msg.data = true;
    m_maze_reset_pub->publish(msg);
  }

  void publish_music_song(uint8_t song) {
    std_msgs::msg::UInt8 msg;
    msg.data = song;
    m_music_song_pub->publish(msg);
  }
};

static std::shared_ptr<Feedback> s_feedback;

BOOL ros2Init(ROS2AppFeedback* _self, UpdateMainTaskCallback main_task_cb,
              AddErrorCallback add_error_cb, UpdateDriveMotorDataCallback drive_motor_data_cb,
              UpdateDriveIMUDataCallback drive_imu_data_cb,
              UpdateDrivePIDDataCallback drive_pid_data_cb,
              UpdateVisionRawDataCallback vision_raw_data_cb,
              UpdateVisionNormDataCallback vision_norm_data_cb,
              UpdateVisionIsCalibratedCallback vision_is_calibrated_cb,
              UpdateMusicPlayingCallback music_playing_cb) {
  rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

  s_feedback = std::make_shared<Feedback>(
      _self, main_task_cb, add_error_cb, drive_motor_data_cb, drive_imu_data_cb, drive_pid_data_cb,
      vision_raw_data_cb, vision_norm_data_cb, vision_is_calibrated_cb, music_playing_cb);

  return YES;
}

void ros2PublishMainTask(uint8_t task, uint8_t starting_position) {
  if (s_feedback) {
    s_feedback->publish_main_task(task);
  }
}

void ros2PublishAppReady(void) {
  if (s_feedback) {
    s_feedback->publish_app_ready();
  }
}

void ros2PublishDrivePID(NSArray* values) {
  if (s_feedback) {
    std::vector<float> pid_values;
    for (NSNumber* number in values) {
      pid_values.push_back([number floatValue]);
    }
    s_feedback->publish_drive_pid(pid_values);
  }
}

void ros2PublishVisionCalibrate(void) {
  if (s_feedback) {
    s_feedback->publish_vision_calibrate();
  }
}

void ros2PublishVisionCalibrateReset(void) {
  if (s_feedback) {
    s_feedback->publish_vision_calibrate_reset();
  }
}

void ros2PublishMazeReset(void) {
  if (s_feedback) {
    s_feedback->publish_maze_reset();
  }
}

void ros2PublishMusicSong(uint8_t song) {
  if (s_feedback) {
    s_feedback->publish_music_song(song);
  }
}

void ros2Process(void) {
  if (s_feedback) {
    rclcpp::spin_some(s_feedback);
  }
}

void ros2Destroy(void) {
  if (s_feedback) {
    rclcpp::shutdown();
  }
  s_feedback = nullptr;
}

