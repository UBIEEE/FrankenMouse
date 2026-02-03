#include "ROS2AppFeedback.h"

#import <AppKit/AppKit.h>
#import <Foundation/Foundation.h>
#import <SpriteKit/SpriteKit.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

static NSArray* toNSArray(std::vector<float> data) {
  size_t size = data.size();
  NSMutableArray* array = [NSMutableArray arrayWithCapacity:size];

  for (NSUInteger i = 0; i < size; i++) {
    NSNumber* number = [NSNumber numberWithFloat:data.at(i)];
    [array addObject:number];
  }
  return array;
}

static void getCoordinates(uint8_t index, uint8_t* x, uint8_t* y) {
  *x = index % 16;
  *y = index / 16;
}

union Cell {
  uint8_t byte;
  struct {
    uint8_t north : 1;
    uint8_t east : 1;
    uint8_t south : 1;
    uint8_t west : 1;
    uint8_t visited : 4;
  } bits;
};

class Feedback : public rclcpp::Node {
  ROS2AppFeedback* m_self;
  UpdateMainTaskCallback m_main_task_cb;
  AddMainErrorCallback m_add_error_cb;
  UpdateMainSongCallback m_main_song_cb;
  UpdateMainStatusCallback m_main_status_cb;
  UpdateVisionRawDataCallback m_vision_raw_readings_cb;
  UpdateVisionNormDataCallback m_vision_distances_cb;
  UpdateDriveMotorDataCallback m_drive_motor_data_cb;
  UpdateDriveIMUDataCallback m_drive_imu_data_cb;
  UpdateDrivePIDDataCallback m_drive_pid_data_cb;
  UpdateDriveChassisSpeedsCallback m_drive_chassis_speeds_cb;
  UpdateMazeCellCallback m_maze_cell_cb;
  UpdateMazeCoordinatesCallback m_maze_coord_cb;

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr m_main_task_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_main_command_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_main_song_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_vision_calibrate_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_drive_pid_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_drive_chassis_speeds_pub;

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr m_main_task_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr m_main_error_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr m_main_song_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr m_main_status_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_vision_raw_readings_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_vision_distances_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_drive_motor_data_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_drive_imu_data_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_drive_pid_data_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_drive_chassis_speeds_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr m_maze_cell_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr m_maze_coordinates_sub;

 public:
  Feedback(ROS2AppFeedback* _self,
          UpdateMainTaskCallback main_task_cb,
          AddMainErrorCallback add_error_cb,
          UpdateMainSongCallback main_song_cb,
          UpdateMainStatusCallback main_status_cb,
          UpdateVisionRawDataCallback vision_raw_readings_cb,
          UpdateVisionNormDataCallback vision_distances_cb,
          UpdateDriveMotorDataCallback drive_motor_data_cb,
          UpdateDriveIMUDataCallback drive_imu_data_cb,
          UpdateDrivePIDDataCallback drive_pid_data_cb,
          UpdateDriveChassisSpeedsCallback drive_chassis_speeds_cb,
          UpdateMazeCellCallback maze_cell_cb,
          UpdateMazeCoordinatesCallback maze_coord_cb)
      : Node("dashboard"),
        m_self(_self),
        m_main_task_cb(main_task_cb),
        m_add_error_cb(add_error_cb),
        m_main_song_cb(main_song_cb),
        m_main_status_cb(main_status_cb),
        m_vision_raw_readings_cb(vision_raw_readings_cb),
        m_vision_distances_cb(vision_distances_cb),
        m_drive_motor_data_cb(drive_motor_data_cb),
        m_drive_imu_data_cb(drive_imu_data_cb),
        m_drive_pid_data_cb(drive_pid_data_cb),
        m_drive_chassis_speeds_cb(drive_chassis_speeds_cb),
        m_maze_cell_cb(maze_cell_cb),
        m_maze_coord_cb(maze_coord_cb) {

    m_main_task_pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/client/main/task", 10);
    m_main_command_pub = this->create_publisher<std_msgs::msg::UInt8>("/client/main/command", 10);
    m_main_song_pub = this->create_publisher<std_msgs::msg::UInt8>("/client/main/song", 10);
    m_vision_calibrate_pub = this->create_publisher<std_msgs::msg::UInt8>("/client/vision/calibrate", 10);
    m_drive_pid_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/client/drive/pid", 10);
    m_drive_chassis_speeds_pub = this->create_publisher<geometry_msgs::msg::Twist>("/client/drive/chassis_speeds", 10);

    m_main_task_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/robot/main/task", 10, [this](const std_msgs::msg::UInt8MultiArray& msg) {
          if (m_main_task_cb && msg.data.size() == 2) {
            m_main_task_cb(m_self, msg.data[0], msg.data[1]);
          }
        });

    m_main_error_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/robot/main/error", 10, [this](const std_msgs::msg::UInt8MultiArray& msg) {
          if (m_add_error_cb && msg.data.size() == 6) {
            uint32_t timestamp = *reinterpret_cast<const uint32_t*>(&msg.data[0]);
            m_add_error_cb(m_self, timestamp, msg.data[4], msg.data[5]);
          }
        });

    m_main_song_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "/robot/main/song", 10, [this](const std_msgs::msg::UInt8& msg) {
          if (m_main_song_cb) {
            m_main_song_cb(m_self, msg.data);
          }
        });
          
    m_main_status_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/robot/main/status", 10, [this](const std_msgs::msg::UInt8MultiArray& msg) {
          if (m_main_status_cb && msg.data.size() == 2) {
            m_main_status_cb(m_self, msg.data[0], msg.data[1]);
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

    m_drive_chassis_speeds_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/robot/drive/chassis_speeds", 10, [this](const geometry_msgs::msg::Twist& msg) {
          if (m_drive_chassis_speeds_cb) {
            m_drive_chassis_speeds_cb(m_self, msg.linear.x, msg.angular.z);
          }
        });

    m_maze_cell_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/robot/maze/cell", 10, [this](const std_msgs::msg::UInt8MultiArray& msg) {
          if (m_maze_cell_cb && msg.data.size() == 2) {
            uint8_t x, y;
            getCoordinates(msg.data[0], &x, &y);
            Cell cell;
            cell.byte = msg.data[1];
            m_maze_cell_cb(m_self, x, y, cell.bits.north, cell.bits.east, cell.bits.south, cell.bits.west,
                           cell.bits.visited);
          }
        });

    m_maze_coordinates_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "/robot/maze/coordinates", 10, [this](const std_msgs::msg::UInt8& msg) {
          if (m_maze_coord_cb) {
            uint8_t x, y;
            getCoordinates(msg.data, &x, &y);
            m_maze_coord_cb(m_self, x, y);
          }
        });

  }

  void publish_main_task(uint8_t task, uint8_t starting_position) {
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = {task, starting_position};
    m_main_task_pub->publish(msg);
  }

  void publish_main_command(uint8_t command) {
    std_msgs::msg::UInt8 msg;
    msg.data = command;
    m_main_command_pub->publish(msg);
  }

  void publish_main_song(uint8_t song) {
    std_msgs::msg::UInt8 msg;
    msg.data = song;
    m_main_song_pub->publish(msg);
  }
  
  void publish_drive_pid(std::vector<float> values) {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = values;
    m_drive_pid_pub->publish(msg);
  }

  void publish_drive_chassis_speeds(float linear, float angular) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    m_drive_chassis_speeds_pub->publish(msg);
  }
};

static std::shared_ptr<Feedback> s_feedback;

BOOL ros2Init(ROS2AppFeedback* _self,
              UpdateMainTaskCallback main_task_cb,
              AddMainErrorCallback add_error_cb,
              UpdateMainSongCallback main_song_cb,
              UpdateMainStatusCallback main_status_cb,
              UpdateVisionRawDataCallback vision_raw_readings_cb,
              UpdateVisionNormDataCallback vision_distances_cb,
              UpdateDriveMotorDataCallback drive_motor_data_cb,
              UpdateDriveIMUDataCallback drive_imu_data_cb,
              UpdateDrivePIDDataCallback drive_pid_data_cb,
              UpdateDriveChassisSpeedsCallback drive_chassis_speeds_cb,
              UpdateMazeCellCallback maze_cell_cb,
              UpdateMazeCoordinatesCallback maze_coord_cb) {
  
  rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

  s_feedback = std::make_shared<Feedback>(
      _self, main_task_cb, add_error_cb, main_song_cb, main_status_cb, vision_raw_readings_cb,
      vision_distances_cb, drive_motor_data_cb, drive_imu_data_cb, drive_pid_data_cb,
      drive_chassis_speeds_cb, maze_cell_cb, maze_coord_cb);

  return YES;
}

void ros2PublishMainTask(uint8_t task, uint8_t starting_position) {
  if (s_feedback) {
    s_feedback->publish_main_task(task, starting_position);
  }
}

void ros2PublishMainCommand(uint8_t command) {
  if (s_feedback) {
    s_feedback->publish_main_command(command);
  }
}

void ros2PublishMainSong(uint8_t song) {
  if (s_feedback) {
    s_feedback->publish_main_song(song);
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

void ros2PublishDriveChassisSpeeds(float linear, float angular) {
  if (s_feedback) {
    s_feedback->publish_drive_chassis_speeds(linear, angular);
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

