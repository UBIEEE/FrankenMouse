#pragma once

#include <Foundation/Foundation.h>

@class ROS2AppFeedback;

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*UpdateMainTaskCallback)(ROS2AppFeedback*, uint8_t, uint8_t);
typedef void (*AddMainErrorCallback)(ROS2AppFeedback*, uint32_t, uint8_t, uint8_t);
typedef void (*UpdateMainSongCallback)(ROS2AppFeedback*, uint8_t);
typedef void (*UpdateMainStatusCallback)(ROS2AppFeedback*, uint8_t, float);
typedef void (*UpdateMainBatteryVoltageCallback)(ROS2AppFeedback*, float);
typedef void (*UpdateVisionRawDataCallback)(ROS2AppFeedback*, NSArray*);
typedef void (*UpdateVisionNormDataCallback)(ROS2AppFeedback*, NSArray*);
typedef void (*UpdateDriveMotorDataCallback)(ROS2AppFeedback*, NSArray*);
typedef void (*UpdateDriveIMUDataCallback)(ROS2AppFeedback*, NSArray*);
typedef void (*UpdateDrivePIDDataCallback)(ROS2AppFeedback*, NSArray*);
typedef void (*UpdateDriveChassisSpeedsCallback)(ROS2AppFeedback*, float, float);
typedef void (*UpdateMazeCellCallback)(ROS2AppFeedback*, uint8_t, uint8_t, BOOL, BOOL, BOOL, BOOL, BOOL);
typedef void (*UpdateMazeCoordinatesCallback)(ROS2AppFeedback*, uint8_t, uint8_t);

BOOL ros2Init(ROS2AppFeedback* _self,
              UpdateMainTaskCallback main_task_cb,
              AddMainErrorCallback add_error_cb,
              UpdateMainSongCallback main_song_cb,
              UpdateMainStatusCallback main_status_cb,
              UpdateMainBatteryVoltageCallback main_battery_voltage_cb,
              UpdateVisionRawDataCallback vision_raw_readings_cb,
              UpdateVisionNormDataCallback vision_distances_cb,
              UpdateDriveMotorDataCallback drive_motor_data_cb,
              UpdateDriveIMUDataCallback drive_imu_data_cb,
              UpdateDrivePIDDataCallback drive_pid_data_cb,
              UpdateDriveChassisSpeedsCallback drive_chassis_speeds_cb,
              UpdateMazeCellCallback maze_cell_cb,
              UpdateMazeCoordinatesCallback maze_coord_cb);

void ros2PublishMainTask(uint8_t task, uint8_t starting_position);
void ros2PublishMainCommand(uint8_t command);
void ros2PublishMainSong(uint8_t song);
void ros2PublishDrivePID(NSArray* values);
void ros2PublishDriveChassisSpeeds(float linear, float angular);

void ros2Process(void);
void ros2Destroy(void);

#ifdef __cplusplus
}
#endif

