#pragma once

#include <Foundation/Foundation.h>

@class ROS2AppFeedback;

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*UpdateMainTaskCallback)(ROS2AppFeedback*, uint8_t);
typedef void (*AddErrorCallback)(ROS2AppFeedback*, uint8_t);
typedef void (*UpdateDriveMotorDataCallback)(ROS2AppFeedback*, NSArray*);
typedef void (*UpdateDriveIMUDataCallback)(ROS2AppFeedback*, NSArray*);
typedef void (*UpdateDrivePIDDataCallback)(ROS2AppFeedback*, NSArray*);
typedef void (*UpdateVisionRawDataCallback)(ROS2AppFeedback*, NSArray*);
typedef void (*UpdateVisionNormDataCallback)(ROS2AppFeedback*, NSArray*);
typedef void (*UpdateVisionIsCalibratedCallback)(ROS2AppFeedback*, BOOL);
typedef void (*UpdateMusicPlayingCallback)(ROS2AppFeedback*, BOOL);

BOOL ros2Init(ROS2AppFeedback* _self, UpdateMainTaskCallback main_task_cb,
              AddErrorCallback add_error_cb, UpdateDriveMotorDataCallback drive_motor_data_cb,
              UpdateDriveIMUDataCallback drive_imu_data_cb,
              UpdateDrivePIDDataCallback drive_pid_data_cb,
              UpdateVisionRawDataCallback vision_raw_data_cb,
              UpdateVisionNormDataCallback vision_norm_data_cb,
              UpdateVisionIsCalibratedCallback vision_is_calibrated_cb,
              UpdateMusicPlayingCallback music_playing_cb);

void ros2PublishMainTask(uint8_t task, uint8_t starting_position);
void ros2PublishAppReady(void);
void ros2PublishDrivePID(NSArray* values);
void ros2PublishVisionCalibrate(void);
void ros2PublishVisionCalibrateReset(void);
void ros2PublishMazeReset(void);
void ros2PublishMusicSong(uint8_t song);

void ros2Process(void);
void ros2Destroy(void);

#ifdef __cplusplus
}
#endif

