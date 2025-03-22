#include "hardware/feedback_impl.hpp"

#include "custom_stm.h"
#include "stm32wbxx_hal.h"

void FeedbackImpl::publish_topic(FeedbackTopicSend topic, uint8_t* data) {
  switch (topic) {
    using enum FeedbackTopicSend;
  MAIN_TASK:
    Custom_STM_App_Update_Char(CUSTOM_STM_MAIN_TASK_CHAR, data);
    break;
  MAIN_ERROR:
    Custom_STM_App_Update_Char(CUSTOM_STM_MAIN_ERRORCODE_CHAR, data);
    break;
  DRIVE_MOTOR_DATA:
    Custom_STM_App_Update_Char(CUSTOM_STM_DRIVE_DATA_CHAR, data);
    break;
  DRIVE_IMU_DATA:
    Custom_STM_App_Update_Char(CUSTOM_STM_DRIVE_IMUDATA_CHAR, data);
    break;
  DRIVE_PID:
    Custom_STM_App_Update_Char(CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR, data);
    break;
  VISION_RAW_READINGS:
    Custom_STM_App_Update_Char(CUSTOM_STM_VISION_RAWREADINGS_CHAR, data);
    break;
  VISION_DISTANCES:
    Custom_STM_App_Update_Char(CUSTOM_STM_VISION_DISTANCES_CHAR, data);
    break;
  VISION_CALIBRATE:
    Custom_STM_App_Update_Char(CUSTOM_STM_VISION_CALIBRATE_CHAR, data);
    break;
  MAZE_CELL:
    Custom_STM_App_Update_Char(CUSTOM_STM_MAZE_CELL_CHAR, data);
    break;
  MAZE_MOUSE_POSITION:
    Custom_STM_App_Update_Char(CUSTOM_STM_MAZE_MOUSEPOS_CHAR, data);
    break;
  MUSIC_IS_PLAYING:
    Custom_STM_App_Update_Char(CUSTOM_STM_MUSIC_ISPLAYING_CHAR, data);
    break;
  }
}

FeedbackImpl& get_mouse_v2_feedback() {
  static FeedbackImpl feedback;
  return feedback;
}

hardware::Feedback& get_platform_feedback() {
  return get_mouse_v2_feedback();
}
