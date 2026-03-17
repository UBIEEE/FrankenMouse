#include "hardware/feedback_impl.hpp"

#include "custom_stm.h"
#include "stm32wbxx_hal.h"

void FeedbackImpl::publish_topic(feedback::TopicSend topic, const uint8_t* _data) {
  // Custom_STM_App_Update_Char() takes uint8_t*, so get rid of const
  uint8_t* data = const_cast<uint8_t*>(_data);

  switch (topic) {
    using enum feedback::TopicSend;
    case MAIN_TASK:
      Custom_STM_App_Update_Char(CUSTOM_STM_MAIN_TASK_CHAR, data);
      break;
    case MAIN_ERROR:
      Custom_STM_App_Update_Char(CUSTOM_STM_MAIN_ERROR_CHAR, data);
      break;
    case MAIN_SONG:
      Custom_STM_App_Update_Char(CUSTOM_STM_MAIN_SONG_CHAR, data);
      break;
    case MAIN_BATTERY_VOLTAGE:
      Custom_STM_App_Update_Char(CUSTOM_STM_MAIN_BATTERYVOLTAGE_CHAR, data);
      break;
    case VISION_RAW_READINGS:
      Custom_STM_App_Update_Char(CUSTOM_STM_VISION_RAWREADINGS_CHAR, data);
      break;
    case VISION_DISTANCES:
      Custom_STM_App_Update_Char(CUSTOM_STM_VISION_DISTANCES_CHAR, data);
      break;
    case DRIVE_MOTOR_DATA:
      Custom_STM_App_Update_Char(CUSTOM_STM_DRIVE_DATA_CHAR, data);
      break;
    case DRIVE_IMU_DATA:
      Custom_STM_App_Update_Char(CUSTOM_STM_DRIVE_IMUDATA_CHAR, data);
      break;
    case DRIVE_PID:
      Custom_STM_App_Update_Char(CUSTOM_STM_DRIVE_PID_CHAR, data);
      break;
    case DRIVE_CHASSIS_SPEEDS:
      Custom_STM_App_Update_Char(CUSTOM_STM_DRIVE_CHASSISSPEEDS_CHAR, data);
      break;
    case MAZE_CELL:
      Custom_STM_App_Update_Char(CUSTOM_STM_MAZE_CELL_CHAR, data);
      break;
    case MAZE_MOUSE_POSITION:
      Custom_STM_App_Update_Char(CUSTOM_STM_MAZE_MOUSEPOS_CHAR, data);
      break;
  }
}

FeedbackImpl& get_mouse_v3_feedback() {
  static FeedbackImpl feedback;
  return feedback;
}

hardware::Feedback& get_platform_feedback() {
  return get_mouse_v3_feedback();
}
