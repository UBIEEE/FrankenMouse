/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.h
  * @author  MCD Application Team
  * @brief   Header for custom_stm.c module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CUSTOM_STM_H
#define CUSTOM_STM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ble_types.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  /* musicService */
  CUSTOM_STM_MUSIC_PLAYSONG_CHAR,
  CUSTOM_STM_MUSIC_ISPLAYING_CHAR,
  /* visionService */
  CUSTOM_STM_VISION_RAWREADINGS_CHAR,
  CUSTOM_STM_VISION_DISTANCES_CHAR,
  CUSTOM_STM_VISION_CALIBRATE_CHAR,
  /* mainService */
  CUSTOM_STM_MAIN_TASK_CHAR,
  CUSTOM_STM_MAIN_APPREADY_CHAR,
  CUSTOM_STM_MAIN_ERRORCODE_CHAR,
  /* driveService */
  CUSTOM_STM_DRIVE_DATA_CHAR,
  CUSTOM_STM_DRIVE_IMUDATA_CHAR,
  CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR,
  CUSTOM_STM_DRIVE_CHASSISSPEEDS_CHAR,
  /* mazeService */
  CUSTOM_STM_MAZE_RESET_CHAR,
  CUSTOM_STM_MAZE_CELL_CHAR,
  CUSTOM_STM_MAZE_MOUSEPOS_CHAR,
} Custom_STM_Char_Opcode_t;

typedef enum
{
  /* music_playSong_writeChar */
  CUSTOM_STM_MUSIC_PLAYSONG_CHAR_WRITE_EVT,
  /* music_isPlaying_notifyChar */
  CUSTOM_STM_MUSIC_ISPLAYING_CHAR_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_MUSIC_ISPLAYING_CHAR_NOTIFY_DISABLED_EVT,
  /* vision_rawReadings_notifyChar */
  CUSTOM_STM_VISION_RAWREADINGS_CHAR_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_VISION_RAWREADINGS_CHAR_NOTIFY_DISABLED_EVT,
  /* vision_distances_notifyChar */
  CUSTOM_STM_VISION_DISTANCES_CHAR_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_VISION_DISTANCES_CHAR_NOTIFY_DISABLED_EVT,
  /* vision_calibrate_writeNotifyChar */
  CUSTOM_STM_VISION_CALIBRATE_CHAR_WRITE_EVT,
  CUSTOM_STM_VISION_CALIBRATE_CHAR_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_VISION_CALIBRATE_CHAR_NOTIFY_DISABLED_EVT,
  /* main_task_writeNotifyChar */
  CUSTOM_STM_MAIN_TASK_CHAR_WRITE_EVT,
  CUSTOM_STM_MAIN_TASK_CHAR_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_MAIN_TASK_CHAR_NOTIFY_DISABLED_EVT,
  /* main_appReady_writeChar */
  CUSTOM_STM_MAIN_APPREADY_CHAR_WRITE_EVT,
  /* main_errorCode_notifyChar */
  CUSTOM_STM_MAIN_ERRORCODE_CHAR_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_MAIN_ERRORCODE_CHAR_NOTIFY_DISABLED_EVT,
  /* drive_data_notifyChar */
  CUSTOM_STM_DRIVE_DATA_CHAR_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_DRIVE_DATA_CHAR_NOTIFY_DISABLED_EVT,
  /* drive_imuData_notifyChar */
  CUSTOM_STM_DRIVE_IMUDATA_CHAR_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_DRIVE_IMUDATA_CHAR_NOTIFY_DISABLED_EVT,
  /* drive_pidConstants_writeNotifyChar */
  CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR_WRITE_EVT,
  CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR_NOTIFY_DISABLED_EVT,
  /* drive_chassisSpeeds_writeNotifyChar */
  CUSTOM_STM_DRIVE_CHASSISSPEEDS_CHAR_WRITE_EVT,
  CUSTOM_STM_DRIVE_CHASSISSPEEDS_CHAR_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_DRIVE_CHASSISSPEEDS_CHAR_NOTIFY_DISABLED_EVT,
  /* maze_reset_writeChar */
  CUSTOM_STM_MAZE_RESET_CHAR_WRITE_EVT,
  /* maze_cell_notifyChar */
  CUSTOM_STM_MAZE_CELL_CHAR_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_MAZE_CELL_CHAR_NOTIFY_DISABLED_EVT,
  /* maze_mousePos_notifyChar */
  CUSTOM_STM_MAZE_MOUSEPOS_CHAR_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_MAZE_MOUSEPOS_CHAR_NOTIFY_DISABLED_EVT,
  CUSTOM_STM_NOTIFICATION_COMPLETE_EVT,

  CUSTOM_STM_BOOT_REQUEST_EVT
} Custom_STM_Opcode_evt_t;

typedef struct
{
  uint8_t * pPayload;
  uint8_t   Length;
} Custom_STM_Data_t;

typedef struct
{
  Custom_STM_Opcode_evt_t       Custom_Evt_Opcode;
  Custom_STM_Data_t             DataTransfered;
  uint16_t                      ConnectionHandle;
  uint8_t                       ServiceInstance;
  uint16_t                      AttrHandle;
} Custom_STM_App_Notification_evt_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
extern uint8_t SizeMusic_Playsong_Char;
extern uint8_t SizeMusic_Isplaying_Char;
extern uint8_t SizeVision_Rawreadings_Char;
extern uint8_t SizeVision_Distances_Char;
extern uint8_t SizeVision_Calibrate_Char;
extern uint8_t SizeMain_Task_Char;
extern uint8_t SizeMain_Appready_Char;
extern uint8_t SizeMain_Errorcode_Char;
extern uint8_t SizeDrive_Data_Char;
extern uint8_t SizeDrive_Imudata_Char;
extern uint8_t SizeDrive_Pidconstants_Char;
extern uint8_t SizeDrive_Chassisspeeds_Char;
extern uint8_t SizeMaze_Reset_Char;
extern uint8_t SizeMaze_Cell_Char;
extern uint8_t SizeMaze_Mousepos_Char;

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void SVCCTL_InitCustomSvc(void);
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification);
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode,  uint8_t *pPayload);
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*CUSTOM_STM_H */
