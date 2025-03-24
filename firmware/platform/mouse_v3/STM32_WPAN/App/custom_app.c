/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* musicService */
  uint8_t               Music_isplaying_char_Notification_Status;
  /* visionService */
  uint8_t               Vision_rawreadings_char_Notification_Status;
  uint8_t               Vision_distances_char_Notification_Status;
  uint8_t               Vision_calibrate_char_Notification_Status;
  /* mainService */
  uint8_t               Main_task_char_Notification_Status;
  uint8_t               Main_errorcode_char_Notification_Status;
  /* driveService */
  uint8_t               Drive_data_char_Notification_Status;
  uint8_t               Drive_imudata_char_Notification_Status;
  uint8_t               Drive_pidconstants_char_Notification_Status;
  /* mazeService */
  uint8_t               Maze_cell_char_Notification_Status;
  uint8_t               Maze_mousepos_char_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* musicService */
static void Custom_Music_isplaying_char_Update_Char(void);
static void Custom_Music_isplaying_char_Send_Notification(void);
/* visionService */
static void Custom_Vision_rawreadings_char_Update_Char(void);
static void Custom_Vision_rawreadings_char_Send_Notification(void);
static void Custom_Vision_distances_char_Update_Char(void);
static void Custom_Vision_distances_char_Send_Notification(void);
static void Custom_Vision_calibrate_char_Update_Char(void);
static void Custom_Vision_calibrate_char_Send_Notification(void);
/* mainService */
static void Custom_Main_task_char_Update_Char(void);
static void Custom_Main_task_char_Send_Notification(void);
static void Custom_Main_errorcode_char_Update_Char(void);
static void Custom_Main_errorcode_char_Send_Notification(void);
/* driveService */
static void Custom_Drive_data_char_Update_Char(void);
static void Custom_Drive_data_char_Send_Notification(void);
static void Custom_Drive_imudata_char_Update_Char(void);
static void Custom_Drive_imudata_char_Send_Notification(void);
static void Custom_Drive_pidconstants_char_Update_Char(void);
static void Custom_Drive_pidconstants_char_Send_Notification(void);
/* mazeService */
static void Custom_Maze_cell_char_Update_Char(void);
static void Custom_Maze_cell_char_Send_Notification(void);
static void Custom_Maze_mousepos_char_Update_Char(void);
static void Custom_Maze_mousepos_char_Send_Notification(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* musicService */
    case CUSTOM_STM_MUSIC_PLAYSONG_CHAR_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MUSIC_PLAYSONG_CHAR_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_MUSIC_PLAYSONG_CHAR_WRITE_EVT */
      break;

    case CUSTOM_STM_MUSIC_ISPLAYING_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MUSIC_ISPLAYING_CHAR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_MUSIC_ISPLAYING_CHAR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_MUSIC_ISPLAYING_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MUSIC_ISPLAYING_CHAR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_MUSIC_ISPLAYING_CHAR_NOTIFY_DISABLED_EVT */
      break;

    /* visionService */
    case CUSTOM_STM_VISION_RAWREADINGS_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_VISION_RAWREADINGS_CHAR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_VISION_RAWREADINGS_CHAR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_VISION_RAWREADINGS_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_VISION_RAWREADINGS_CHAR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_VISION_RAWREADINGS_CHAR_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_VISION_DISTANCES_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_VISION_DISTANCES_CHAR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_VISION_DISTANCES_CHAR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_VISION_DISTANCES_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_VISION_DISTANCES_CHAR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_VISION_DISTANCES_CHAR_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_VISION_CALIBRATE_CHAR_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_VISION_CALIBRATE_CHAR_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_VISION_CALIBRATE_CHAR_WRITE_EVT */
      break;

    case CUSTOM_STM_VISION_CALIBRATE_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_VISION_CALIBRATE_CHAR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_VISION_CALIBRATE_CHAR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_VISION_CALIBRATE_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_VISION_CALIBRATE_CHAR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_VISION_CALIBRATE_CHAR_NOTIFY_DISABLED_EVT */
      break;

    /* mainService */
    case CUSTOM_STM_MAIN_TASK_CHAR_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MAIN_TASK_CHAR_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_MAIN_TASK_CHAR_WRITE_EVT */
      break;

    case CUSTOM_STM_MAIN_TASK_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MAIN_TASK_CHAR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_MAIN_TASK_CHAR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_MAIN_TASK_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MAIN_TASK_CHAR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_MAIN_TASK_CHAR_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_MAIN_APPREADY_CHAR_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MAIN_APPREADY_CHAR_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_MAIN_APPREADY_CHAR_WRITE_EVT */
      break;

    case CUSTOM_STM_MAIN_ERRORCODE_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MAIN_ERRORCODE_CHAR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_MAIN_ERRORCODE_CHAR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_MAIN_ERRORCODE_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MAIN_ERRORCODE_CHAR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_MAIN_ERRORCODE_CHAR_NOTIFY_DISABLED_EVT */
      break;

    /* driveService */
    case CUSTOM_STM_DRIVE_DATA_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DRIVE_DATA_CHAR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_DRIVE_DATA_CHAR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_DRIVE_DATA_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DRIVE_DATA_CHAR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_DRIVE_DATA_CHAR_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_DRIVE_IMUDATA_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DRIVE_IMUDATA_CHAR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_DRIVE_IMUDATA_CHAR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_DRIVE_IMUDATA_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DRIVE_IMUDATA_CHAR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_DRIVE_IMUDATA_CHAR_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR_WRITE_EVT */
      break;

    case CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR_NOTIFY_DISABLED_EVT */
      break;

    /* mazeService */
    case CUSTOM_STM_MAZE_RESET_CHAR_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MAZE_RESET_CHAR_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_MAZE_RESET_CHAR_WRITE_EVT */
      break;

    case CUSTOM_STM_MAZE_CELL_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MAZE_CELL_CHAR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_MAZE_CELL_CHAR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_MAZE_CELL_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MAZE_CELL_CHAR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_MAZE_CELL_CHAR_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_MAZE_MOUSEPOS_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MAZE_MOUSEPOS_CHAR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_MAZE_MOUSEPOS_CHAR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_MAZE_MOUSEPOS_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MAZE_MOUSEPOS_CHAR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_MAZE_MOUSEPOS_CHAR_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* musicService */
void Custom_Music_isplaying_char_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Music_isplaying_char_UC_1*/

  /* USER CODE END Music_isplaying_char_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MUSIC_ISPLAYING_CHAR, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Music_isplaying_char_UC_Last*/

  /* USER CODE END Music_isplaying_char_UC_Last*/
  return;
}

void Custom_Music_isplaying_char_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Music_isplaying_char_NS_1*/

  /* USER CODE END Music_isplaying_char_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MUSIC_ISPLAYING_CHAR, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Music_isplaying_char_NS_Last*/

  /* USER CODE END Music_isplaying_char_NS_Last*/

  return;
}

/* visionService */
void Custom_Vision_rawreadings_char_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Vision_rawreadings_char_UC_1*/

  /* USER CODE END Vision_rawreadings_char_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_VISION_RAWREADINGS_CHAR, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Vision_rawreadings_char_UC_Last*/

  /* USER CODE END Vision_rawreadings_char_UC_Last*/
  return;
}

void Custom_Vision_rawreadings_char_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Vision_rawreadings_char_NS_1*/

  /* USER CODE END Vision_rawreadings_char_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_VISION_RAWREADINGS_CHAR, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Vision_rawreadings_char_NS_Last*/

  /* USER CODE END Vision_rawreadings_char_NS_Last*/

  return;
}

void Custom_Vision_distances_char_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Vision_distances_char_UC_1*/

  /* USER CODE END Vision_distances_char_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_VISION_DISTANCES_CHAR, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Vision_distances_char_UC_Last*/

  /* USER CODE END Vision_distances_char_UC_Last*/
  return;
}

void Custom_Vision_distances_char_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Vision_distances_char_NS_1*/

  /* USER CODE END Vision_distances_char_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_VISION_DISTANCES_CHAR, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Vision_distances_char_NS_Last*/

  /* USER CODE END Vision_distances_char_NS_Last*/

  return;
}

void Custom_Vision_calibrate_char_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Vision_calibrate_char_UC_1*/

  /* USER CODE END Vision_calibrate_char_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_VISION_CALIBRATE_CHAR, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Vision_calibrate_char_UC_Last*/

  /* USER CODE END Vision_calibrate_char_UC_Last*/
  return;
}

void Custom_Vision_calibrate_char_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Vision_calibrate_char_NS_1*/

  /* USER CODE END Vision_calibrate_char_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_VISION_CALIBRATE_CHAR, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Vision_calibrate_char_NS_Last*/

  /* USER CODE END Vision_calibrate_char_NS_Last*/

  return;
}

/* mainService */
void Custom_Main_task_char_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Main_task_char_UC_1*/

  /* USER CODE END Main_task_char_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MAIN_TASK_CHAR, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Main_task_char_UC_Last*/

  /* USER CODE END Main_task_char_UC_Last*/
  return;
}

void Custom_Main_task_char_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Main_task_char_NS_1*/

  /* USER CODE END Main_task_char_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MAIN_TASK_CHAR, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Main_task_char_NS_Last*/

  /* USER CODE END Main_task_char_NS_Last*/

  return;
}

void Custom_Main_errorcode_char_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Main_errorcode_char_UC_1*/

  /* USER CODE END Main_errorcode_char_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MAIN_ERRORCODE_CHAR, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Main_errorcode_char_UC_Last*/

  /* USER CODE END Main_errorcode_char_UC_Last*/
  return;
}

void Custom_Main_errorcode_char_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Main_errorcode_char_NS_1*/

  /* USER CODE END Main_errorcode_char_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MAIN_ERRORCODE_CHAR, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Main_errorcode_char_NS_Last*/

  /* USER CODE END Main_errorcode_char_NS_Last*/

  return;
}

/* driveService */
void Custom_Drive_data_char_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Drive_data_char_UC_1*/

  /* USER CODE END Drive_data_char_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DRIVE_DATA_CHAR, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Drive_data_char_UC_Last*/

  /* USER CODE END Drive_data_char_UC_Last*/
  return;
}

void Custom_Drive_data_char_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Drive_data_char_NS_1*/

  /* USER CODE END Drive_data_char_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DRIVE_DATA_CHAR, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Drive_data_char_NS_Last*/

  /* USER CODE END Drive_data_char_NS_Last*/

  return;
}

void Custom_Drive_imudata_char_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Drive_imudata_char_UC_1*/

  /* USER CODE END Drive_imudata_char_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DRIVE_IMUDATA_CHAR, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Drive_imudata_char_UC_Last*/

  /* USER CODE END Drive_imudata_char_UC_Last*/
  return;
}

void Custom_Drive_imudata_char_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Drive_imudata_char_NS_1*/

  /* USER CODE END Drive_imudata_char_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DRIVE_IMUDATA_CHAR, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Drive_imudata_char_NS_Last*/

  /* USER CODE END Drive_imudata_char_NS_Last*/

  return;
}

void Custom_Drive_pidconstants_char_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Drive_pidconstants_char_UC_1*/

  /* USER CODE END Drive_pidconstants_char_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Drive_pidconstants_char_UC_Last*/

  /* USER CODE END Drive_pidconstants_char_UC_Last*/
  return;
}

void Custom_Drive_pidconstants_char_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Drive_pidconstants_char_NS_1*/

  /* USER CODE END Drive_pidconstants_char_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Drive_pidconstants_char_NS_Last*/

  /* USER CODE END Drive_pidconstants_char_NS_Last*/

  return;
}

/* mazeService */
void Custom_Maze_cell_char_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Maze_cell_char_UC_1*/

  /* USER CODE END Maze_cell_char_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MAZE_CELL_CHAR, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Maze_cell_char_UC_Last*/

  /* USER CODE END Maze_cell_char_UC_Last*/
  return;
}

void Custom_Maze_cell_char_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Maze_cell_char_NS_1*/

  /* USER CODE END Maze_cell_char_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MAZE_CELL_CHAR, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Maze_cell_char_NS_Last*/

  /* USER CODE END Maze_cell_char_NS_Last*/

  return;
}

void Custom_Maze_mousepos_char_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Maze_mousepos_char_UC_1*/

  /* USER CODE END Maze_mousepos_char_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MAZE_MOUSEPOS_CHAR, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Maze_mousepos_char_UC_Last*/

  /* USER CODE END Maze_mousepos_char_UC_Last*/
  return;
}

void Custom_Maze_mousepos_char_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Maze_mousepos_char_NS_1*/

  /* USER CODE END Maze_mousepos_char_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MAZE_MOUSEPOS_CHAR, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Maze_mousepos_char_NS_Last*/

  /* USER CODE END Maze_mousepos_char_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
