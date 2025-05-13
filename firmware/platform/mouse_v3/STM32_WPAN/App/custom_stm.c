/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.c
  * @author  MCD Application Team
  * @brief   Custom Example Service.
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
#include "common_blesvc.h"
#include "custom_stm.h"

/* USER CODE BEGIN Includes */

#include <micromouse/robot.h>
#include "hardware/drivetrain_impl.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t  CustomMusicserviceHdle;                    /**< musicService handle */
  uint16_t  CustomMusic_Playsong_CharHdle;                  /**< music_playSong_writeChar handle */
  uint16_t  CustomMusic_Isplaying_CharHdle;                  /**< music_isPlaying_notifyChar handle */
  uint16_t  CustomVisionserviceHdle;                    /**< visionService handle */
  uint16_t  CustomVision_Rawreadings_CharHdle;                  /**< vision_rawReadings_notifyChar handle */
  uint16_t  CustomVision_Distances_CharHdle;                  /**< vision_distances_notifyChar handle */
  uint16_t  CustomVision_Calibrate_CharHdle;                  /**< vision_calibrate_writeNotifyChar handle */
  uint16_t  CustomMainserviceHdle;                    /**< mainService handle */
  uint16_t  CustomMain_Task_CharHdle;                  /**< main_task_writeNotifyChar handle */
  uint16_t  CustomMain_Appready_CharHdle;                  /**< main_appReady_writeChar handle */
  uint16_t  CustomMain_Errorcode_CharHdle;                  /**< main_errorCode_notifyChar handle */
  uint16_t  CustomDriveserviceHdle;                    /**< driveService handle */
  uint16_t  CustomDrive_Data_CharHdle;                  /**< drive_data_notifyChar handle */
  uint16_t  CustomDrive_Imudata_CharHdle;                  /**< drive_imuData_notifyChar handle */
  uint16_t  CustomDrive_Pidconstants_CharHdle;                  /**< drive_pidConstants_writeNotifyChar handle */
  uint16_t  CustomDrive_Chassisspeeds_CharHdle;                  /**< drive_chassisSpeeds_writeNotifyChar handle */
  uint16_t  CustomMazeserviceHdle;                    /**< mazeService handle */
  uint16_t  CustomMaze_Reset_CharHdle;                  /**< maze_reset_writeChar handle */
  uint16_t  CustomMaze_Cell_CharHdle;                  /**< maze_cell_notifyChar handle */
  uint16_t  CustomMaze_Mousepos_CharHdle;                  /**< maze_mousePos_notifyChar handle */
/* USER CODE BEGIN Context */
  /* Place holder for Characteristic Descriptors Handle*/

/* USER CODE END Context */
}CustomContext_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define UUID_128_SUPPORTED  1

#if (UUID_128_SUPPORTED == 1)
#define BM_UUID_LENGTH  UUID_TYPE_128
#else
#define BM_UUID_LENGTH  UUID_TYPE_16
#endif

#define BM_REQ_CHAR_SIZE    (3)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET         2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET              1
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
uint8_t SizeMusic_Playsong_Char = 1;
uint8_t SizeMusic_Isplaying_Char = 1;
uint8_t SizeVision_Rawreadings_Char = 4*4;
uint8_t SizeVision_Distances_Char = 4*4;
uint8_t SizeVision_Calibrate_Char = 1;
uint8_t SizeMain_Task_Char = 2;
uint8_t SizeMain_Appready_Char = 1;
uint8_t SizeMain_Errorcode_Char = 1;
uint8_t SizeDrive_Data_Char = 4*sizeof(float)+3*sizeof(float);
uint8_t SizeDrive_Imudata_Char = 6*sizeof(float);
uint8_t SizeDrive_Pidconstants_Char = 6*sizeof(float);
uint8_t SizeDrive_Chassisspeeds_Char = 2*sizeof(float);
uint8_t SizeMaze_Reset_Char = 1;
uint8_t SizeMaze_Cell_Char = 2;
uint8_t SizeMaze_Mousepos_Char = 2;

/**
 * START of Section BLE_DRIVER_CONTEXT
 */
static CustomContext_t CustomContext;

/**
 * END of Section BLE_DRIVER_CONTEXT
 */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *pckt);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/* USER CODE BEGIN PFD */

/* USER CODE END PFD */

/* Private functions ----------------------------------------------------------*/

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

#define COPY_MUSICSERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_MUSIC_PLAYSONG_WRITECHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_MUSIC_ISPLAYING_NOTIFYCHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_VISIONSERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_VISION_RAWREADINGS_NOTIFYCHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x05,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_VISION_DISTANCES_NOTIFYCHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x06,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_VISION_CALIBRATE_WRITENOTIFYCHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x07,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_MAINSERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_MAIN_TASK_WRITENOTIFYCHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x0a,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_MAIN_APPREADY_WRITECHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x0b,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_MAIN_ERRORCODE_NOTIFYCHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x0c,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_DRIVESERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x03,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_DRIVE_DATA_NOTIFYCHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x0f,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_DRIVE_IMUDATA_NOTIFYCHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x10,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_DRIVE_PIDCONSTANTS_WRITENOTIFYCHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x11,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_DRIVE_CHASSISSPEEDS_WRITENOTIFYCHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x12,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_MAZESERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x04,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_MAZE_RESET_WRITECHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x14,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_MAZE_CELL_NOTIFYCHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x15,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_MAZE_MOUSEPOS_NOTIFYCHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x16,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)

/* USER CODE BEGIN PF */

/* USER CODE END PF */

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blecore_aci *blecore_evt;
  aci_gatt_attribute_modified_event_rp0 *attribute_modified;
  aci_gatt_notification_complete_event_rp0    *notification_complete;
  Custom_STM_App_Notification_evt_t     Notification;
  /* USER CODE BEGIN Custom_STM_Event_Handler_1 */

  /* USER CODE END Custom_STM_Event_Handler_1 */

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch (event_pckt->evt)
  {
    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      blecore_evt = (evt_blecore_aci*)event_pckt->data;
      switch (blecore_evt->ecode)
      {
        case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */
          attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blecore_evt->data;
          if (attribute_modified->Attr_Handle == (CustomContext.CustomMusic_Isplaying_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2 */

            /* USER CODE END CUSTOM_STM_Service_1_Char_2 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_1_Char_2_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_1_Char_2_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_MUSIC_ISPLAYING_CHAR_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_1_Char_2_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_1_Char_2_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_MUSIC_ISPLAYING_CHAR_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_1_Char_2_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2_default */

                /* USER CODE END CUSTOM_STM_Service_1_Char_2_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomMusic_Isplaying_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomVision_Rawreadings_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1 */

            /* USER CODE END CUSTOM_STM_Service_2_Char_1 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_2_Char_1_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_VISION_RAWREADINGS_CHAR_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_VISION_RAWREADINGS_CHAR_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_default */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomVision_Rawreadings_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomVision_Distances_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2 */

            /* USER CODE END CUSTOM_STM_Service_2_Char_2 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_2_Char_2_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_2_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_VISION_DISTANCES_CHAR_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_2_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_2_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_VISION_DISTANCES_CHAR_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_2_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_default */

                /* USER CODE END CUSTOM_STM_Service_2_Char_2_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomVision_Distances_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomVision_Calibrate_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_3 */

            /* USER CODE END CUSTOM_STM_Service_2_Char_3 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_3_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_2_Char_3_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_3_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_3_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_VISION_CALIBRATE_CHAR_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_3_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_3_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_3_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_3_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_VISION_CALIBRATE_CHAR_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_3_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_3_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_3_default */

                /* USER CODE END CUSTOM_STM_Service_2_Char_3_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomVision_Calibrate_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomMain_Task_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1 */

            /* USER CODE END CUSTOM_STM_Service_3_Char_1 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_3_Char_1_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_MAIN_TASK_CHAR_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_MAIN_TASK_CHAR_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_default */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomMain_Task_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomMain_Errorcode_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_3 */

            /* USER CODE END CUSTOM_STM_Service_3_Char_3 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_3_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_3_Char_3_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_3_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_3_Char_3_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_MAIN_ERRORCODE_CHAR_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_3_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_3_Char_3_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_3_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_3_Char_3_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_MAIN_ERRORCODE_CHAR_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_3_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_3_Char_3_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_3_default */

                /* USER CODE END CUSTOM_STM_Service_3_Char_3_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomMain_Errorcode_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomDrive_Data_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1 */

            /* USER CODE END CUSTOM_STM_Service_4_Char_1 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_4_Char_1_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_4_Char_1_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_DRIVE_DATA_CHAR_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_4_Char_1_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_4_Char_1_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_DRIVE_DATA_CHAR_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_4_Char_1_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_default */

                /* USER CODE END CUSTOM_STM_Service_4_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomDrive_Data_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomDrive_Imudata_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_2 */

            /* USER CODE END CUSTOM_STM_Service_4_Char_2 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_2_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_4_Char_2_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_2_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_4_Char_2_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_DRIVE_IMUDATA_CHAR_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_2_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_4_Char_2_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_2_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_4_Char_2_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_DRIVE_IMUDATA_CHAR_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_2_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_4_Char_2_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_2_default */

                /* USER CODE END CUSTOM_STM_Service_4_Char_2_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomDrive_Imudata_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomDrive_Pidconstants_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_3 */

            /* USER CODE END CUSTOM_STM_Service_4_Char_3 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_3_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_4_Char_3_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_3_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_4_Char_3_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_3_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_4_Char_3_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_3_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_4_Char_3_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_3_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_4_Char_3_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_3_default */

                /* USER CODE END CUSTOM_STM_Service_4_Char_3_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomDrive_Pidconstants_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomDrive_Chassisspeeds_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_4 */

            /* USER CODE END CUSTOM_STM_Service_4_Char_4 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_4_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_4_Char_4_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_4_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_4_Char_4_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_DRIVE_CHASSISSPEEDS_CHAR_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_4_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_4_Char_4_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_4_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_4_Char_4_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_DRIVE_CHASSISSPEEDS_CHAR_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_4_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_4_Char_4_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_4_default */

                /* USER CODE END CUSTOM_STM_Service_4_Char_4_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomDrive_Chassisspeeds_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomMaze_Cell_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_2 */

            /* USER CODE END CUSTOM_STM_Service_5_Char_2 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_2_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_5_Char_2_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_2_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_5_Char_2_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_MAZE_CELL_CHAR_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_2_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_5_Char_2_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_2_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_5_Char_2_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_MAZE_CELL_CHAR_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_2_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_5_Char_2_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_2_default */

                /* USER CODE END CUSTOM_STM_Service_5_Char_2_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomMaze_Cell_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomMaze_Mousepos_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_3 */

            /* USER CODE END CUSTOM_STM_Service_5_Char_3 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_3_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_5_Char_3_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_3_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_5_Char_3_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_MAZE_MOUSEPOS_CHAR_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_3_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_5_Char_3_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_3_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_5_Char_3_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_MAZE_MOUSEPOS_CHAR_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_3_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_5_Char_3_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_3_default */

                /* USER CODE END CUSTOM_STM_Service_5_Char_3_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomMaze_Mousepos_CharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomMusic_Playsong_CharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */

            if (attribute_modified->Attr_Data_Length == 1) {
              Robot_DelegateReceivedFeedback(FB_TOPIC_RECEIVE_MUSIC_PLAY_SONG, attribute_modified->Attr_Data);
            }

            /* USER CODE END CUSTOM_STM_Service_1_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomMusic_Playsong_CharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomVision_Calibrate_CharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_3_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */

            if (attribute_modified->Attr_Data_Length == 1) {
              Robot_DelegateReceivedFeedback(FB_TOPIC_RECEIVE_VISION_CALIBRATE, attribute_modified->Attr_Data);
            }

            /* USER CODE END CUSTOM_STM_Service_2_Char_3_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomVision_Calibrate_CharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomMain_Task_CharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */

            if (attribute_modified->Attr_Data_Length == 2) {
              Robot_DelegateReceivedFeedback(FB_TOPIC_RECEIVE_MAIN_TASK, attribute_modified->Attr_Data);
            }

            /* USER CODE END CUSTOM_STM_Service_3_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomMain_Task_CharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomMain_Appready_CharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_2_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */

            // When app is ready, refresh all app feedback data.
            Robot_PublishExtraFeedback();

            /* USER CODE END CUSTOM_STM_Service_3_Char_2_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomMain_Appready_CharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomDrive_Pidconstants_CharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_3_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
            
            if (attribute_modified->Attr_Data_Length == 6 * sizeof(float)) {
              const float* pid = (const float*)attribute_modified->Attr_Data;
              DrivetrainImpl_UpdatePIDValues(pid);
            }

            /* USER CODE END CUSTOM_STM_Service_4_Char_3_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomDrive_Pidconstants_CharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomDrive_Chassisspeeds_CharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_4_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */

            if (attribute_modified->Attr_Data_Length == 2 * sizeof(float)) {
              Robot_DelegateReceivedFeedback(FB_TOPIC_RECEIVE_DRIVE_CHASSIS_SPEEDS, attribute_modified->Attr_Data);
            }

            /* USER CODE END CUSTOM_STM_Service_4_Char_4_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomDrive_Chassisspeeds_CharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomMaze_Reset_CharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */

            Robot_DelegateReceivedFeedback(FB_TOPIC_RECEIVE_MAZE_RESET, NULL);

            /* USER CODE END CUSTOM_STM_Service_5_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomMaze_Reset_CharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */
          break;

        case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE :
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_END */
          break;

        case ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */
          break;

		case ACI_GATT_NOTIFICATION_COMPLETE_VSEVT_CODE:
        {
          /* USER CODE BEGIN EVT_BLUE_GATT_NOTIFICATION_COMPLETE_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_NOTIFICATION_COMPLETE_BEGIN */
          notification_complete = (aci_gatt_notification_complete_event_rp0*)blecore_evt->data;
          Notification.Custom_Evt_Opcode = CUSTOM_STM_NOTIFICATION_COMPLETE_EVT;
          Notification.AttrHandle = notification_complete->Attr_Handle;
          Custom_STM_App_Notification(&Notification);
          /* USER CODE BEGIN EVT_BLUE_GATT_NOTIFICATION_COMPLETE_END */

          /* USER CODE END EVT_BLUE_GATT_NOTIFICATION_COMPLETE_END */
          break;
        }

        /* USER CODE BEGIN BLECORE_EVT */

        /* USER CODE END BLECORE_EVT */
        default:
          /* USER CODE BEGIN EVT_DEFAULT */

          /* USER CODE END EVT_DEFAULT */
          break;
      }
      /* USER CODE BEGIN EVT_VENDOR*/

      /* USER CODE END EVT_VENDOR*/
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT_CASES*/

      /* USER CODE END EVENT_PCKT_CASES*/

    default:
      /* USER CODE BEGIN EVENT_PCKT*/

      /* USER CODE END EVENT_PCKT*/
      break;
  }

  /* USER CODE BEGIN Custom_STM_Event_Handler_2 */

  /* USER CODE END Custom_STM_Event_Handler_2 */

  return(return_value);
}/* end Custom_STM_Event_Handler */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void SVCCTL_InitCustomSvc(void)
{

  Char_UUID_t  uuid;
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint8_t max_attr_record;

  /* USER CODE BEGIN SVCCTL_InitCustomSvc_1 */

  /* USER CODE END SVCCTL_InitCustomSvc_1 */

  /**
   *  Register the event handler to the BLE controller
   */
  SVCCTL_RegisterSvcHandler(Custom_STM_Event_Handler);

  /**
   *          musicService
   *
   * Max_Attribute_Records = 1 + 2*2 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for musicService +
   *                                2 for music_playSong_writeChar +
   *                                2 for music_isPlaying_notifyChar +
   *                                1 for music_isPlaying_notifyChar configuration descriptor +
   *                              = 6
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 6;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  COPY_MUSICSERVICE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomMusicserviceHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: musicService, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: musicService \n\r");
  }

  /**
   *  music_playSong_writeChar
   */
  COPY_MUSIC_PLAYSONG_WRITECHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomMusicserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeMusic_Playsong_Char,
                          CHAR_PROP_WRITE,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomMusic_Playsong_CharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : MUSIC_PLAYSONG_CHAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : MUSIC_PLAYSONG_CHAR \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char1/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char1 */
  /**
   *  music_isPlaying_notifyChar
   */
  COPY_MUSIC_ISPLAYING_NOTIFYCHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomMusicserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeMusic_Isplaying_Char,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomMusic_Isplaying_CharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : MUSIC_ISPLAYING_CHAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : MUSIC_ISPLAYING_CHAR \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char2/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char2 */

  /**
   *          visionService
   *
   * Max_Attribute_Records = 1 + 2*3 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for visionService +
   *                                2 for vision_rawReadings_notifyChar +
   *                                2 for vision_distances_notifyChar +
   *                                2 for vision_calibrate_writeNotifyChar +
   *                                1 for vision_rawReadings_notifyChar configuration descriptor +
   *                                1 for vision_distances_notifyChar configuration descriptor +
   *                                1 for vision_calibrate_writeNotifyChar configuration descriptor +
   *                              = 10
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 10;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  COPY_VISIONSERVICE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomVisionserviceHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: visionService, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: visionService \n\r");
  }

  /**
   *  vision_rawReadings_notifyChar
   */
  COPY_VISION_RAWREADINGS_NOTIFYCHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomVisionserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeVision_Rawreadings_Char,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomVision_Rawreadings_CharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : VISION_RAWREADINGS_CHAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : VISION_RAWREADINGS_CHAR \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service2_Char1/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service2_Char1 */
  /**
   *  vision_distances_notifyChar
   */
  COPY_VISION_DISTANCES_NOTIFYCHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomVisionserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeVision_Distances_Char,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomVision_Distances_CharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : VISION_DISTANCES_CHAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : VISION_DISTANCES_CHAR \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service2_Char2/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service2_Char2 */
  /**
   *  vision_calibrate_writeNotifyChar
   */
  COPY_VISION_CALIBRATE_WRITENOTIFYCHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomVisionserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeVision_Calibrate_Char,
                          CHAR_PROP_WRITE | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomVision_Calibrate_CharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : VISION_CALIBRATE_CHAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : VISION_CALIBRATE_CHAR \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service2_Char3/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service2_Char3 */

  /**
   *          mainService
   *
   * Max_Attribute_Records = 1 + 2*3 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for mainService +
   *                                2 for main_task_writeNotifyChar +
   *                                2 for main_appReady_writeChar +
   *                                2 for main_errorCode_notifyChar +
   *                                1 for main_task_writeNotifyChar configuration descriptor +
   *                                1 for main_errorCode_notifyChar configuration descriptor +
   *                              = 9
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 9;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  COPY_MAINSERVICE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomMainserviceHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: mainService, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: mainService \n\r");
  }

  /**
   *  main_task_writeNotifyChar
   */
  COPY_MAIN_TASK_WRITENOTIFYCHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomMainserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeMain_Task_Char,
                          CHAR_PROP_WRITE | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomMain_Task_CharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : MAIN_TASK_CHAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : MAIN_TASK_CHAR \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service3_Char1/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service3_Char1 */
  /**
   *  main_appReady_writeChar
   */
  COPY_MAIN_APPREADY_WRITECHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomMainserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeMain_Appready_Char,
                          CHAR_PROP_WRITE,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomMain_Appready_CharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : MAIN_APPREADY_CHAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : MAIN_APPREADY_CHAR \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service3_Char2/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service3_Char2 */
  /**
   *  main_errorCode_notifyChar
   */
  COPY_MAIN_ERRORCODE_NOTIFYCHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomMainserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeMain_Errorcode_Char,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomMain_Errorcode_CharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : MAIN_ERRORCODE_CHAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : MAIN_ERRORCODE_CHAR \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service3_Char3/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service3_Char3 */

  /**
   *          driveService
   *
   * Max_Attribute_Records = 1 + 2*4 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for driveService +
   *                                2 for drive_data_notifyChar +
   *                                2 for drive_imuData_notifyChar +
   *                                2 for drive_pidConstants_writeNotifyChar +
   *                                2 for drive_chassisSpeeds_writeNotifyChar +
   *                                1 for drive_data_notifyChar configuration descriptor +
   *                                1 for drive_imuData_notifyChar configuration descriptor +
   *                                1 for drive_pidConstants_writeNotifyChar configuration descriptor +
   *                                1 for drive_chassisSpeeds_writeNotifyChar configuration descriptor +
   *                              = 13
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 13;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  COPY_DRIVESERVICE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomDriveserviceHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: driveService, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: driveService \n\r");
  }

  /**
   *  drive_data_notifyChar
   */
  COPY_DRIVE_DATA_NOTIFYCHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomDriveserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeDrive_Data_Char,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomDrive_Data_CharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : DRIVE_DATA_CHAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : DRIVE_DATA_CHAR \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service4_Char1/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service4_Char1 */
  /**
   *  drive_imuData_notifyChar
   */
  COPY_DRIVE_IMUDATA_NOTIFYCHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomDriveserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeDrive_Imudata_Char,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomDrive_Imudata_CharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : DRIVE_IMUDATA_CHAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : DRIVE_IMUDATA_CHAR \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service4_Char2/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service4_Char2 */
  /**
   *  drive_pidConstants_writeNotifyChar
   */
  COPY_DRIVE_PIDCONSTANTS_WRITENOTIFYCHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomDriveserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeDrive_Pidconstants_Char,
                          CHAR_PROP_WRITE | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomDrive_Pidconstants_CharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : DRIVE_PIDCONSTANTS_CHAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : DRIVE_PIDCONSTANTS_CHAR \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service4_Char3/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service4_Char3 */
  /**
   *  drive_chassisSpeeds_writeNotifyChar
   */
  COPY_DRIVE_CHASSISSPEEDS_WRITENOTIFYCHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomDriveserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeDrive_Chassisspeeds_Char,
                          CHAR_PROP_WRITE | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomDrive_Chassisspeeds_CharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : DRIVE_CHASSISSPEEDS_CHAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : DRIVE_CHASSISSPEEDS_CHAR \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service4_Char4/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service4_Char4 */

  /**
   *          mazeService
   *
   * Max_Attribute_Records = 1 + 2*3 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for mazeService +
   *                                2 for maze_reset_writeChar +
   *                                2 for maze_cell_notifyChar +
   *                                2 for maze_mousePos_notifyChar +
   *                                1 for maze_cell_notifyChar configuration descriptor +
   *                                1 for maze_mousePos_notifyChar configuration descriptor +
   *                              = 9
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 9;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  COPY_MAZESERVICE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomMazeserviceHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: mazeService, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: mazeService \n\r");
  }

  /**
   *  maze_reset_writeChar
   */
  COPY_MAZE_RESET_WRITECHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomMazeserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeMaze_Reset_Char,
                          CHAR_PROP_WRITE,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomMaze_Reset_CharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : MAZE_RESET_CHAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : MAZE_RESET_CHAR \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service5_Char1/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service5_Char1 */
  /**
   *  maze_cell_notifyChar
   */
  COPY_MAZE_CELL_NOTIFYCHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomMazeserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeMaze_Cell_Char,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomMaze_Cell_CharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : MAZE_CELL_CHAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : MAZE_CELL_CHAR \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service5_Char2/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service5_Char2 */
  /**
   *  maze_mousePos_notifyChar
   */
  COPY_MAZE_MOUSEPOS_NOTIFYCHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomMazeserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeMaze_Mousepos_Char,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomMaze_Mousepos_CharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : MAZE_MOUSEPOS_CHAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : MAZE_MOUSEPOS_CHAR \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service5_Char3/ */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service5_Char3 */

  /* USER CODE BEGIN SVCCTL_InitCustomSvc_2 */

  /* USER CODE END SVCCTL_InitCustomSvc_2 */

  return;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 *
 */
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_1 */

  /* USER CODE END Custom_STM_App_Update_Char_1 */

  switch (CharOpcode)
  {

    case CUSTOM_STM_MUSIC_PLAYSONG_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomMusicserviceHdle,
                                       CustomContext.CustomMusic_Playsong_CharHdle,
                                       0, /* charValOffset */
                                       SizeMusic_Playsong_Char, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value MUSIC_PLAYSONG_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value MUSIC_PLAYSONG_CHAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_1*/
      break;

    case CUSTOM_STM_MUSIC_ISPLAYING_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomMusicserviceHdle,
                                       CustomContext.CustomMusic_Isplaying_CharHdle,
                                       0, /* charValOffset */
                                       SizeMusic_Isplaying_Char, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value MUSIC_ISPLAYING_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value MUSIC_ISPLAYING_CHAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_2*/
      break;

    case CUSTOM_STM_VISION_RAWREADINGS_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomVisionserviceHdle,
                                       CustomContext.CustomVision_Rawreadings_CharHdle,
                                       0, /* charValOffset */
                                       SizeVision_Rawreadings_Char, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value VISION_RAWREADINGS_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value VISION_RAWREADINGS_CHAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_2_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_2_Char_1*/
      break;

    case CUSTOM_STM_VISION_DISTANCES_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomVisionserviceHdle,
                                       CustomContext.CustomVision_Distances_CharHdle,
                                       0, /* charValOffset */
                                       SizeVision_Distances_Char, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value VISION_DISTANCES_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value VISION_DISTANCES_CHAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_2_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_2_Char_2*/
      break;

    case CUSTOM_STM_VISION_CALIBRATE_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomVisionserviceHdle,
                                       CustomContext.CustomVision_Calibrate_CharHdle,
                                       0, /* charValOffset */
                                       SizeVision_Calibrate_Char, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value VISION_CALIBRATE_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value VISION_CALIBRATE_CHAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_2_Char_3*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_2_Char_3*/
      break;

    case CUSTOM_STM_MAIN_TASK_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomMainserviceHdle,
                                       CustomContext.CustomMain_Task_CharHdle,
                                       0, /* charValOffset */
                                       SizeMain_Task_Char, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value MAIN_TASK_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value MAIN_TASK_CHAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_3_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_3_Char_1*/
      break;

    case CUSTOM_STM_MAIN_APPREADY_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomMainserviceHdle,
                                       CustomContext.CustomMain_Appready_CharHdle,
                                       0, /* charValOffset */
                                       SizeMain_Appready_Char, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value MAIN_APPREADY_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value MAIN_APPREADY_CHAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_3_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_3_Char_2*/
      break;

    case CUSTOM_STM_MAIN_ERRORCODE_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomMainserviceHdle,
                                       CustomContext.CustomMain_Errorcode_CharHdle,
                                       0, /* charValOffset */
                                       SizeMain_Errorcode_Char, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value MAIN_ERRORCODE_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value MAIN_ERRORCODE_CHAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_3_Char_3*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_3_Char_3*/
      break;

    case CUSTOM_STM_DRIVE_DATA_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomDriveserviceHdle,
                                       CustomContext.CustomDrive_Data_CharHdle,
                                       0, /* charValOffset */
                                       SizeDrive_Data_Char, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value DRIVE_DATA_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value DRIVE_DATA_CHAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_4_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_4_Char_1*/
      break;

    case CUSTOM_STM_DRIVE_IMUDATA_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomDriveserviceHdle,
                                       CustomContext.CustomDrive_Imudata_CharHdle,
                                       0, /* charValOffset */
                                       SizeDrive_Imudata_Char, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value DRIVE_IMUDATA_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value DRIVE_IMUDATA_CHAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_4_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_4_Char_2*/
      break;

    case CUSTOM_STM_DRIVE_PIDCONSTANTS_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomDriveserviceHdle,
                                       CustomContext.CustomDrive_Pidconstants_CharHdle,
                                       0, /* charValOffset */
                                       SizeDrive_Pidconstants_Char, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value DRIVE_PIDCONSTANTS_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value DRIVE_PIDCONSTANTS_CHAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_4_Char_3*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_4_Char_3*/
      break;

    case CUSTOM_STM_DRIVE_CHASSISSPEEDS_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomDriveserviceHdle,
                                       CustomContext.CustomDrive_Chassisspeeds_CharHdle,
                                       0, /* charValOffset */
                                       SizeDrive_Chassisspeeds_Char, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value DRIVE_CHASSISSPEEDS_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value DRIVE_CHASSISSPEEDS_CHAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_4_Char_4*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_4_Char_4*/
      break;

    case CUSTOM_STM_MAZE_RESET_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomMazeserviceHdle,
                                       CustomContext.CustomMaze_Reset_CharHdle,
                                       0, /* charValOffset */
                                       SizeMaze_Reset_Char, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value MAZE_RESET_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value MAZE_RESET_CHAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_5_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_5_Char_1*/
      break;

    case CUSTOM_STM_MAZE_CELL_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomMazeserviceHdle,
                                       CustomContext.CustomMaze_Cell_CharHdle,
                                       0, /* charValOffset */
                                       SizeMaze_Cell_Char, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value MAZE_CELL_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value MAZE_CELL_CHAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_5_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_5_Char_2*/
      break;

    case CUSTOM_STM_MAZE_MOUSEPOS_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomMazeserviceHdle,
                                       CustomContext.CustomMaze_Mousepos_CharHdle,
                                       0, /* charValOffset */
                                       SizeMaze_Mousepos_Char, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value MAZE_MOUSEPOS_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value MAZE_MOUSEPOS_CHAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_5_Char_3*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_5_Char_3*/
      break;

    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_2 */

  /* USER CODE END Custom_STM_App_Update_Char_2 */

  return ret;
}
